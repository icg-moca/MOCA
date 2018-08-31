#if 1
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost\bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/mutex.hpp>
#include <libfreenect2/config.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/packet_processor.h>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/async_packet_processor.h>
#include <libfreenect2/rgb_packet_stream_parser.h>
#include <libfreenect2/depth_packet_stream_parser.h>
#include <libfreenect2/threading.h>
#include <timer.h>
#include <png.h>
#include <turbojpeg.h>
#include <IP_v4.h>
#include <MocaSocket.h>
#include <ICMPping.h>
using namespace libfreenect2;
//--------------------------VARIABLES------------------------------
// set protonect won't be shutdown in current configuration
bool protonect_shutdown = false;
bool bkey_capture_frame = false;			// save to imgs
bool bkey_request_frame = false;			// request newest frame
// Create freenect instances
const int MAX_KINECT_CONNECTION = 4;
const int MAX_VIEW_NAME_LENGTH = 64;

static Freenect2 freenect2[MAX_KINECT_CONNECTION];
std::shared_ptr<Freenect2Device> dev[MAX_KINECT_CONNECTION];
std::shared_ptr<SyncMultiFrameListener> listener[MAX_KINECT_CONNECTION];

void jpeg_decompress(size_t jpeg_size, unsigned char *jpeg_buffer, unsigned char *dst_buffer, int width, int height, int pixelFormat = TJPF_BGRX) {
	tjhandle decompressor = tjInitDecompress();
	if (decompressor == 0) {
		std::cout << "Failed to initialize TurboJPEG decompressor! TurboJPEG error: '" << tjGetErrorStr() << std::endl;
		return;
	}

	if (tjDecompress2(decompressor, jpeg_buffer, jpeg_size, dst_buffer, width, width * tjPixelSize[pixelFormat], height, pixelFormat, 0)) {
		std::cout << "Failed to decompress rgb image! TurboJPEG error: '" << tjGetErrorStr() << std::endl;
	}

	if (tjDestroy(decompressor) == -1) {
		std::cout << "Failed to destroy TurboJPEG decompressor! TurboJPEG error: '" << tjGetErrorStr() << std::endl;
	}
}
//======================
// NodeSensor
//======================
class NodeSensor {
public:
	int				deviceID;					// local id
	std::string		serial;						// serial num

	float			save_period;
	Timer			timer;
	int				save_count;
	std::vector<double>									timesteps;

	std::string											save_path;
	std::vector<unsigned short>							frame;
	int												dirt;

public:
	NodeSensor() : deviceID(-1), save_period(0), save_count(0), dirt(0) {
	}
};

//======================
// Client
//======================
// videoID, deviceID, poseID
const char DEPTH_PATH[MAX_PATH] = "CapturedData/Video%d/K%d/Pose_%d.png";
const char COLOR_PATH[MAX_PATH] = "CapturedData/Video%d/K%d/Pose_%d.jpeg";
const char DEPTH_FILE_SUFFIX[MAX_PATH] = "Pose_%d.png";
const char COLOR_FILE_SUFFIX[MAX_PATH] = "Pose_%d.jpeg";
// videoID, deviceID
const char TIMESTEP_PATH[MAX_PATH] = "CapturedData/Video%d/K%d/tIdx.txt";


class Client {
public:
	Socket												socket;

	std::string											ip_addr;
	std::string											port;

	int													numDevices;
	NodeSensor											sensors[MAX_KINECT_CONNECTION];

	boost::mutex										mutex;

	bool												rgb_record;
	bool												dep_record;
	bool												dep_only;
	Timer												timer;
	double												timeOffset;

	int													clips_count;

public:
	Client(char* ip, char* port) : rgb_record(false), dep_record(false), dep_only(false), timeOffset(0), clips_count(0) {
		this->ip_addr = ip;
		this->port = port;
		timer.Start();
		LaunchDev();
	}

	~Client() {
		for (int i = 0; i < numDevices; ++i) {
			dev[i]->stop();
			dev[i]->close();
		}
	}

	void Connect(boost::asio::io_service& io_service) {
		boost::asio::ip::tcp::resolver resolver(io_service);
		boost::asio::ip::tcp::resolver::query query(ip_addr, port);
		boost::asio::ip::tcp::resolver::iterator endpoint_iter = resolver.resolve(query);
		boost::asio::ip::tcp::resolver::iterator end;

		//connect
		socket.socket.reset(new boost::asio::ip::tcp::socket(io_service));
		boost::system::error_code error = boost::asio::error::host_not_found;
		while (error && endpoint_iter != end)
		{
			socket.close();
			socket.connect(endpoint_iter, error);
		}
		socket.setOption();
		std::cout << "[Client] :: Connect to MASTER COM, IP: " << socket.socket->remote_endpoint().address().to_string() << " Successfully! " << std::endl;
	}

	void LaunchDev() {
		// enum devices  (numK guides device iteration)
		numDevices = freenect2[0].enumerateDevices();
		std::cout << "numDevices : " << numDevices << std::endl;
		if (numDevices > MAX_KINECT_CONNECTION) {
			numDevices = MAX_KINECT_CONNECTION;
		}

		// <<< KINECT SETTING >>> //
		// open devices + configure args
		DepthPacketProcessor::Config config;
		config.MinDepth = 0.5f;
		config.MaxDepth = 15.0f;
		config.EnableBilateralFilter = true;
		config.EnableEdgeAwareFilter = true;
		if (numDevices > 0) {
			for (int i = 0; i < numDevices; ++i) {
				OpenCLPacketPipeline *pipeline = new OpenCLPacketPipeline(i);
				pipeline->getDepthPacketProcessor()->setConfiguration(config);
				dev[i].reset(freenect2[i].openDevice(i, pipeline));
				if (!dev[i]) {
					std::cout << "no device connected or failure opening the default one!" << std::endl;
					continue;
				}

				sensors[i].deviceID = i;
				sensors[i].serial = dev[i]->getSerialNumber();
			}

			for (int i = 0; i < numDevices; ++i) {
				listener[i].reset(new SyncMultiFrameListener(Frame::Depth | Frame::Color));
				dev[i]->setColorFrameListener(listener[i].get());
				dev[i]->setIrAndDepthFrameListener(listener[i].get());
				dev[i]->start();
			}
			std::cout << "[CLIENT :: RUN KINECT] :: running devices...." << std::endl;
		}
	}

	void WaitForNewConnection(boost::asio::io_service& io_service) {
		socket.close();
		while (!socket.socket->is_open()) {
			Connect(io_service);
		}
	}

	void Request_device(void) {
		std::cout << "[CLIENT :: REQUEST DEVICE] :: start enumerate devices...." << std::endl;
		// send total number of device
		socket.write(&numDevices, 4);
		boost::asio::streambuf stream_request;
		std::ostream request(&stream_request);
		for (int i = 0; i < numDevices; ++i) {
			request.write(reinterpret_cast<char*>(&sensors[i].deviceID), 4);									// local id
			request.write(sensors[i].serial.c_str(), 12);
			std::cout << "[Client :: RequestDevice] :: Kinect localID: " << i << std::endl;
			std::cout << "                          :: Serial number: " << dev[i]->getSerialNumber() << std::endl;
		}
		socket.write(stream_request);
	}

	void MeasrueOffset(double &Offset) {
		double cur_localOrigin = timer.Record();
		double cur_hostOrigin = 0.0;
		socket.read(&cur_hostOrigin, 8);

		Offset = cur_hostOrigin - cur_localOrigin;
	}

	template<typename T>
	int SaveDepthPng(char *filename, std::vector<T> &buffer) {
#if 1
		FILE *fp = fopen(filename, "wb");
		if (fp){
			fwrite(buffer.data(), sizeof(T), buffer.size(), fp);
			fclose(fp);
		}
		return 1;
#else
		png_image image;
		memset(&image, 0, sizeof(image));
		image.version = PNG_IMAGE_VERSION;
		image.width = 512;
		image.height = 424;
		image.format = PNG_FORMAT_FLAG_LINEAR;

		int res = png_image_write_to_file(&image, filename, 0, buffer.data(), 0, 0);

		png_image_free(&image);
		return res;
#endif
	}

	int SaveColorJpeg(char *filename, size_t bufferSize, void *buffer) {
		FILE *fp = fopen(filename, "wb");
		if (fp){
			fwrite(buffer, 1, bufferSize, fp);
			fclose(fp);
		}
		return 1;
	}

	void CreatePath(std::string &fullpath, int deviceID) {
		char prefix_path[MAX_PATH];
		sprintf(prefix_path, "CapturedData/Video%d", clips_count);
		CreateDirectory(prefix_path, NULL);
		sprintf(prefix_path, "CapturedData/Video%d/K%d", clips_count, deviceID);
		CreateDirectory(prefix_path, NULL);
		std::cout << prefix_path << std::endl;
		fullpath = prefix_path;
	}

	void KinectLoop(int deviceID) {
		std::cout << "[Client :: Thread] :: master request local deviceID: >> " << deviceID << " <<" << std::endl;
		sensors[deviceID].save_count = 0;

		std::vector<unsigned short> latestDepthFrame;
		while (1) {
			if (rgb_record || dep_record || sensors[deviceID].dirt) {
				FrameMap frame;
				listener[deviceID]->waitForNewFrame(frame);
				
				// save depth to buf
				Frame *depth = frame[Frame::Depth];
				latestDepthFrame.resize(depth->height * depth->width);
				for (int y = 0; y < depth->height; y++) {
					for (int x = 0; x < depth->width; x++) {
						latestDepthFrame[(depth->width - 1 - x) + y * depth->width] = ((float*)(depth->data))[x + y * depth->width];
					}
				}
				// save color to buf
				Frame *color = frame[Frame::Color];

				if (sensors[deviceID].dirt) {
					socket.write_buffer(latestDepthFrame);
					char md[16];
					socket.read(md, 4);
					sensors[deviceID].dirt = 0;
				}

				// save depth content
				if (dep_record) {
					double record_time = timer.Record() + timeOffset;	// add timeoffset
					sensors[deviceID].timesteps.push_back(record_time);
					// debug info
					std::cout << "[Client :: Record_Frame] :: master request local deviceID: >> " << deviceID << " << " << record_time << std::endl;

					char saveDepthPath[MAX_PATH];
					std::sprintf(saveDepthPath, DEPTH_PATH, clips_count, deviceID, sensors[deviceID].timesteps.size() - 1);
					SaveDepthPng(saveDepthPath, latestDepthFrame);
					std::cout << saveDepthPath << std::endl;
					
				}	
				// save rgb content
				if (rgb_record) {
					char saveColorPath[MAX_PATH];
					std::sprintf(saveColorPath, COLOR_PATH, clips_count, deviceID, sensors[deviceID].timesteps.size() - 1);
					SaveColorJpeg(saveColorPath, color->bufferSize, color->data);
					std::cout << saveColorPath << std::endl;
				}

				listener[deviceID]->release(frame);
			}
		}
	}

	//write to idx file
	void SaveTimestep(int deviceID) {
		char saveDepIdx_path[MAX_PATH];
		sprintf(saveDepIdx_path, TIMESTEP_PATH, clips_count, deviceID);

		std::filebuf fb;
		fb.open(saveDepIdx_path, std::ios::out);

		char timestr[128];
		for (int j = 0; j < sensors[deviceID].timesteps.size(); ++j) {
			std::ostream os(&fb);
			sprintf(timestr, "%.2f", sensors[deviceID].timesteps[j]);
			os << timestr << std::endl;
		}
		fb.close();
		std::cout << "[SAVE TIMESTEP] :: Save Timestep index file done... " << std::endl;
	}

	void RunDevices() {
		std::vector<boost::thread> thrds;
		for (int i = 0; i < numDevices; i++) {
			thrds.push_back(boost::thread(&Client::KinectLoop, this, i));
		}
		std::for_each(thrds.begin(), thrds.end(), std::mem_fn(&boost::thread::detach));
	}

	void SendImage(int clipID, int deviceID, int poseID, const char* imgtype_path, const char* filesuffix) {
		// send filename
		char filename[MAX_PATH];
		sprintf(filename, filesuffix, poseID);
		std::cout << "send image : " << filename << std::endl;
		socket.write_string(filename);

		// open image
		char filepath[MAX_PATH];
		// clips_count must minus1...
		sprintf(filepath, imgtype_path, clipID, deviceID, poseID);
		std::ifstream source_file(filepath, std::ios_base::binary | std::ios_base::ate);
		if (!source_file) {
			std::cout << "[SYSTEM] :: failed to open " << filepath << std::endl;
			return;
		}

		// read image
		size_t filesize = source_file.tellg();
		source_file.seekg(0);
		if (filesize <= 0) {
			std::cout << "[SYSTEM] file is empty" << std::endl;
			return;
		}
		std::vector<char> buffer(filesize);
		source_file.read(buffer.data(), (std::streamsize)buffer.size());

		// send image
		if (!socket.write_buffer(buffer))
		{
			std::cout << "[SYSTEM] :: [I/O] :: send error:" << std::endl;
			return;
		}
	}

	void Request_periodFrames(float start, float end, int deviceID) {
		if (end <= start) {
			return;
		}

		std::cout << "[DEBUG] :: Successfully get time period!!!" << start << " :: " << end << std::endl;

		// read index buffer
		std::vector<double> sequence_buf;
		{
			char saveDepthIdx_path[MAX_PATH];
			sprintf(saveDepthIdx_path, TIMESTEP_PATH, clips_count, deviceID);

			std::ifstream file(saveDepthIdx_path);
			std::string str;
			while (std::getline(file, str)) {
				sequence_buf.push_back(std::atof(str.c_str()));
			}
		}

		// send kin tag
		socket.write("kin", 4);

		// get frames in range
		std::vector<double> readyToSendSeq;
		std::vector<int>   readyToSendIdx;
		for (int i = 0; i < sequence_buf.size(); i++) {
			if (sequence_buf[i] >= start * 1000 && sequence_buf[i] < end * 1000) {
				readyToSendSeq.push_back(sequence_buf[i]);
				readyToSendIdx.push_back(i);
			}
		}

		// write time stamps
		socket.write_buffer(readyToSendSeq);

		// send correspondent images
		for (int i = 0; i < readyToSendIdx.size(); i++) {
			SendImage(clips_count, deviceID, readyToSendIdx[i], DEPTH_PATH, DEPTH_FILE_SUFFIX);
			SendImage(clips_count, deviceID, readyToSendIdx[i], COLOR_PATH, COLOR_FILE_SUFFIX);
		}

		// rcv tag
		char recv_tag[4];
		socket.read(recv_tag, 4);
		if (recv_tag != "rcv") {
			std::cout << "[ERROR IN RECV IMAGES] :: Kinect deviceID." << deviceID << " didn't get rcv tag..." << std::endl;
			return;
		}
	}

	void RequestPeriod(float start, float end) {
		for (int i = 0; i < numDevices; i++) {
			Request_periodFrames(start, end, i);
		}
	}

	void Request_clip(int deviceID) {
		// read index buf
		std::cout << "[DEBUG] :: Successfully get GETALLIMGS command " << std::endl;
		std::vector<double> sequence_buf;
		{
			char saveDepthIdx_path[MAX_PATH];
			sprintf(saveDepthIdx_path, TIMESTEP_PATH, clips_count, deviceID);

			std::ifstream file(saveDepthIdx_path);
			std::string str;
			while (std::getline(file, str)) {
				sequence_buf.push_back(std::atof(str.c_str()));
			}
		}

		// send kin tag
		socket.write("kin", 4);

		// write time stamps
		socket.write_buffer(sequence_buf);

		// send correspondent images
		for (int i = 0; i < sequence_buf.size(); i++) {
			SendImage(clips_count, deviceID, i, DEPTH_PATH, DEPTH_FILE_SUFFIX);
			if (dep_only == false) {
				SendImage(clips_count, deviceID, i, COLOR_PATH, COLOR_FILE_SUFFIX);
			}
		}

		// rcv tag
		//char recv_tag[4];
		//socket.read(recv_tag, 4);
		//if (recv_tag != "rcv") {
		//	std::cout << "[ERROR IN RECV IMAGES] :: Kinect deviceID." << deviceID << " didn't get rcv tag..." << std::endl;
		//	return;
		//}
	}

	void RequestClip() {
		for (int i = 0; i < numDevices; i++) {
			Request_clip(i);
		}
	}

	void Stop_kinect(void) {
		for (int i = 0; i < numDevices; ++i) {
			dev[i]->stop();
			dev[i]->close();
		}
		numDevices = 0;
	}

	void Parse_cmd(boost::asio::io_service &io_service, std::string& cmd) {
		// request frame on certain kinect
		if (cmd == "rfo") {
			int deviceID = -1;
			socket.read(&deviceID, sizeof(int));
			sensors[deviceID].dirt = true;
			while (sensors[deviceID].dirt) {
				boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
			}
		}
		// request period frames on certain kinect
		else if (cmd == "rpf") {
			float start = -1.0f, end = -1.0f;
			socket.read(&start, sizeof(float));
			socket.read(&end, sizeof(float));
			std::cout << "[REQUEST PERIOD TIME] :: START-> " << start << ", END-> " << end << std::endl;
			RequestPeriod(start, end);
		}
		// request devices
		else if (cmd == "rdv") {
			Request_device();
		}
		// start record
		else if (cmd == "scp") {
			for (int i = 0; i < numDevices; i++) {
				sensors[i].timesteps.clear();
				CreatePath(sensors[i].save_path, i);
			}

			std::string time;
			socket.read_string(time);
			rgb_record = true;
			dep_record = true;
		}
		// start record depth only
		else if (cmd == "sdp") {
			for (int i = 0; i < numDevices; i++) {
				sensors[i].timesteps.clear();
				CreatePath(sensors[i].save_path, i);
			}

			std::string time;
			socket.read_string(time);
			dep_record = true;
			rgb_record = false;
			dep_only = true;				// indicates only cap depth (logic flag)
		}
		// stop record
		else if (cmd == "stp") {
			rgb_record = false;
			dep_record = false;
			for (int i = 0; i < numDevices; i++) {
				SaveTimestep(i);
			}
		}
		// syn nodes
		else if (cmd == "syn") {
			//*
			// calculate deltaT
			double deltaT = 0.0;
			{
				// warm up
				//double del = 0.0;
				for (int i = 0; i < 50; i++) {
					double start = 0.0;
					socket.write(&start, 8);
					socket.read(&start, 8);
				}
				double start = timer.Record();
				for (int i = 0; i < MEASURETIMES; ++i) {
					socket.write(&start, 8);
					socket.read(&start, 8);
				}
				double end = timer.Record();
				deltaT = (end - start) / MEASURETIMES;
				std::cout << "[CLIENT :: Propagation LATENCY] :: client -> server is: " << deltaT << std::endl;
			}
			//deltaT = deltaT;
			// calculate ave Offset
			double offset = 0.0;
			{
				MeasrueOffset(offset);
			}
			std::cout << "[CLIENT :: offset LATENCY] :: client -> server originOffset is: " << offset << std::endl;

			// TimeOffset = (hostOrigin + deltaT/2 - localOrigin)
			timeOffset = offset + deltaT / 2.0;
			std::cout << "[CLIENT :: FINAL timeOffset] :: client -> server timeOffset is: " << timeOffset << std::endl;

			//*/
		}
		// get all the valid images on certain clips
		else if (cmd == "gis") {
			RequestClip();
			dep_only = false;
		}
	}

	void run(boost::asio::io_service &io_service) {
		RunDevices();

		while (1) {
			char cmd[4];
			if (socket.read(cmd, 4)) {
				std::string cmd_str = cmd;
				//std::cout << "CMD " << cmd << std::endl;
				Parse_cmd(io_service, cmd_str);
			}
			else {
				WaitForNewConnection(io_service);
			}
		}
	}
};

int main() {
	try {
		boost::asio::io_service io_service;
		Client client("192.168.10.1", "1114");
		//Client client("127.0.0.1", "1113");
		client.Connect(io_service);
		client.run(io_service);
		io_service.run();

	}
	catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
#endif
