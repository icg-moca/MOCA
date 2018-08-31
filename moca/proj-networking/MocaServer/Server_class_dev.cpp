//
// server.cpp
// ~~~~~~~~~~

#if 1
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>

#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <timer.h>
#include <MocaSocket.h>
#include <opencv2/opencv.hpp>
using boost::asio::ip::tcp;

//================
// Sensor
//================
class Sensor {
public:
	std::string	serial;

	int			sensorID;
	int			serverID;				// node computer ID
	int			deviceID;				// local device ID

	bool		connected;

	Sensor() : sensorID(-1), serverID(-1), deviceID(-1), connected(false){
	}
};

//================
// Server
//================
class Server {
public:
	int									portID;
	int									serverID;
	std::vector<int>					connectedSensorIDs;

	boost::shared_ptr<tcp::acceptor>	acceptor;

	Socket								socket;

	std::string							recorded_timestep;

public:
	Server() : portID(-1), serverID(-1) {
	}

	void start_accept(boost::asio::io_service& io_service)
	{
		acceptor.reset(new tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), portID)));
		socket.socket.reset(new tcp::socket(io_service));

		acceptor->async_accept(
			*socket.socket,
			boost::bind(
			&Server::handle_accept,
			this,
			socket.socket,
			boost::asio::placeholders::error
			)
			);
	}

	void handle_accept(boost::shared_ptr<tcp::socket> &socket, const boost::system::error_code &error)
	{
		std::cout << "[SYSTEM] :: SERVER - CLIENT connection has been set up !!!!" << std::endl;
		std::cout << "[SYSTEM] :: Client connected from << " << socket->remote_endpoint().address().to_string().c_str() << " >>" << std::endl;

		if (error) {
			return;
		}

		//image_session_->teststart();
		// Maybe put time sychronization code here...
		boost::asio::ip::tcp::no_delay option(true);
		socket->set_option(option);

		//DEBUG
		std::cout << "COMMAND DEBUG INSTRUCTION..." << std::endl;
		std::cout << "request device" << std::endl;
		std::cout << "start cap" << std::endl;
		std::cout << "rframe ID" << std::endl;
	}

	void wait_new_connection(boost::asio::io_service& io_service) {
		socket.close();
		while (!socket.socket->is_open()) {
			start_accept(io_service);
		}
	}
};

//================
// SensorNetwork
//================
class SensorNetwork {
public:
	std::vector<Sensor>			sensors;
	std::vector<Server>			servers;
	Timer						timer;

	int							withRGB;
public:
	SensorNetwork() {
		// server
		servers.resize(4);
		for (int i = 0; i < servers.size(); i++) {
			servers[i].serverID = i;
		}

		servers[0].portID = 1113;
		servers[1].portID = 1114;
		servers[2].portID = 1115;
		servers[3].portID = 1116;

		// sensor
		sensors.resize(24);
		for (int i = 0; i < sensors.size(); i++) {
			sensors[i].sensorID = i;
		}

		sensors[0].serial = "506904442542";
		sensors[1].serial = "008427750247";
		sensors[2].serial = "501778742542";
		sensors[3].serial = "006039350247";
		sensors[4].serial = "501916642542";
		sensors[5].serial = "026340651247";
		sensors[6].serial = "007697650247";
		sensors[7].serial = "025150151247";

		withRGB = 0;
	}

	void StartAccept(boost::asio::io_service &io_service) {
		for (int i = 0; i < servers.size(); i++) {
			servers[i].start_accept(io_service);
		}
	}

	void WaitForNewConnection(boost::asio::io_service &io_service) {
		for (int i = 0; i < servers.size(); i++) {
			servers[i].wait_new_connection(io_service);
		}
	}

	int FindSensor(const std::string &serial) {
		for (int i = 0; i < sensors.size(); i++) {
			if (sensors[i].serial == serial) {
				return i;
			}
		}
		return -1;
	}

	int FindServer(int port) {
		for (int i = 0; i < servers.size(); i++) {
			if (servers[i].portID == port) {
				return i;
			}
		}
		return -1;
	}

	template<class T>
	bool RequestFrame(int sensorID, std::vector<T> &buffer){
		// locate
		int serverID = sensors[sensorID].serverID;
		int deviceID = sensors[sensorID].deviceID;

		if (serverID == -1 || deviceID == -1){
			return false;
		}
		Server *server = &servers[serverID];

		// request frame on
		if (!server->socket.write("rfo", 4)) {
			return false;
		}

		// send deviceID
		if (!server->socket.write(&deviceID, 4)) {
			return false;
		}

		//read frame
		server->socket.read_buffer(buffer);

		//send
		server->socket.write("rcv", 4);
	}

	void RequestFramesPeriod(int serverID, float start, float end) {
		Server *server = &servers[serverID];

		// request frame : rpf : start : end
		if (!server->socket.write("rpf", 4)) {
			std::cout << "[ERROR IN TRANS] :: send to port" << server->portID << " failed..." << std::endl;
		}
		if (!server->socket.write(&start, sizeof(float))){
			std::cout << "[ERROR IN TRANS] :: send to port" << server->portID << " failed..." << std::endl;
		}
		if (!server->socket.write(&end, sizeof(float))){
			std::cout << "[ERROR IN TRANS] :: send to port" << server->portID << " failed..." << std::endl;
		}

		for (int i = 0; i < server->connectedSensorIDs.size(); ++i) {
			//////////////////////////////////////////////////////
			int targetSensorID = server->connectedSensorIDs[i];
			if (sensors[targetSensorID].serverID != serverID || sensors[targetSensorID].deviceID != i) {
				std::cout << "[REQUEST PERIOD FRAMES] :: can't find sensorID in current node com device info buffer @NODE PORT: " << serverID << std::endl;
				continue;
			}
			std::cout
				<< "sensor : " << sensors[targetSensorID].sensorID << ", "
				<< "server : " << sensors[targetSensorID].serverID << ", "
				<< "device : " << sensors[targetSensorID].deviceID << std::endl;

			//////////////////////////////////////////////////////
			{
				char data[4];
				if (!server->socket.read(&data, 4)) {
					std::cout << "[Error:REQUEST PERIOD FRAMES] read tag" << std::endl;
				}

				if (std::string(data) != "kin") {
					std::cout << "[Error:REQUEST PERIOD FRAMES] data tag" << data << std::endl;
					continue;
				}
			}

			//////////////////////////////////////////////////////
			// read index buffer
			std::vector<double> index_data;
			if (!server->socket.read_buffer(index_data)) {
				std::cout << "[ERROR IN read_buffer] :: read from port " << server->portID << "failed..." << std::endl;
			}
			std::cout << "[TotalFrame] :: " << index_data.size() << std::endl;
			if (index_data.empty()) {
				std::cout << "[ERROR IN data_index buffer] :: index_Data size is empty... " << std::endl;
			}


			//////////////////////////////////////////////////////
			// save index buffer to file
			{
				char saveDepIdx_path[MAX_PATH];
				sprintf(saveDepIdx_path, "ReceivedData/%s", server->recorded_timestep);
				CreateDirectory(saveDepIdx_path, NULL);
				sprintf(saveDepIdx_path, "ReceivedData/%s/K%d", server->recorded_timestep, targetSensorID);
				CreateDirectory(saveDepIdx_path, NULL);
				sprintf(saveDepIdx_path, "ReceivedData/%s/K%d/tIdx.txt", server->recorded_timestep, targetSensorID);

				std::filebuf fb;
				fb.open(saveDepIdx_path, std::ios::out);

				char timestr[128];
				for (int k = 0; k < index_data.size(); ++k) {
					std::ostream os(&fb);
					sprintf(timestr, "%.2f", index_data[k]);
					os << timestr << std::endl;
				}

				fb.close();
			}

#if 1
			//////////////////////////////////////////////////////
			// recv images
			{

				std::vector<char> image_buffer;

				std::cout << "[index data] :: " << index_data.size() << std::endl;
				for (int j = 0; j < index_data.size() * 2; ++j) {  //for jpeg+bin
				//for (int j = 0; j < index_data.size(); ++j) {
					// read filename
					std::vector<char> filename;
					server->socket.read_buffer(filename);
					std::cout << "[DEBUG :: RECV IMGS] :: Current file name is: " << filename.data() << std::endl;

					// read image buffer
					server->socket.read_buffer(image_buffer);

					// open file
					char openpath[MAX_PATH];
					sprintf(openpath, "ReceivedData/%s/K%d/%s", server->recorded_timestep, targetSensorID, filename.data());
					std::ofstream output_file(openpath, std::ios_base::binary); //output_file.open();
					if (!output_file)
					{
						std::cout << "[OFSTREAM OUTPUT] :: failed to open " << openpath << std::endl;
						return;
					}

					// save image buffer to file
					output_file.write(image_buffer.data(), image_buffer.size());
					if (output_file.tellp() == (std::fstream::pos_type)(std::streamsize)image_buffer.size()) {
						std::cout << "[OFSTREAM OUTPUT] :: received " << output_file.tellp() << " bytes.\n";
					}
				}
			}
#endif			
			// send rcv tag
			server->socket.write("rcv", 4);
		}
	}

	// Request all frames
	void RequestFramesOnClip(int serverID) {
		Server *server = &servers[serverID];

		// request frame : rpf : start : end
		if (!server->socket.write("gis", 4)) {
			std::cout << "[ERROR IN GIS] :: send to port" << server->portID << " failed..." << std::endl;
		}

		for (int i = 0; i < server->connectedSensorIDs.size(); ++i) {
			//////////////////////////////////////////////////////
			int targetSensorID = server->connectedSensorIDs[i];
			if (sensors[targetSensorID].serverID != serverID || sensors[targetSensorID].deviceID != i) {
				std::cout << "[REQUEST PERIOD FRAMES] :: can't find sensorID in current node com device info buffer @NODE PORT: " << serverID << std::endl;
				continue;
			}
			std::cout
				<< "sensor : " << sensors[targetSensorID].sensorID << ", "
				<< "server : " << sensors[targetSensorID].serverID << ", "
				<< "device : " << sensors[targetSensorID].deviceID << std::endl;

			//////////////////////////////////////////////////////
			{
				char data[4];
				if (!server->socket.read(&data, 4)) {
					std::cout << "[Error:REQUEST PERIOD FRAMES] read tag" << std::endl;
				}

				if (std::string(data) != "kin") {
					std::cout << "[Error:REQUEST PERIOD FRAMES] data tag" << data << std::endl;
					continue;
				}
			}

			//////////////////////////////////////////////////////
			// read index buffer
			std::vector<double> index_data;
			if (!server->socket.read_buffer(index_data)) {
				std::cout << "[ERROR IN read_buffer] :: read from port " << server->portID << "failed..." << std::endl;
			}
			std::cout << "[TotalFrame] :: " << index_data.size() << std::endl;
			if (index_data.empty()) {
				std::cout << "[ERROR IN data_index buffer] :: index_Data size is empty... " << std::endl;
			}


			//////////////////////////////////////////////////////
			// save index buffer to file
			{
				char saveDepIdx_path[MAX_PATH];
				sprintf(saveDepIdx_path, "ReceivedData/%s", server->recorded_timestep);
				CreateDirectory(saveDepIdx_path, NULL);
				sprintf(saveDepIdx_path, "ReceivedData/%s/K%d", server->recorded_timestep, targetSensorID);
				CreateDirectory(saveDepIdx_path, NULL);
				sprintf(saveDepIdx_path, "ReceivedData/%s/K%d/tIdx.txt", server->recorded_timestep, targetSensorID);

				std::filebuf fb;
				fb.open(saveDepIdx_path, std::ios::out);

				char timestr[128];
				for (int k = 0; k < index_data.size(); ++k) {
					std::ostream os(&fb);
					sprintf(timestr, "%.2f", index_data[k]);
					os << timestr << std::endl;
				}

				fb.close();
			}

			//////////////////////////////////////////////////////
			// recv images
			{

				std::vector<char> image_buffer;

				std::cout << "[index data] :: " << index_data.size() << std::endl;
				for (int j = 0; j < index_data.size() * withRGB; ++j) {  //for jpeg+bin
				//for (int j = 0; j < index_data.size(); ++j) {
					// read filename
					std::vector<char> filename;
					server->socket.read_buffer(filename);
					std::cout << "[DEBUG :: RECV IMGS] :: Current file name is: " << filename.data() << std::endl;

					// read image buffer
					server->socket.read_buffer(image_buffer);

					// open file
					char openpath[MAX_PATH];
					sprintf(openpath, "ReceivedData/%s/K%d/%s", server->recorded_timestep, targetSensorID, filename.data());
					std::ofstream output_file(openpath, std::ios_base::binary); //output_file.open();
					if (!output_file)
					{
						std::cout << "[OFSTREAM OUTPUT] :: failed to open " << openpath << std::endl;
						return;
					}

					// save image buffer to file
					output_file.write(image_buffer.data(), image_buffer.size());
					if (output_file.tellp() == (std::fstream::pos_type)(std::streamsize)image_buffer.size()) {
						std::cout << "[OFSTREAM OUTPUT] :: received " << output_file.tellp() << " bytes.\n";
					}
				}
			}
			// send rcv tag
			//server->socket.write("rcv", 4);
		}
	}

	void RequestDevices(int serverID) {
		std::cout << "Request Devices : on NODE COM " << serverID << std::endl;

		Server *server = &servers[serverID];

		// request
		if (!server->socket.write("rdv", 4)) {
			return;
		}

		// num of sensors
		int numLocalSensors = 0;
		if (!server->socket.read(&numLocalSensors, 4)) {
			return;
		}
		std::cout << "[ :: ] TOTAL DEV NUM IS " << numLocalSensors << std::endl;

		// sensor info
		std::vector<char> buffer(numLocalSensors * 16);
		if (!server->socket.read(buffer.data(), buffer.size())) {
			return;
		}

		// parse serial and device id
		const char *ptr = buffer.data();
		for (int i = 0; i < numLocalSensors; ++i) {
			int deviceID;
			memcpy(&deviceID, ptr, 4);
			ptr += 4;

			char serial[16];
			memcpy(serial, ptr, 12);
			serial[12] = 0;
			ptr += 12;

			int sensorID = FindSensor(serial);
			if (sensorID >= 0) {
				sensors[sensorID].serverID = serverID;
				sensors[sensorID].deviceID = deviceID;
				// store current node com avaliable sensorID for fast indexing
				server->connectedSensorIDs.push_back(sensorID);

				printf("%d : ( %d, %d ) : %s\n", sensorID, serverID, deviceID, serial);
			}
		}
	}

	void StartRecord(int serverID) {
		std::cout << "Start Capture : " << serverID << std::endl;
		servers[serverID].socket.write("scp", 4);
		tm ct;
		_getsystime(&ct);
		char time[MAX_PATH];
		sprintf(time, "%d%d%d%d%d%d", ct.tm_year + 1900, ct.tm_mon + 1, ct.tm_mday, ct.tm_hour, ct.tm_min, ct.tm_sec);
		servers[serverID].socket.write_string(time);

		servers[serverID].recorded_timestep = time;
	}

	void StartRecordDepthOnly(int serverID) {
		std::cout << "Start Capture <<Depth Only>>: " << serverID << std::endl;
		servers[serverID].socket.write("sdp", 4);
		tm ct;
		_getsystime(&ct);
		char time[MAX_PATH];
		sprintf(time, "%d%d%d%d%d%d", ct.tm_year + 1900, ct.tm_mon + 1, ct.tm_mday, ct.tm_hour, ct.tm_min, ct.tm_sec);
		servers[serverID].socket.write_string(time);

		servers[serverID].recorded_timestep = time;
	}

	void StopRecord(int serverID) {
		std::cout << "Stop Capture : " << serverID << std::endl;
		servers[serverID].socket.write("stp", 4);
	}

	void MeasureLatecy(int serverID){
		Server *server = &servers[serverID];
		// request
		if (!server->socket.write("syn", 4)) {
			return;
		}
		
		for (int i = 0; i < 450; i++) {
			double readti = 0.0;
			server->socket.read(&readti, 8);
			//std::cout << ".........test.................." << readti << std::endl;
			server->socket.write(&readti, 8);
		}
		double cur_origin = timer.start;
		server->socket.write(&cur_origin, 8);
		//server->socket.read(&c_send_timestep, 8);
		//std::cout << "[SERVER :: MEASURE LATENCY] :: client -> server latency is: " << latency / 500 << std::endl;

	}
};

const int TEST_NODE = 1;
int main()
{
	try
	{
		// Maybe need 2 io_services, one for client func, one for server func
		boost::asio::io_service io_service;
		// Idea from http://thisthread.blogspot.com/2011/04/multithreading-with-asio.html
		boost::asio::io_service::work work(io_service);
		//boost::asio::strand strand(io_service);

		SensorNetwork sn;
		sn.timer.Start();
		sn.StartAccept(io_service);

		boost::thread_group threads;
		// Service must be run before detecting cmd input
		for (int i = 0; i < 4; i++) {
			threads.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
		}

		char cmd_buf[1024];
		std::vector<char> recv_buf; size_t buf_size;
		while (std::cin.getline(cmd_buf, 1024)) {
			std::string cmd_ = cmd_buf;
			if (cmd_.find("rframe") < cmd_.size()){
				std::string num_str = cmd_.substr(cmd_.find("rframe") + 7, cmd_.size() - cmd_.find("rframe") - 7);
				int globalID = std::atoi(num_str.c_str());
				std::cout << "[SYSTEM] :: SENDING 'REQUEST FRAME ON' GLOBAL ID: " << globalID << std::endl;
				sn.RequestFrame(globalID, recv_buf);
			}
			else if (cmd_.find("rpframe") < cmd_.size()){
				if (cmd_.size() > 8){
					std::string num_str = cmd_.substr(cmd_.find("rpframe") + 8, cmd_.size() - cmd_.find("rpframe") - 8);
					size_t space = num_str.find_first_of(",");
					std::string start_str = num_str.substr(0, space);
					std::cout << start_str << std::endl;
					std::string end_str = num_str.substr(space + 1, num_str.size());
					std::cout << end_str << std::endl;
					float start = (float)std::atof(start_str.c_str());
					float end = (float)std::atof(end_str.c_str());
					std::cout << "[SYSTEM] :: SENDING 'REQUEST FRAME ON PERIOD'  Period time: " << space << std::endl;
					for (int i = 0; i < TEST_NODE; i++){
						sn.RequestFramesPeriod(i, start, end);
					}
				}
				else{
					std::cout << "[USAGE] :: rpframe start-time,end-time " << std::endl;
				}
			}
			else if (cmd_ == "getimgs") {
				std::cout << "[SYSTEM] :: SENDING 'GET IMAGES' COMMAND... " << std::endl;
				sn.RequestFramesOnClip(0);
				sn.withRGB = 0;  // reset
			}
			else if (cmd_ == "request frame"){
				// io_service.post(boost::bind(&SensorNetwork::RequestFrame, &sn, 0));
			}
			else if (cmd_ == "rd"){
				sn.RequestDevices(0);
				//sn.MeasureLatecy(0);
				//sn.RequestDevices(1);
				//double readti = 0.0;
				//sn.servers[0].socket.read(&readti, 8);
				//std::cout << ".........test.................." << readti << std::endl;
				//sn.servers[0].socket.write(&readti, 8);
			}
			else if (cmd_ == "start cap") {
				std::cout << "[SYSTEM] :: SENDING 'START CAP' COMMAND... " << std::endl;
				sn.StartRecord(0);
				sn.withRGB = 2;
			}
			else if (cmd_ == "sdp") {
				std::cout << "[SYSTEM] :: SENDING 'START CAP' COMMAND... " << std::endl;
				sn.StartRecordDepthOnly(0);
				sn.withRGB = 1;
			}
			else if (cmd_ == "stop cap") {
				std::cout << "[SYSTEM] :: SENDING 'STOP CAPTURE' COMMAND... " << std::endl;
				// stp
				sn.StopRecord(0);
			}
			else if (cmd_ == "sc") {
				std::cout << "[SYSTEM] :: SENDING 'SET CONFIG' COMMAND... " << std::endl;
				//checkcmd.Print("Set config time is...");
				// cnf
				//image_session_->send_config(id_);
				sn.timer.Print("dfdksajfkkdsaf");
			}
			else if (cmd_ == "sn") {
				//std::cout << "[SYSTEM] :: SENDING 'SYNCHRONIZE NODES' COMMAND... " << std::endl;
				//checkcmd.Print("Synchronize nodes cmd time is...");
				// syn
				sn.MeasureLatecy(0);
				
			}
			else {
				std::cout << "[SYSTEM] :: [Port]" << " :: INVALID COMMAND.. Please enter command again..." << std::endl;
				
			}
			//io_service.post(boost::bind(&tcp_server::parsing_cmd, &server0, cmd_line));
		}

		io_service.stop();
		threads.join_all();
	}

	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
#endif
