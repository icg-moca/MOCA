//
//  freenect.h
//  Moc
//
//  Created by Wei Li on 6/4/15.
//

#ifndef Moca_freenect_h
#define Moca_freenect_h

// deps
#include <opencv2/opencv.hpp>
#include <libfreenect2/config.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/packet_processor.h>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/async_packet_processor.h>
#include <turbojpeg.h>
#include <libfreenect2/threading.h>

// time
#include "time.h"

using namespace libfreenect2;

class Freenect {
public :
	libfreenect2::Freenect2								freenect2;
	shared_ptr<libfreenect2::Freenect2Device>			device;
	shared_ptr<libfreenect2::SyncMultiFrameListener>	listener;

	cv::Mat	img_depth, img_ir, img_rgb;

public :
	void InitDevices( void ) {
		img_ir = cv::Mat(0, 0, CV_32FC1);
		img_depth = cv::Mat(0, 0, CV_32FC1);
		img_rgb = cv::Mat(0, 0, CV_8UC3);

		const int numDevices = freenect2.enumerateDevices();
		std::cout << "device connected : " << numDevices << std::endl;

		device.reset( freenect2.openDevice(0) );
		if (!device) {
			cout << "Freenect2 could not open device " << freenect2.getDeviceSerialNumber(0) << endl;
			return;
		}
		cout << "device serial: " << device->getSerialNumber() << endl;

		listener.reset( new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth) );
		
		device->setColorFrameListener(listener.get());
		device->setIrAndDepthFrameListener(listener.get());

		device->start();
	}

	void Loop( void ) {
		libfreenect2::FrameMap frames;
		listener->waitForNewFrame( frames );

		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		//cv::imshow("rgb", cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC3, rgb->data));
		cv::flip(cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC3, rgb->data), img_rgb, 1);
		cv::flip( cv::Mat((int)ir->height, (int)ir->width, CV_32FC1, ir->data ), img_ir, 1 );
		cv::flip( cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data), img_depth, 1 );

		//cv::Size size(1024, 848);
		//cv::Mat dst;
		//cv::resize(img_ir,dst,size);
		//img_ir = dst;

//		cv::resize(img_depth, dst, size);
//		img_depth = dst;

		//cv::imshow("depth", cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data) / 4500.0f);
		listener->release(frames);
	}
};

//========================================================


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

class LIBFREENECT2_API MocaJpegRgbPacketProcessor : public RgbPacketProcessor
{
public:
	MocaJpegRgbPacketProcessor() {

	}

	virtual ~MocaJpegRgbPacketProcessor() {

	}

	virtual void process(const RgbPacket &packet)
	{
		//printf("SHANG %d %d\n", packet.jpeg_buffer, packet.jpeg_buffer_length);

		if (listener_) {
			listener_->onNewFrame(Frame::Color, 0);
		}
	}
};

class LIBFREENECT2_API CLPacketPipeline : public BasePacketPipeline
{
protected:
	const int deviceId;


public:
	CLPacketPipeline(const int deviceId = -1) : deviceId(deviceId)
	{
		initialize();
	}

	virtual ~CLPacketPipeline() {
		delete async_rgb_processor_;
		delete async_depth_processor_;
		delete rgb_processor_;
		delete depth_processor_;
		delete rgb_parser_;
		delete depth_parser_;
	}

	virtual void initialize() {
		rgb_parser_ = new RgbPacketStreamParser();
		depth_parser_ = new DepthPacketStreamParser();

		rgb_processor_ = new MocaJpegRgbPacketProcessor();
		//rgb_processor_ = new TurboJpegRgbPacketProcessor();
		depth_processor_ = createDepthPacketProcessor();
		async_rgb_processor_ = new AsyncPacketProcessor<RgbPacket>(rgb_processor_);
		async_depth_processor_ = new AsyncPacketProcessor<DepthPacket>(depth_processor_);

		rgb_parser_->setPacketProcessor(rgb_processor_);
		depth_parser_->setPacketProcessor(depth_processor_);
	}

	virtual DepthPacketProcessor *createDepthPacketProcessor() {
		OpenCLDepthPacketProcessor *depth_processor = new OpenCLDepthPacketProcessor();
		depth_processor->load11To16LutFromFile("11to16.bin");
		depth_processor->loadXTableFromFile("xTable.bin");
		depth_processor->loadZTableFromFile("zTable.bin");
		DepthPacketProcessor::Config config;
		config.MinDepth = 0.5f;
		config.MaxDepth = 4.5f;
		config.EnableBilateralFilter = false;
		config.EnableEdgeAwareFilter = false;
		depth_processor->setConfiguration(config);
		return depth_processor;
	}
};




class MultiFreenect {
public:
	int numDevices; 
	libfreenect2::Freenect2 freenect[4];
	shared_ptr<libfreenect2::Freenect2Device>			device[4];
	shared_ptr<libfreenect2::SyncMultiFrameListener>	listener[4];

public:
	void InitDevices(void) {
		numDevices = freenect[0].enumerateDevices();
		std::cout << "device connected : " << numDevices << std::endl;
		if (numDevices > 4) {
			numDevices = 4;
		}

		for (int i = 0; i < numDevices; i++) {
			device[i].reset(freenect[i].openDevice(i));
			
			listener[i].reset(new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth));
			device[i]->setColorFrameListener(listener[i].get());
			device[i]->setIrAndDepthFrameListener(listener[i].get());
			
			device[i]->start();
		}
	}

	void Release(void) {
		for (int i = 0; i < numDevices; i++) {
			device[i]->stop();
			device[i]->close();
			device[i].reset();
		}
	}

	void CaptureFrameRGB(int id, cv::Mat &img_rgb) {
		libfreenect2::FrameMap frames;
		listener[id]->waitForNewFrame(frames);

		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];

		img_rgb = cv::Mat(1920, 1080, CV_8UC3);

		cv::flip(cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC3, rgb->data), img_rgb, 1);

		/*
#if 0
		{
			libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
			cv::flip(cv::Mat((int)ir->height, (int)ir->width, CV_32FC1, ir->data), img_ir, 1);
		}
#endif
		{
			libfreenect2::Frame *frame = frames[libfreenect2::Frame::Depth];

			depth = cv::Mat((int)frame->height, (int)frame->width, CV_16UC1);

			for (int y = 0; y < frame->height; y++) {
				for (int x = 0; x < frame->width; x++) {
					((unsigned short*)depth.ptr())[(frame->width - 1 - x) + y * frame->width] = ((float*)frame->data)[x + y * frame->width];
				}
			}
		}
		*/

		listener[id]->release(frames);
	}

	void CaptureFrameIR(int id, cv::Mat &img_ir) {
		libfreenect2::FrameMap frames;
		listener[id]->waitForNewFrame(frames);

		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];

		img_ir = cv::Mat(512, 424, CV_32FC1);

		cv::flip(cv::Mat((int)ir->height, (int)ir->width, CV_32FC1, ir->data), img_ir, 1);

		listener[id]->release(frames);
	}
};

MultiFreenect kinect;

#endif
