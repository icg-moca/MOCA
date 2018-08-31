//
//  freenect.h
//  Moc
//
//  Created by Wei Li on 6/4/15.
//

#ifndef Moc_freenect_h
#define Moc_freenect_h

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/packet_processor.h>
#include <libfreenect2/async_packet_processor.h>

using namespace libfreenect2;

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

		rgb_processor_ = new TurboJpegRgbPacketProcessor();
		//rgb_processor_ = new TurboJpegRgbPacketProcessor();
		depth_processor_ = createDepthPacketProcessor();
		async_rgb_processor_ = new AsyncPacketProcessor<RgbPacket>(rgb_processor_);
		async_depth_processor_ = new AsyncPacketProcessor<DepthPacket>(depth_processor_);

		rgb_parser_->setPacketProcessor(async_rgb_processor_);
		depth_parser_->setPacketProcessor(async_depth_processor_);
	}

	virtual DepthPacketProcessor *createDepthPacketProcessor() {
		OpenCLDepthPacketProcessor *depth_processor = new OpenCLDepthPacketProcessor();
		depth_processor->load11To16LutFromFile("11to16.bin");
		depth_processor->loadXTableFromFile("xTable.bin");
		depth_processor->loadZTableFromFile("zTable.bin");
		DepthPacketProcessor::Config config;
		config.MinDepth = 0.5f;
		config.MaxDepth = 10.5f;
		config.EnableBilateralFilter = false;
		config.EnableEdgeAwareFilter = false;
		depth_processor->setConfiguration(config);
		return depth_processor;
	}
};

class Freenect{
public:
	libfreenect2::Freenect2								freenect2;
	shared_ptr<libfreenect2::Freenect2Device>			device;
	shared_ptr<libfreenect2::SyncMultiFrameListener>	listener;
	libfreenect2::FrameMap frames;

	cv::Mat	img_depth, img_ir, img_rgb;

public:
	~Freenect() {
		if (device) {
			device->close();
		}
	}

	void InitDevices(void) {
		img_ir = cv::Mat(0, 0, CV_32FC1);
		img_depth = cv::Mat(0, 0, CV_32FC1);
		img_rgb = cv::Mat(0, 0, CV_8UC3);

		const int numDevices = freenect2.enumerateDevices();
		std::cout << "device connected : " << numDevices << std::endl;

		device.reset(freenect2.openDevice(0, new CLPacketPipeline ));
		if (!device) {
			cout << "Freenect2 could not open device " << freenect2.getDeviceSerialNumber(0) << endl;
			return;
		}
		cout << "device serial: " << device->getSerialNumber() << endl;

		//listener.reset(new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth));
		listener.reset(new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Depth));

		//device->setColorFrameListener(listener.get());
		device->setIrAndDepthFrameListener(listener.get());

		device->start();
	}

	void Loop(void) {
		listener->waitForNewFrame(frames);

		//libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		//libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		//cv::imshow("rgb", cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC3, rgb->data));
		//cv::flip(cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC3, rgb->data), img_rgb, 1);
		//cv::flip(cv::Mat((int)ir->height, (int)ir->width, CV_32FC1, ir->data), img_ir, 1);
		cv::flip(cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data), img_depth, 1);

		//cv::imshow("depth", cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data) / 4500.0f);
		listener->release(frames);
	}
};


#endif
