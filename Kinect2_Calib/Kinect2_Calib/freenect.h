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

class Freenect {
public :
	libfreenect2::Freenect2										freenect2;
	vector<shared_ptr<libfreenect2::Freenect2Device>>			devices;
	vector<shared_ptr<libfreenect2::SyncMultiFrameListener>>	listeners;

	cv::Mat		img_depth, img_ir;

public :
	void InitDevices( void ) {

		const int numDevices = freenect2.enumerateDevices();

		std::cout << "device connected : " << numDevices << std::endl;

		devices.reserve(numDevices);
		listeners.reserve(numDevices);

		for(int i = 0; i < numDevices; ++i)
		{
			shared_ptr<libfreenect2::Freenect2Device> device(freenect2.openDevice(i));

			if(!device)
			{
				cout << "Freenect2 could not open device " << freenect2.getDeviceSerialNumber(i) << endl;
				continue;
			}

			shared_ptr<libfreenect2::SyncMultiFrameListener> listener = make_shared<libfreenect2::SyncMultiFrameListener>(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

			cout << "device serial: " << device->getSerialNumber() << endl;

			device->setColorFrameListener(listener.get());
			device->setIrAndDepthFrameListener(listener.get());

			devices.emplace_back(device);
			listeners.emplace_back(listener);

			device->start();
		}

		img_ir		= cv::Mat( 0, 0, CV_32FC1 );
		img_depth	= cv::Mat( 0, 0, CV_32FC1 );
	}

	void Loop( void ) {
		for(int i = 0; i < devices.size(); i++) {
			libfreenect2::FrameMap frames;
			listeners[i]->waitForNewFrame( frames );
			//libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
			libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
			libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

			//cv::imshow("rgb", cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC3, rgb->data));
			cv::flip( cv::Mat((int)ir->height, (int)ir->width, CV_32FC1, ir->data ), img_ir, 1 );
			cv::flip( cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data), img_depth, 1 );

			//cv::imshow("depth", cv::Mat((int)depth->height, (int)depth->width, CV_32FC1, depth->data) / 4500.0f);
			listeners[i]->release(frames);
		}
	}
};


#endif
