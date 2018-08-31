
/*
MultiKinect - Shang
*/
#ifndef MultiK_h
#define MultiK_h
// libfreenect
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
// Multithread
#include <Windows.h>
#include <process.h>
//CV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

void KeyFuc(int key){
	//   ... ADD KEY FUNC HERE 
}

struct FreeArg{
	libfreenect2::Freenect2Device* devices;
	int* idx;
};

//=====================
//	Thread Function
//=====================
unsigned int __stdcall RunFreenect2(PVOID pM)
{
	// connect arg to current func
	FreeArg* tempKinect = (FreeArg*)pM;
	// setup listener

// SETTTING THE CAPTURE MODE YOU WANT
#define ID
#ifdef CID
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
#endif
#ifdef ID
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
#endif
#ifdef I
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Ir);
#endif
	libfreenect2::FrameMap frames;

	if (tempKinect->devices == 0)
	{
		std::cout << "no device connected or failure opening the default one!" << std::endl;
		return -1;
	}

	// RGB and DEPTH listener init
#ifdef CID
	tempKinect->devices->setColorFrameListener(&listener);
#endif
	tempKinect->devices->setIrAndDepthFrameListener(&listener);
	tempKinect->devices->start();

	std::cout << "device serial: " << tempKinect->devices->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << tempKinect->devices->getFirmwareVersion() << std::endl;

	while (1)
	{
		// listen to new frame comes
		listener.waitForNewFrame(frames);
		// put correspondent data to correspondent categories
#ifdef CID
		libfreenect2::Frame *rgb1 = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir1 = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth1 = frames[libfreenect2::Frame::Depth];
		// Display
		char WndCO[512];
		sprintf(WndCO, "Color_%d", *(tempKinect->idx));
		char WndIR[512];
		sprintf(WndIR, "Ir_%d", *(tempKinect->idx));
		char WndDP[512];
		sprintf(WndDP, "Depth_%d", *(tempKinect->idx));
		cv::imshow(WndCO, cv::Mat(rgb1->height, rgb1->width, CV_8UC3, rgb1->data));
		cv::Mat img_ir = cv::Mat(ir1->height, ir1->width, CV_32FC1, ir1->data);
		cv::imshow(WndIR, img_ir / 6000.0f);
		cv::imshow(WndDP, cv::Mat(depth1->height, depth1->width, CV_32FC1, depth1->data) / 1000.0f);
#endif
#ifdef ID
		libfreenect2::Frame *ir1 = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth1 = frames[libfreenect2::Frame::Depth];
		// Display
		char WndIR[512];
		sprintf(WndIR, "Ir_%d", *(tempKinect->idx));
		char WndDP[512];
		sprintf(WndDP, "Depth_%d", *(tempKinect->idx));
		cv::Mat img_ir = cv::Mat(ir1->height, ir1->width, CV_32FC1, ir1->data);
		cv::imshow(WndIR, img_ir / 6000.0f);
		cv::imshow(WndDP, cv::Mat(depth1->height, depth1->width, CV_32FC1, depth1->data) / 1000.0f);
#endif
#ifdef I
		libfreenect2::Frame *ir1 = frames[libfreenect2::Frame::Ir];
		// Display
		char WndIR[512];
		sprintf(WndIR, "Ir_%d", *(tempKinect->idx));
		cv::Mat img_ir = cv::Mat(ir1->height, ir1->width, CV_32FC1, ir1->data);
		cv::imshow(WndIR, img_ir / 6000.0f);
#endif
		int key = cv::waitKey(1);

		//	KEY
		KeyFuc(key);

		//protonect_shutdown = protonect_shutdown || (key1 > 0 && ((key1 & 0xFF) == 27)); // shutdown on escape
		// free space after display the data
		listener.release(frames);
	}

	tempKinect->devices->stop();
	tempKinect->devices->close();

	return 0;
}


//=====================
//		FreenectM
//=====================
class FreenectM {
public:
	int totalK;
	int* idx;
	HANDLE* hKthread;
	FreeArg* freeArg;

public:

	FreenectM(vector<libfreenect2::Freenect2Device*> *dev, int numK)
	{
		//	number of kinect
		idx = new int[numK];
		totalK = numK;
		//  number of arguments need to pass to thread func
		freeArg = new FreeArg[numK];
		// thread handle allocation
		hKthread = new HANDLE[numK];
		for (int i = 0; i < numK; i++){
			freeArg[i].devices = dev->at(i);
			idx[i] = i;
			freeArg[i].idx = &idx[i];

		}
	}


	~FreenectM()
	{

	}

	void Release()
	{
		for (int i = 0; i < totalK; i++)
		{
			CloseHandle(hKthread[i]);
		}
		delete[] hKthread;
		delete[] freeArg;
		delete[] idx;

	}

	//void HandleK(void){
	void RunThreads(void){
		// setup thread for each Kinect
		// use _beginthreadx to allocate individual memory storage (_tiddata block) for each thread
		for (int i = 0; i < totalK; i++){
			// run thread
			hKthread[i] = (HANDLE)_beginthreadex(						// return the HANDLE of thread
				NULL,				// security
				0,			// stack_size "0" = default (1MB)
				RunFreenect2,		// address of function
				&freeArg[i],			// arg list
				0,					// 0 indicates manipulate immediately 
				NULL);				// return thread ID, NULL means no return
		}
		//WaitForSingleObject(hKthread, INFINITE);
		WaitForMultipleObjects(totalK, hKthread, TRUE, INFINITE);

	}
};



#endif // MultiK_h
