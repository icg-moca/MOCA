//
//  sample.h
//  Moc
//
//  Created by Wei Li on 6/4/15.
//

#ifndef Moc_sample_h
#define Moc_sample_h

#include "camera_calibration.h"
#include "vicon_tracker.h"
#include "freenect.h"
#include "fusion.h"

void sample_main_vicon_track(void) {
	VRPN_Tracker vrpn_pattern, vrpn_kinect;
	vrpn_pattern.Create("PATTERNLARGE@192.168.1.136");
	vrpn_kinect.Create("KINECT@192.168.1.136");

	while (1) {
		// update vrpn
		vrpn_pattern.Loop();
		vrpn_kinect.Loop();

		vrpn_pattern.Log();
		vrpn_kinect.Log();
	}
}

void sample_main_kinect(void) {
	//=====================
	// Kinect
	//=====================
	Freenect kinect;
	kinect.InitDevices();

	//=====================
	// Loop
	//=====================
	while (1) {
		// update kinect
		kinect.Loop();
		cv::imshow("ir", kinect.img_ir / 2000.0f);
		cv::waitKey(1);
	}
}

void sample_main_calibrate_vicon_kinect(void) {
	//=====================
	// Vicon
	//=====================
	VRPN_Tracker vrpn_pattern, vrpn_kinect;
	vrpn_pattern.Create("PATTERNLARGE@192.168.1.136");
	vrpn_kinect.Create("KINECT@192.168.1.136");

	//=====================
	// Kinect
	//=====================
	Freenect kinect;
	kinect.InitDevices();

	//=====================
	// Calibrator
	//=====================
	Moc::Calibrator_Kinect_To_Vicon cali;

	// init chessboard
	vector< cv::Point3f > markers(4);
	markers[0].x = -0.213301147460938;
	markers[0].y = 0.400469909667969;
	markers[0].z = -0.270611022949219;

	markers[1].x = 0.153394592285156;
	markers[1].y = 0.385762634277344;
	markers[1].z = 0.190735992431641;

	markers[2].x = 0.140245666503906;
	markers[2].y = -0.396933837890625;
	markers[2].z = 0.177871490478516;

	markers[3].x = -0.226152709960938;
	markers[3].y = -0.383085083007813;
	markers[3].z = -0.281757385253906;

	cali.SetChessboard(cv::Size(11, 8), 0.0655, markers);

	// reset frames
	cali.ResetFrames();
	cali.LoadCali("cam_cali.txt");

	//=====================
	// Loop
	//=====================
	while (1) {
		// update vrpn
		vrpn_pattern.Loop();
		vrpn_kinect.Loop();

		// update kinect
		kinect.Loop();

		if (kinect.img_ir.size().width > 0){
			if (cali.isCalibrated) {
				vector<cv::Point2f> proj;
				Moc::ProjectPoints(cali.board_corners, cali.cam_intr, cali.cam_extr * vrpn_kinect.Mat().inv() * vrpn_pattern.Mat(), proj);
				for (int i = 0; i < proj.size(); i++) {
					cv::Point pt(proj[i].x, proj[i].y);
					cv::Scalar color(0, 0, 255);
					cv::circle(kinect.img_ir, pt, 3, color, 1, CV_AA, 0);
				}
			}

			cv::imshow("ir", kinect.img_ir / 2000.0f);
		}

		int key = cv::waitKey(1);

		// capture new frame
		if (key == '0') {
			cali.CaptureFrame(kinect.img_ir, vrpn_pattern.Mat(), vrpn_kinect.Mat());
		}

		// calibrate kinect and save config
		if (key == '1') {
			cali.Calibrate();
			cali.SaveCali("cam_cali.txt");
		}
	}
}

void sample_main_calibrate_kinect_kinect(void) {
	//=====================
	// Kinect
	//=====================
	Freenect kinect;
	kinect.InitDevices();

	//=====================
	// Calibrator
	//=====================
	vector< Moc::Calibrator_Kinect > cali;
	cali.resize(2);

	for (int i = 0; i < cali.size(); i++) {
		// init chessboard
		cali[i].SetChessboard(cv::Size(11, 8), 0.0655);

		// reset frames
		cali[i].ResetFrames();
	}

	//=====================
	// Loop
	//=====================
	while (1) {
		// update kinect
		kinect.Loop();
		cv::imshow("ir", kinect.img_ir / 2000.0f);
		int key = cv::waitKey(1);

		// capture new frame
		if (key == '0') {
			for (int i = 0; i < cali.size(); i++) {
				cali[i].CaptureFrame(kinect.img_ir);
			}
		}

		// calibrate kinect and save config
		if (key == '1') {
			for (int i = 0; i < cali.size(); i++) {
				cali[i].Calibrate();
			}
		}
	}

	vector< cv::Mat > extr;
	extr.resize(cali.size());
	extr[0] = cv::Mat::eye(4, 4, CV_64F);
	for (int i = 1; i < extr.size(); i++) {
		// improve by projection error
		extr[i] = cali[0].cam_extr[0].Mat() * cali[1].cam_extr[0].Mat().inv();
	}
}

void sample_main_calibrate_vicon_velodyne(void) {
	// velodyne
	vector< cv::Point3f > src;
	src.push_back(cv::Point3f(344.876770, 21.191553, -8.421365));
	src.push_back(cv::Point3f(329.321472, 21.196106, 112.414787));
	src.push_back(cv::Point3f(273.297333, -54.085236, 103.468338));
	src.push_back(cv::Point3f(283.640472, -49.967205, -16.086639));

	// vicon
	vector< cv::Point3f > dst;
	dst.push_back(cv::Point3f(57.7072, 108.886, 180.456));
	dst.push_back(cv::Point3f(176.59, 108.459, 192.929));
	dst.push_back(cv::Point3f(170.349, 38.8436, 247.97));
	dst.push_back(cv::Point3f(51.4353, 40.1155, 236.71));

	cv::Mat M;
	Moc::PairPointsRigidRegistration(src, dst, M);
}

#if 1
void main() {
	//sample_main_vicon_track();
	//sample_main_kinect();
	sample_main_calibrate_vicon_kinect();
	//sample_main_calibrate_kinect_kinect();
	//sample_main_calibrate_vicon_velodyne();
}
#endif

#endif
