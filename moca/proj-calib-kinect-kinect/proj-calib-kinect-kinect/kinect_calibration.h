#pragma once
//
//  sample.h
//  Moc
//
//  Created by Wei Li on 6/4/15.
//


#include "camera_calibration.h"
#include "freenect.h"
#include "fusion.h"

void normlizeVec(double v[3]) {
	double len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

	if (fabs(len) > 1e-6) {
		v[0] /= len;
		v[1] /= len;
		v[2] /= len;
	}
	else {
		v[0] = 0;
		v[1] = 0;
		v[2] = 0;
	}
}

void crossProduct(const double u[3], const double v[3], double w[3]) {
	w[0] = u[1] * v[2] - u[2] * v[1];
	w[1] = u[2] * v[0] - u[0] * v[2];
	w[2] = u[0] * v[1] - u[1] * v[0];
}

void normalizeMatrix(cv::Mat &mat) {
	double M[3][3];

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			M[i][j] = mat.at<double>(i, j);
		}
	}

	normlizeVec(M[2]);
	crossProduct(M[2], M[0], M[1]);
	normlizeVec(M[1]);
	crossProduct(M[1], M[2], M[0]);
	normlizeVec(M[0]);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			mat.at<double>(i, j) = M[i][j];
		}
	}

	mat.at<double>(3, 3) = 1.0;
}

void SplatDepthMap(
	cv::Mat &depth, const Moca::CameraParameter &cam_ir,
	cv::Mat &rgb, const Moca::CameraParameter &cam_rgb,
	cv::Mat &depth_rgb
) {
	cv::Mat M_ir_to_rgb = cam_rgb.extr * cam_ir.extr.inv();

	// splat to rgb depth
	depth_rgb = cv::Mat(rgb.rows, rgb.cols, CV_32FC1);

	for (int y = 0; y < depth_rgb.rows; y++) {
		for (int x = 0; x < depth_rgb.cols; x++) {
			depth_rgb.at<float>(y, x) = 1e6;
		}
	}

	for (int y = 0; y < depth.rows; y++) {
		for (int x = 0; x < depth.cols; x++) {
			float d = depth.at<float>(y, x);
			if (d < 10) {
				continue;
			}

			cv::Point3f p(x, y, 1);

			p.x = (p.x - cam_ir.intr.IntrVec().at<double>(2)) / cam_ir.intr.IntrVec().at<double>(0);
			p.y = (p.y - cam_ir.intr.IntrVec().at<double>(3)) / cam_ir.intr.IntrVec().at<double>(1);

			p = p * (d * 0.001f);

			p = M_ir_to_rgb * p;

			p.x /= p.z;
			p.y /= p.z;

			p.x = p.x * cam_rgb.intr.IntrVec().at<double>(0) + cam_rgb.intr.IntrVec().at<double>(2);
			p.y = p.y * cam_rgb.intr.IntrVec().at<double>(1) + cam_rgb.intr.IntrVec().at<double>(3);

			for (int _x = -16; _x <= +16; _x++) {
				for (int _y = -16; _y <= +16; _y++) {
					int qx = p.x + _x;
					int qy = p.y + _y;

					if (qx >= 0 && qx < depth_rgb.cols && qy >= 0 && qy < depth_rgb.rows) {

						float &nd = depth_rgb.at<float>(qy, qx);
						if (nd > p.z) {
							nd = p.z;
						}
					}
				}
			}
		}
	}

	for (int y = 0; y < depth_rgb.rows; y++) {
		for (int x = 0; x < depth_rgb.cols; x++) {
			if (depth_rgb.at<float>(y, x) > 10) {
				depth_rgb.at<float>(y, x) = 0;
			}
		}
	}
}


//=====================
// Test Stereo Calibration
//=====================
void sample_main_calibrate_kinect_kinect_stereo(void) {
	//=====================
	// Kinect
	//=====================
	MultiFreenect kinect;
	kinect.InitDevices();

	//=====================
	// Pattern
	//=====================
	Moca::Pattern pattern2d;
	{
		// 1 --- 2 \
		// |     |  4
		// 0 --- 3 /
		pattern2d.SetChessboard2D(cv::Size(7, 6), 0.09); // unit : meter
	}

	//=====================
	// Calibration
	//=====================
	Moca::CameraParameter cam_rgb0, cam_rgb1;

	Moca::Calibratior_Stereo cali_stereo;

	bool isCalibrated = false;

	// image corners
	vector< cv::Point2f > corners_rgb0, corners_rgb1, corners;

	cam_rgb0.LoadCali("KINECT0_ir_cali.txt");
	cam_rgb1.LoadCali("KINECT6_ir_cali.txt");

	cv::Mat colorMap0, colorMap1;

	cv::Mat mat_0_to_1;
	//Moca::LoadMat("stereo_0_to_7.txt", mat_0_to_1);

	//=====================
	// Loop
	//=====================
	while (1) {
		//printf( "/////////////////////////////////////////////////////////\n" );
		// update kinect
		kinect.CaptureFrameIR(0, colorMap0);
		kinect.CaptureFrameIR(1, colorMap1);

		int key = cv::waitKey(1);

		// capture new frame
		if (key == '1') {
			bool success0 = Moca::CaptureIRFrame(colorMap0, pattern2d.boardSize, 4, corners_rgb0);
			bool success1 = Moca::CaptureIRFrame(colorMap1, pattern2d.boardSize, 4, corners_rgb1);

			colorMap0 = colorMap0 / 2000.0f;
			colorMap1 = colorMap1 / 2000.0f;
			if (success0) {
				Moca::DisplayCorners(colorMap0, pattern2d.boardSize, corners_rgb0, "corner0");
			}
			if (success1) {
				Moca::DisplayCorners(colorMap1, pattern2d.boardSize, corners_rgb1, "corner1");
			}
			if (success0 && success1) {
				cali_stereo.CaptureFrame(colorMap0.size(), colorMap1.size(), corners_rgb0, corners_rgb1);
			}
		}

		// calibrate kinect and save config
		if (key == '2') {
			// extr : rgb->ir
			mat_0_to_1 = cali_stereo.CalibrateCam0To1(cam_rgb0, cam_rgb1, pattern2d);

			Moca::SaveMat("stereo_0_to_6_ir.txt", mat_0_to_1);
			isCalibrated = true;
		}

		if (key == '0') {
			if (Moca::CaptureIRFrame(colorMap0, pattern2d.boardSize, 5, corners)) {
				cv::Mat extr1;
				Moca::EstimateCameraPose(pattern2d.corners, corners, cam_rgb0.intr, extr1);

				Moca::ProjectPoints(pattern2d.corners, corners, cam_rgb1.intr, mat_0_to_1 * extr1);

				Moca::DrawCircles(colorMap1, corners);

				cv::imshow("test", colorMap1 / 2000.0f);
			}
		}

		{
			//cv::imshow("rgb", kinect.img_rgb);
			cv::imshow("rgb0", colorMap0 / 2000.0f);
			cv::imshow("rgb1", colorMap1 / 2000.0f);
		}
	}
}

//=====================
// Test Stereo Calibration
//=====================
void sample_main_calibrate_kinect_stereo(void) {
	//=====================
	// Kinect
	//=====================
	Freenect kinect;
	kinect.InitDevices();

	//=====================
	// Pattern
	//=====================
	Moca::Pattern pattern2d;
	pattern2d.SetChessboard2D(cv::Size(7, 6), 0.09);

	//=====================
	// Calibration
	//=====================
	Moca::CameraParameter cam_rgb, cam_ir;
	Moca::Calibrator_Camera cali_rgb, cali_ir;
	Moca::Calibratior_Stereo cali_stereo;

	bool isCalibrated = false;

	// image corners
	vector< cv::Point2f > corners_ir, corners_rgb;

	//isCalibrated = true;

	cam_rgb.LoadCali("stereo_rgb_cali.txt");
	cam_ir.LoadCali("stereo_ir_cali.txt");

	cam_ir.intr.InitUndistortMap(cv::Size(512, 424));
	cam_rgb.intr.InitUndistortMap(cv::Size(1920, 1080));

	//=====================
	// Loop
	//=====================
	while (1) {
		// update kinect
		kinect.Loop();

		cv::imshow("ir", kinect.img_ir / 2000.0f);

		int key = cv::waitKey(1);

		// capture new frame
		if (key == '1') {
			bool success_rgb = Moca::CaptureRGBFrame(kinect.img_rgb, pattern2d.boardSize, 5, corners_rgb);
			bool success_ir = Moca::CaptureIRFrame(kinect.img_ir, pattern2d.boardSize, 4, corners_ir);

			cout << "capture rgb :" << endl;
			if (success_rgb) {
				Moca::DisplayCorners(kinect.img_rgb, pattern2d.boardSize, corners_rgb, "corner_rgb");
				cali_rgb.CaptureFrame(kinect.img_rgb.size(), corners_rgb);
			}

			cout << "capture ir :" << endl;
			if (success_ir) {
				cv::Mat img_ir = kinect.img_ir / 2000.0f;
				Moca::DisplayCorners(img_ir, pattern2d.boardSize, corners_ir, "corner_ir");
				cali_ir.CaptureFrame(kinect.img_ir.size(), corners_ir);
			}

			cout << "capture stereo :" << endl;
			if (success_rgb && success_ir) {
				cali_stereo.CaptureFrame(kinect.img_rgb.size(), kinect.img_ir.size(), corners_rgb, corners_ir);
			}
		}

		// calibrate kinect and save config
		if (key == '2') {
			if (cali_rgb.imgPoints.size() > 0 && cali_ir.imgPoints.size() > 0) {
				// intr : rgb
				cali_rgb.Calibrate(cam_rgb, pattern2d);

				// intr : ir
				cali_ir.Calibrate(cam_ir, pattern2d);

				// extr : rgb->ir
				cali_stereo.Calibrate(cam_rgb, cam_ir, pattern2d);

				cam_rgb.SaveCali("stereo_rgb_cali.txt");
				cam_ir.SaveCali("stereo_ir_cali.txt");
				isCalibrated = true;
			}
		}

		if (key == '9') {
			if (Moca::CaptureRGBFrame(kinect.img_rgb, pattern2d.boardSize, 4, corners_rgb)) {

				cv::Mat extr;
				Moca::EstimateCameraPose(pattern2d.corners, corners_rgb, cam_rgb.intr, extr);

				Moca::ProjectPoints(pattern2d.corners, corners_ir, cam_ir.intr, cam_ir.extr * extr);

				Moca::DrawCircles(kinect.img_ir, corners_ir);
				cv::imshow("proj_u", kinect.img_ir / (255.0f*20.0f));
			}
		}

		if (key == '0') {
			cv::Mat depth;
			//Moca::UndistortImage(kinect.img_depth, depth, cam_ir.intr, cv::INTER_LINEAR);

			cv::Mat ir;
			//Moca::UndistortImage(kinect.img_ir, ir, cam_ir.intr, cv::INTER_LINEAR);

			cv::Mat rgb;
			//Moca::UndistortImage(kinect.img_rgb, rgb, cam_rgb.intr, cv::INTER_LINEAR);

			//cv::imshow("irr", ir/(255.0f*20.0f));

			if (Moca::CaptureIRFrame(ir, pattern2d.boardSize, 4, corners_ir)) {
				corners_rgb.resize(corners_ir.size());

				vector<cv::Point3f> points(corners_ir.size());
				for (int i = 0; i < corners_ir.size(); i++) {
					float x = corners_ir[i].x;
					float y = corners_ir[i].y;
					float d = ((float*)depth.data)[(int)x + (int)y * depth.size().width] * 0.001f;

					x -= cam_ir.intr.cameraMatrix.at<double>(0, 2);
					y -= cam_ir.intr.cameraMatrix.at<double>(1, 2);
					x /= cam_ir.intr.cameraMatrix.at<double>(0, 0);
					y /= cam_ir.intr.cameraMatrix.at<double>(1, 1);

					points[i] = cam_ir.extr.inv() * cv::Point3d(x * d, y * d, d);
				}
				Moca::ProjectPoints(points, corners_rgb, cam_rgb.intr, cam_rgb.extr);
				Moca::DrawCircles(kinect.img_rgb, corners_rgb);
				cv::imshow("proj", kinect.img_rgb);

				for (int i = 0; i < corners_ir.size(); i++) {
					cv::Point3f v = points[i];

					v.x /= v.z;
					v.y /= v.z;

					v.x *= cam_rgb.intr.cameraMatrix.at<double>(0, 0);
					v.y *= cam_rgb.intr.cameraMatrix.at<double>(1, 1);
					v.x += cam_rgb.intr.cameraMatrix.at<double>(0, 2);
					v.y += cam_rgb.intr.cameraMatrix.at<double>(1, 2);

					corners_rgb[i].x = v.x;
					corners_rgb[i].y = v.y;
				}
				Moca::DrawCircles(kinect.img_rgb, corners_rgb);
				cv::imshow("proj_u", kinect.img_rgb);
			}
		}
	}
}



