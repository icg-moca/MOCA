//
//  sample.h
//  Moc
//
//  Created by Wei Li on 6/4/15.
//


#include "camera_calibration.h"
#include "vicon_tracker.h"
#include "freenect.h"
#include "fusion.h"

static void VRPN_CALLBACK sample_callback(void* user_data, const vrpn_TRACKERCB tData) {
	VRPN_Tracker *tracker = (VRPN_Tracker*)user_data;

	printf("[Sensor %d : %s]: \n", tData.sensor, tracker->m_id.c_str());
	printf("time : %ld %ld\n", tData.msg_time.tv_sec, tData.msg_time.tv_usec);
	printf("t : %f %f %f\n", tData.pos[0], tData.pos[1], tData.pos[2]);
	printf("q : %f %f %f %f\n\n", tData.quat[0], tData.quat[1], tData.quat[2], tData.quat[3]);
}


cv::Mat vrpn_data_to_mat(vrpn_TRACKERCB data) {
	return TranslateRotate(cv::Point3d(data.pos[0], data.pos[1], data.pos[2]), QuatToMat(data.quat[0], data.quat[1], data.quat[2], data.quat[3]));
}

static void VRPN_CALLBACK sample_cali_callback(void* user_data, const vrpn_TRACKERCB tData) {
	cv::Mat *mat = (cv::Mat*)user_data;

	*mat = vrpn_data_to_mat(tData);
}

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
void sample_main_calibrate_vicon_kinect_stereo(void) {
	//=====================
	// Kinect
	//=====================
	Freenect kinect;
	kinect.InitDevices();

	const int numSensors = 10;
	string sensors[numSensors];

	sensors[0] = "008427750247";
	sensors[1] = "506904442542";
	sensors[2] = "501778742542";
	sensors[3] = "006039350247";
	sensors[4] = "501916642542";
	sensors[5] = "026266651247";
	sensors[6] = "007697650247";
	sensors[7] = "025150151247";

	sensors[8] = "008753550747";
	sensors[9] = "010005150747";

	string serial = kinect.device->getSerialNumber();
	int sensorID = -1;
	for (int i = 0; i < numSensors; i++) {
		if (sensors[i] == serial) {
			sensorID = i;
			break;
		}
	}
	if (sensorID < 0) {
		return;
	}

	char cstr[256];
	sprintf_s(cstr, "KINECT%d", sensorID);

	std::string kinect_name = cstr;
	cout << "////////////////////////////" << endl << "Output name : " << kinect_name << endl;

	//=====================
	// Vicon
	//=====================
	const char *ip = "192.168.10.1";
	VRPN_Tracker vrpn_pattern, vrpn_kinect;
	cv::Mat mat_pattern = cv::Mat_<double>(4, 4);
	cv::Mat mat_kinect = cv::Mat_<double>(4, 4);
	vrpn_pattern.Create("PATTERN", ip, &mat_pattern, sample_cali_callback);
	vrpn_kinect.Create(kinect_name.c_str(), ip, &mat_kinect, sample_cali_callback);

	cv::Point3f plane_o, plane_s, plane_t, plane_n;
	float len_s, len_t;

	//=====================
	// Pattern
	//=====================
	Moca::Pattern pattern2d, pattern3d;
	{
		// 1 --- 2 \
		// |     |  4
		// 0 --- 3 /
		// vsk unit : mm
		vector< cv::Point3f > markers(4);
		markers[0].x = -407.80142211914062;
		markers[0].y = -119.23642730712891;
		markers[0].z = -322.19158935546875;
		markers[1].x = -412.94619750976562;
		markers[1].y = 114.73293304443359;
		markers[1].z = 266.53179931640625;
		markers[2].x = 220.64505004882812;
		markers[2].y = 104.16347503662109;
		markers[2].z = 276.37896728515625;
		markers[3].x = 226.10403442382812;
		markers[3].y = -127.99021148681641;
		markers[3].z = -312.4443359375;
		for (int i = 0; i < 4; i++) {
			markers[i] = markers[i] * 0.001f;
		}

		// local coordinates of pattern
		plane_s = markers[3] - markers[0];
		plane_t = markers[1] - markers[0];

		len_s = (float)cv::norm(plane_s);
		len_t = (float)cv::norm(plane_t);

		plane_o = markers[0];

		plane_s = plane_s / len_s;
		plane_t = plane_t / len_t;

		plane_n = plane_s.cross(plane_t);
		plane_n = plane_n / (float)cv::norm(plane_n);

		plane_t = plane_n.cross(plane_s);
		plane_t = plane_t / (float)cv::norm(plane_t);

		for (int i = 0; i < markers.size(); i++) {
			markers[i] = markers[i] - plane_n * 0.008f; // convert to meter
		}

		pattern2d.SetChessboard2D(cv::Size(7, 6), 0.09f); // unit : meter
		pattern3d.SetChessboard3D(cv::Size(7, 6), markers);
	}

	//=====================
	// Calibration
	//=====================
	Moca::CameraParameter cam_rgb, cam_ir;
	Moca::Calibrator_Camera cali_rgb, cali_ir;
	Moca::Calibrator_Extr cali_rgb_vicon;
	Moca::Calibrator_Extr cali_ir_vicon;
	Moca::Calibratior_Stereo cali_stereo;

	bool isCalibrated = false;

	// image corners
	vector< cv::Point2f > corners_ir, corners_rgb;

	// 
	cam_ir.LoadCali((kinect_name + "_ir_cali.txt").c_str());
	cam_rgb.LoadCali((kinect_name + "_rgb_cali.txt").c_str());
	//	isCalibrated = true;
	vector<cv::Point3f> points_rgb;
	vector<cv::Point3f> points_vicon;

	//=====================
	// Loop
	//=====================
	while (1) {
		// update vrpn
		vrpn_pattern.Loop();
		vrpn_kinect.Loop();

		cv::Mat avg_mat_kinect = mat_kinect;
		cv::Mat avg_mat_pattern = mat_pattern;

		// update kinect
		kinect.Loop();

		if (isCalibrated) {
			{
				// IR
				cv::Mat undistortedImage;
				//cv::undistort(kinect.img_ir, undistortedImage, cali.intr.cameraMatrix, cali.intr.distCoef);
				undistortedImage = kinect.img_ir;

				Moca::CameraIntrinsic intr = cam_ir.intr;
				//intr.distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

				vector<cv::Point2f> proj;
				Moca::ProjectPoints(pattern3d.corners, proj, intr, cam_ir.extr * mat_pattern);
				for (int i = 0; i < proj.size(); i++) {
					cv::Point pt(proj[i].x, proj[i].y);
					cv::Scalar color(0, 0, 255);
					cv::circle(undistortedImage, pt, 3, color, 1, CV_AA, 0);
				}
				cv::imshow("ir", undistortedImage / 2000.0f);
			}
			{
				// RGB
				cv::Mat undistortedImage;
				//cv::undistort(kinect.img_ir, undistortedImage, cali.intr.cameraMatrix, cali.intr.distCoef);
				undistortedImage = kinect.img_rgb;

				Moca::CameraIntrinsic intr = cam_rgb.intr;
				//intr.distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

				vector<cv::Point2f> proj;
				Moca::ProjectPoints(pattern3d.corners, proj, intr, cam_rgb.extr * mat_pattern);
				for (int i = 0; i < proj.size(); i++) {
					cv::Point pt(proj[i].x, proj[i].y);
					cv::Scalar color(0, 0, 255);
					cv::circle(undistortedImage, pt, 3, color, 1, CV_AA, 0);
				}
				cv::imshow("rgb", undistortedImage);
			}
		}
		else {
			//cv::imshow("rgb", kinect.img_rgb);
			cv::imshow("ir", kinect.img_ir / 2000.0f);
		}

		int key = cv::waitKey(1);


		// capture new frame
		if (key == '1') {
			bool success_rgb = Moca::CaptureRGBFrame(kinect.img_rgb, pattern2d.boardSize, 5, corners_rgb);

			cout << "capture rgb :" << endl;
			if (success_rgb) {
				Moca::DisplayCorners(kinect.img_rgb, pattern2d.boardSize, corners_rgb, "corner_rgb");
				cali_rgb.CaptureFrame(kinect.img_rgb.size(), corners_rgb);
				cali_rgb_vicon.CaptureFrame(avg_mat_pattern);
			}

			//cout << "capture stereo :" << endl;
			//if (success_rgb && success_ir) {
			//	cali_stereo.CaptureFrame(kinect.img_rgb.size(), kinect.img_ir.size(), corners_rgb, corners_ir);
			//}
		}

		if (key == '2') {
			bool success_ir = Moca::CaptureIRFrame(kinect.img_ir, pattern2d.boardSize, 4, corners_ir);

			cout << "capture ir :" << endl;
			if (success_ir) {
				cv::Mat img_ir = kinect.img_ir / 2000.0f;
				Moca::DisplayCorners(img_ir, pattern2d.boardSize, corners_ir, "corner_ir");
				cali_ir.CaptureFrame(kinect.img_ir.size(), corners_ir);
				cali_ir_vicon.CaptureFrame(avg_mat_pattern);
			}
		}

		// calibrate kinect intr
		if (key == '4') {
			if (cali_rgb.imgPoints.size() > 0 && cali_ir.imgPoints.size() > 0) {
				// intr : rgb
				cali_rgb.Calibrate(cam_rgb, pattern2d);

				// intr : ir
				cali_ir.Calibrate(cam_ir, pattern2d);
			}
		}

		// calibrate kinect extr in vicon space
		if (key == '3') {
			if (cali_rgb.imgPoints.size() > 0 && cali_ir.imgPoints.size() > 0) {
				// extr : vicon->rgb
				cali_rgb_vicon.Calibrate(cam_rgb, pattern3d, cali_rgb.imgPoints);

				// extr : vicon->ir
				cali_ir_vicon.Calibrate(cam_ir, pattern3d, cali_ir.imgPoints);
				// extr : rgb->ir
				//cali_stereo.Calibrate(cam_rgb, cam_ir, pattern2d);
			}
		}

		if (key == '0') {
			cam_rgb.SaveCali((vrpn_kinect.m_id + "_rgb_cali.txt").c_str());

			cam_ir.SaveCali((vrpn_kinect.m_id + "_ir_cali.txt").c_str());

			isCalibrated = true;
		}

		if (key == '6') {
			points_rgb.clear();
			points_vicon.clear();
		}

#if 0
		if (key == '8') {
			printf("====================pair\n");
			Moca::PairPointsRigidRegistration(points_vicon, points_rgb, cam_rgb.extr);
		}

		if (key == '7') {
			cv::Mat depth(kinect.img_depth.rows, kinect.img_depth.cols, CV_32FC1);
			Moca::UndistortImage<float>(kinect.img_depth, depth, cam_ir.intr, 0);
			cv::Mat rgb(kinect.img_rgb.rows, kinect.img_rgb.cols, CV_8UC3);
			Moca::UndistortImage<cv::Vec3b>(kinect.img_rgb, rgb, cam_rgb.intr, cv::Vec3b(0, 0, 0));


			vector< cv::Point2f > corners;
			Moca::FindChessboardCornersFromImage(rgb, pattern2d.boardSize, corners);

			cv::Mat depth_rgb;
			SplatDepthMap(depth, cam_ir, rgb, cam_rgb, depth_rgb);


			cv::imshow("ok", depth / 1000.0f);
			//cv::imshow("okok", depth_rgb);

			vector<cv::Point3f> points(corners.size());
			vector<cv::Point3f> points2(corners.size());
			for (int i = 0; i < corners.size(); i++) {
				points[i] = cam_rgb.intr.BackProjectPoints(corners[i], depth_rgb.at<float>(corners[i].y, corners[i].x));
				points2[i] = mat_pattern * pattern3d.corners[i];
				printf("%f %f : %f\n", corners[i].x, corners[i].y, depth_rgb.at<float>(corners[i].y, corners[i].x));
			}

			points_rgb.reserve(points_rgb.size() + corners.size());
			points_vicon.reserve(points_rgb.size() + corners.size());
			for (int i = 0; i < corners.size(); i++) {
				points_rgb.push_back(points[i]);
				points_vicon.push_back(points2[i]);
			}
			printf("RME : %d %d %f\n", corners.size(), points_vicon.size(), Moca::RME(pattern3d.corners, points, cam_rgb.extr * mat_pattern));

			for (int i = 0; i < 5; i++) {
				for (int j = 0; j < 6; j++) {
					//	printf( "%d %d : %f\n", i, j, Length( points[(i +1)* 7 + j] - points[i * 7 + j] ) );
					//	printf("%d %d : %f\n", i, j, Length(pattern3d.corners[(i + 1) * 7 + j] - pattern3d.corners[i * 7 + j]));
				}
			}
		}
#endif

	}
}

//=====================
// Nikon Calibration
//=====================
void sample_main_calibrate_vicon_nikon(void) {
	//=====================
	// Vicon
	//=====================
	const char *ip = "192.168.10.1";
	VRPN_Tracker vrpn_pattern, vrpn_nikon;
	cv::Mat mat_pattern = cv::Mat_<double>(4, 4);
	cv::Mat mat_nikon = cv::Mat_<double>(4, 4);
	vrpn_pattern.Create("PATTERN", ip, &mat_pattern, sample_cali_callback);
	vrpn_nikon.Create("NIKON", ip, &mat_nikon, sample_cali_callback);

	//=====================
	// Pattern
	//=====================
	Moca::Pattern pattern2d, pattern3d;
	{
		// 1 --- 2 \
						// |     |  4
// 0 --- 3 /
// vsk unit : mm
		vector< cv::Point3f > markers(4);
		markers[0].x = -407.80142211914062;
		markers[0].y = -119.23642730712891;
		markers[0].z = -322.19158935546875;
		markers[1].x = -412.94619750976562;
		markers[1].y = 114.73293304443359;
		markers[1].z = 266.53179931640625;
		markers[2].x = 220.64505004882812;
		markers[2].y = 104.16347503662109;
		markers[2].z = 276.37896728515625;
		markers[3].x = 226.10403442382812;
		markers[3].y = -127.99021148681641;
		markers[3].z = -312.4443359375;

		//subtract the height of the marker
		cv::Point3f vec_1 = markers[0] - markers[1];
		cv::Point3f vec_2 = markers[2] - markers[1];
		cv::Point3f norm_pattern = vec_1.cross(vec_2) / (float)(cv::norm(vec_1.cross(vec_2))) * 8.0f;

		for (int i = 0; i < markers.size(); i++) {
			markers[i] = (markers[i] - norm_pattern) * 0.001f; // convert to meter
		}

		pattern2d.SetChessboard2D(cv::Size(7, 6), 0.09); // unit : meter
		pattern3d.SetChessboard3D(cv::Size(7, 6), markers);
	}

	//=====================
	// Calibration
	//=====================
	Moca::CameraParameter cam_rgb;
	Moca::Calibrator_Camera cali_rgb;
	Moca::Calibrator_Extr cali_rgb_vicon;

	bool testCalibrate = false;

	// image corners
	vector< cv::Point2f > corners_rgb;
	int img_width = 0, img_height = 0;
	//=====================
	// Loop
	//=====================
	std::vector<int> failList;
	const int downScale = 2;
	while (1) {
		// update vrpn
		vrpn_pattern.Loop();
		vrpn_nikon.Loop();

		cv::Mat avg_mat_nikon = mat_nikon;
		cv::Mat avg_mat_pattern = mat_pattern;

		cv::imshow("aaa", cv::Mat::zeros(cv::Size(512, 424), CV_16UC1));
		int key = cv::waitKey(1);


		if (testCalibrate) {
			cout << "Please input the test image id: " << std::endl;
			int testID = 0;
			cin >> testID;
			char filename[64];
			sprintf(filename, "../NiconCali/DSC_0%d.JPG", testID);
			cv::Mat testImage = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
			cv::Mat down;
			down = testImage.clone();
			cam_rgb.intr.InitUndistortMap(cv::Size(down.cols, down.rows));

			Moca::CameraIntrinsic intr;
			intr = cam_rgb.intr;

			vector<cv::Point2f> proj;
			Moca::ProjectPoints(pattern3d.corners, proj, cam_rgb.intr, cam_rgb.extr * mat_pattern);
			for (int i = 0; i < proj.size(); i++) {
				cv::Point pt(proj[i].x, proj[i].y);
				cv::Scalar color(0, 0, 255);
				cv::circle(down, pt, 3, color, 1, CV_AA, 0);
			}
			cv::imshow("rgb", down);
			cv::waitKey(1);
			testCalibrate = !testCalibrate;
		}

		// record current vicon frame
		if (key == '1') {
			cout << "capture rgb correspondent vicon:" << endl;
			cali_rgb_vicon.CaptureFrame(avg_mat_pattern);
			std::cout << avg_mat_pattern << std::endl;
		}

		// load images
		if (key == '2') {
			cout << "Please input the origin id: " << std::endl;
			int origin = 0;
			cin >> origin;
			cout << "Origin is : " << origin << "\n";
			cout << "Please input num of capture images " << std::endl;
			int numImg = 0;
			cin >> numImg;
			cout << "numImg is : " << numImg << "\n";

			cout << "load rgb images" << endl;
			char filename[64];
			for (int i = 0; i < numImg; i++) {
				sprintf(filename, "../NiconCali/DSC_0%d.JPG", i + origin);
				cv::Mat temp = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
				img_width = temp.cols;
				img_height = temp.rows;

				// downsample
				int hf_width = img_width / 2;
				int hf_height = img_height / 2;
				cv::Mat down = cv::Mat(hf_height, hf_width, CV_8UC3);
				for (int y = 0; y < hf_height; y++) {
					for (int x = 0; x < hf_width; x++) {
						down.at<cv::Vec3b>(y, x) = 0.25f * temp.at<cv::Vec3b>(2 * y, 2 * x) +
							0.25f * temp.at<cv::Vec3b>(2 * y, 2 * x + 1) +
							0.25f * temp.at<cv::Vec3b>(2 * y + 1, 2 * x) +
							0.25f * temp.at<cv::Vec3b>(2 * y + 1, 2 * x + 1);
					}
				}
				cam_rgb.intr.InitUndistortMap(cv::Size(down.cols, down.rows));

				bool success = Moca::CaptureRGBFrame(down, pattern2d.boardSize, 4, corners_rgb);
				if (success) {
					Moca::DisplayCorners(down, pattern2d.boardSize, corners_rgb, "Corners");
					cv::waitKey(1);
					cali_rgb.CaptureFrame(down.size(), corners_rgb);
				}
				else {
					failList.push_back(i);
					printf("%dth image : unable to find corner points... \n", i);
				}
			}
			printf("Process images corner done \n");
		}

		// calibrate kinect and save config
		if (key == '3') {
			if (cali_rgb.imgPoints.size() > 0) {
				printf("[ EXTR ] Start calibrate camera with corners\n");
				// extr : vicon->rgb
				cali_rgb_vicon.CalibrateNikon(cam_rgb, pattern3d, cali_rgb.imgPoints, failList);

				cam_rgb.SaveCali((vrpn_nikon.m_id + "_nikon_cali.txt").c_str());
			}
		}

		// load calibrate result
		if (key == '4') {
			cam_rgb.LoadCali("NIKON_nikon_cali.txt");
			testCalibrate = false;
			printf("Load calibration param done. \n");
		}
		// do intrinsic calibration
		if (key == '5') {
			if (cali_rgb.imgPoints.size() > 0) {
				printf("[ INTR ] Start calibrate camera\n");
				// intr : rgb
				cali_rgb.Calibrate(cam_rgb, pattern2d);
				cam_rgb.intr.cameraMatrix.at<double>(0, 0) *= downScale;
				cam_rgb.intr.cameraMatrix.at<double>(1, 1) *= downScale;
				cam_rgb.intr.cameraMatrix.at<double>(0, 2) *= downScale;
				cam_rgb.intr.cameraMatrix.at<double>(1, 2) *= downScale;
				cam_rgb.SaveCali((vrpn_nikon.m_id + "_nikon_cali.txt").c_str());
			}
		}
		// terminate testing mode and switch back to calibration mode
		if (key == '8') {
			testCalibrate = !testCalibrate;
			if (testCalibrate == true) {
				cam_rgb.LoadCali("NIKON_nikon_cali.txt");
				cam_rgb.intr.cameraMatrix.at<double>(0, 0) *= downScale;
				cam_rgb.intr.cameraMatrix.at<double>(1, 1) *= downScale;
				cam_rgb.intr.cameraMatrix.at<double>(0, 2) *= downScale;
				cam_rgb.intr.cameraMatrix.at<double>(1, 2) *= downScale;
			}
			printf("Switch to calibration mode\n");
		}
	}
}

