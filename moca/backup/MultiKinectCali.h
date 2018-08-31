#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <cv_op.h>

#include <vector>
#include <stdio.h>
using namespace std;

namespace Calib{
	//================
	// CameraIntrinsic
	//================
	class CameraIntrinsic {
	public:
		cv::Mat		cameraMatrix;
		cv::Mat		distCoef;
		cv::Mat		undistMap[2];

	public:
		CameraIntrinsic() {
			cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
			distCoef = cv::Mat::zeros(8, 1, CV_64F);
		}

		cv::Mat Intr(void) const {
			cv::Mat K = cv::Mat(4, 1, CV_64F);

			K.at<double>(0, 0) = cameraMatrix.at<double>(0, 2);
			K.at<double>(1, 0) = cameraMatrix.at<double>(1, 2);
			K.at<double>(2, 0) = cameraMatrix.at<double>(0, 0);
			K.at<double>(3, 0) = cameraMatrix.at<double>(1, 1);

			return K;
		}
	};

	//================
	// RigidTransform
	//================
	class RigidTransform {
	public:
		cv::Mat		rvec;
		cv::Mat		tvec;

	public:
		RigidTransform() {
			rvec = cv::Mat::zeros(3, 1, CV_64F);
			tvec = cv::Mat::zeros(3, 1, CV_64F);
		}

		cv::Mat Mat(void) const {
			cv::Mat m = cv::Mat::eye(4, 4, CV_64F);

			m.at<double>(0, 3) = tvec.at<double>(0, 0);
			m.at<double>(1, 3) = tvec.at<double>(1, 0);
			m.at<double>(2, 3) = tvec.at<double>(2, 0);

			cv::Mat rmat;
			cv::Rodrigues(rvec, rmat);
			for (int j = 0; j < 3; j++) {
				for (int i = 0; i < 3; i++) {
					m.at<double>(i, j) = rmat.at<double>(i, j);
				}
			}


			//	camera position test
			cv::Mat R_t = rmat.t();
			cv::Mat R_inv = rmat.inv();
			cout << R_t << endl;
			cout << R_inv << endl;

			cv::Mat P = -R_inv*tvec;
			double* p = (double *)P.data;
			printf("x=%lf, y=%lf, z=%lf", p[0], p[1], p[2]);

			return m;
		}
	};

	double ReprojectionError(
		const vector< cv::Point3f > &objectPoints,
		const vector< cv::Point2f > &imagePoints,
		const CameraIntrinsic &intr, const RigidTransform &extr
		) {
		// project
		vector< cv::Point2f > proj;
		cv::projectPoints(cv::Mat(objectPoints), extr.rvec, extr.tvec, intr.cameraMatrix, intr.distCoef, proj);

		// difference
		double err = cv::norm(cv::Mat(imagePoints), cv::Mat(proj), CV_L2);

		// rms
		return std::sqrt(err * err / (double)objectPoints.size());
	}

	void CalcChessboardCornersFromSquare(cv::Size boardSize, float squareSize, vector< cv::Point3f > &corners) {
		corners.clear();
		corners.reserve(boardSize.area());
		for (int y = 0; y < boardSize.height; y++) {
			for (int x = 0; x < boardSize.width; x++) {
				corners.push_back(cv::Point3f(x * squareSize, y * squareSize, 0));
			}
		}
	}

	// corners
	// 1 --- 2
	// |     |
	// 0 --- 3
	bool FindChessboardCornersFromImage(const cv::Mat &image, cv::Size boardSize, vector< cv::Point2f > &corners) {
		if (!cv::findChessboardCorners(image, boardSize, corners)) {
			return false;
		}
		cv::Mat gray;
		cv::cvtColor(image, gray, CV_BGR2GRAY);
		cv::cornerSubPix(gray, corners, cv::Size(2, 2), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		return true;
	}


	void CalibrateCameraWithChessboard(
		const vector< vector< cv::Point2f > > &imgPoints, cv::Size imageSize, cv::Size boardSize, float squareSize,
		CameraIntrinsic &intr, vector< RigidTransform > &extr, int flag = CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5
		) {
		if (imgPoints.empty()){
			return;
		}

		// object points from squares
		vector< vector< cv::Point3f > > objPoints;
		objPoints.resize(imgPoints.size());

		CalcChessboardCornersFromSquare(boardSize, squareSize, objPoints[0]);
		for (int i = 1; i < imgPoints.size(); i++) {
			objPoints[i] = objPoints[0];
		}

		if (flag & CV_CALIB_FIX_ASPECT_RATIO) {
			intr.cameraMatrix.at<double>(0, 0) = 0.0; // FIXME
		}

		vector< cv::Mat > rvecs, tvecs;
		cv::calibrateCamera(
			objPoints, imgPoints, imageSize,
			intr.cameraMatrix, intr.distCoef, rvecs, tvecs, flag
			);

		extr.resize(rvecs.size());
		for (int i = 0; i < extr.size(); i++) {
			extr[i].rvec = rvecs[i];
			extr[i].tvec = tvecs[i];
		}

		// undistort
		cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(intr.cameraMatrix, intr.distCoef, imageSize, 1, imageSize, 0);
		cv::initUndistortRectifyMap(
			intr.cameraMatrix, intr.distCoef, cv::Mat(), newCameraMatrix,
			imageSize, CV_16SC2, intr.undistMap[0], intr.undistMap[1]
			);

		cout << "camera intr :" << endl;
		cout << intr.cameraMatrix << endl;
		cout << intr.distCoef << endl;
	}

	//================
	// Calibrator for Kinect to Kinect
	//================
	class Calibrator_Kinect {
	public:
		// board
		cv::Size							board_size;
		float								board_square_size;
		cv::Size							img_size;

		// frames for calibration
		vector< vector< cv::Point2f > >		img_points;

		// camera
		Calib::CameraIntrinsic				cam_intr;
		vector<RigidTransform>				cam_extr;
		bool								isCalibrated;

	public:
		void SetChessboard(cv::Size size, float square_size) {
			board_size = size;
			board_square_size = square_size;
		}

		void ResetFrames(void) {
			img_points.clear();
			cam_extr.clear();
		}

		void CaptureFrame(const cv::Mat &img_ir, int idx) {
			cv::Mat cvImg(img_ir.size(), CV_8UC3);
			int num = img_ir.size().area();
			for (int i = 0; i < num; i++) {
				unsigned char c = Clamp(((float*)img_ir.ptr())[i] / 20.0f, 0.0f, 255.0f);
				cvImg.ptr()[i * 3 + 0] = c;
				cvImg.ptr()[i * 3 + 1] = c;
				cvImg.ptr()[i * 3 + 2] = c;
				//cout << (float*)cvImg.ptr()[i * 3 + 0] << endl;				cout << (float*)cvImg.ptr()[i * 3 + 1] << endl; cout << (float*)cvImg.ptr()[i * 3 + 2] << endl;
			}

			vector< cv::Point2f > corners;
			if (Calib::FindChessboardCornersFromImage(cvImg, board_size, corners)) {
				cv::drawChessboardCorners(cvImg, board_size, cv::Mat(corners), true);
				img_points.push_back(corners);
				img_size = cvImg.size();
				cout << "capture : " << img_points.size() << endl;
			}
			// FIX LOGIC HERE
			char Wndname[512];
			sprintf(Wndname, "corner%d", idx);
			cv::imshow(Wndname, cvImg);
		}

		void Calibrate(void) {
			// intrisic
			Calib::CalibrateCameraWithChessboard(img_points, img_size, board_size, board_square_size, cam_intr, cam_extr);

			// extrinsic

		}

		void SaveCali(const char *path) {
			FILE *fp = fopen(path, "w");
			if (!fp) {
				return;
			}

			// camera matrix
			for (int i = 0; i < 9; i++) {
				fprintf(fp, "%f ", ((double*)cam_intr.cameraMatrix.ptr())[i]);
			}
			fprintf(fp, "\n");

			// distortion
			for (int i = 0; i < 5; i++) {
				fprintf(fp, "%f ", ((double*)cam_intr.distCoef.ptr())[i]);
			}
			fprintf(fp, "\n");

			// kinect to camera
			int ex_size = cam_extr.size();
			cv::Mat tmp = cam_extr[ex_size - 1].Mat();
			//cv::Mat tmp = cam_extr[0].Mat();
			for (int i = 0; i < 16; i++) {
				fprintf(fp, "%f ", ((double*)tmp.ptr())[i]);
			}
			fprintf(fp, "\n");
			fclose(fp);
			fp = NULL;
		}
		
	};
}

