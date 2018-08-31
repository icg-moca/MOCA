//
//  Moca_cameraCalibration.h
//  Moc
//
//  Created by Wei Li on 4/24/15.
//

#ifndef Moca_cameraCalibration_h
#define Moca_cameraCalibration_h

#include <stdio.h>
#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_op.h>

namespace Moca {

//================
// CameraIntrinsic
//================
class CameraIntrinsic {
public :
	cv::Mat		cameraMatrix; // 3x3 64F matrix
	cv::Mat		distCoeffs; // 8x1 64F vector
	cv::Mat		undistMap[2];

public :
	CameraIntrinsic() {
		cameraMatrix = cv::Mat::eye  ( 3, 3, CV_64F );
		distCoeffs   = cv::Mat::zeros( 5, 1, CV_64F );
	}

	// 4x1 64F vector : focal length, optical center
	cv::Mat IntrVec( void ) const {
		cv::Mat K = cv::Mat( 4, 1, CV_64F );

		K.at<double>( 0, 0 ) = cameraMatrix.at<double>( 0, 0 );
		K.at<double>( 1, 0 ) = cameraMatrix.at<double>( 1, 1 );
		K.at<double>( 2, 0 ) = cameraMatrix.at<double>( 0, 2 );
		K.at<double>( 3, 0 ) = cameraMatrix.at<double>( 1, 2 );

		return K;
	}

	cv::Point2f ProjectPoints( cv::Point3f v ) {
		cv::Point2f u;
		u.x = v.x / v.z * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2);
		u.y = v.y / v.z * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2);

		return u;
	}

	cv::Point3f BackProjectPoints(cv::Point2f v, float d) {
		cv::Point3f u;
		u.x = (v.x - cameraMatrix.at<double>(0, 2)) / cameraMatrix.at<double>(0, 0) * d;
		u.y = (v.y - cameraMatrix.at<double>(1, 2)) / cameraMatrix.at<double>(1, 1) * d;
		u.z = d;
		return u;
	}

	void InitUndistortMap( cv::Size imageSize ) {
		cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix( cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0 );
		//cv::Mat newCameraMatrix = cameraMatrix;
		cv::initUndistortRectifyMap(
			cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix,
			imageSize, CV_16SC2, undistMap[0], undistMap[1]
		);
	}
};

//================
// RigidTransform
//================
class RigidTransform {
public :
	// 3x1 64F vector
	cv::Mat		rvec;
	cv::Mat		tvec;
	cv::Mat		evec;
	cv::Mat		fvec;

public :
	RigidTransform() {
		rvec = cv::Mat::zeros( 3, 1, CV_64F );
		tvec = cv::Mat::zeros( 3, 1, CV_64F );
	}

	// 4x4 64F matrix
	cv::Mat Mat( void ) const {
		cv::Mat m = cv::Mat::eye( 4, 4, CV_64F );

		// T
		m.at<double>( 0, 3 ) = tvec.at<double>( 0, 0 );
		m.at<double>( 1, 3 ) = tvec.at<double>( 1, 0 );
		m.at<double>( 2, 3 ) = tvec.at<double>( 2, 0 );

		// R
		cv::Mat rmat;
		cv::Rodrigues( rvec, rmat );
		for ( int j = 0; j < 3; j++ ) {
			for ( int i = 0; i < 3; i++ ) {
				m.at<double>( i, j ) = rmat.at<double>( i, j );
			}
		}

		return m;
	}
};

//================
// Pattern
//================
class Pattern {
public:
	cv::Size				boardSize; // inner corner size

	vector< cv::Point3f >	corners; // boardSize.width * boardSize.height

public:
	// Chessboard
	void SetChessboard2D(const cv::Size &boardSize, const float gridSize) {
		this->boardSize = boardSize;

		corners.clear();
		corners.reserve(boardSize.area());
		for (int y = 0; y < boardSize.height; y++) {
			for (int x = 0; x < boardSize.width; x++) {
				corners.push_back(cv::Point3f(x * gridSize, y * gridSize, 0));
			}
		}
	}

	// 1 --- 2
	// |     |
	// 0 --- 3
	void SetChessboard3D(const cv::Size &boardSize, const vector< cv::Point3f > &markers) {
		this->boardSize = boardSize;

		corners.clear();
		corners.reserve(boardSize.area());
		for (int y = boardSize.height; y >= 1; y--) {
			float fy = (float)y / (float)(boardSize.height + 1);
			for (int x = 1; x <= boardSize.width; x++) {
				//float fx = ( float )x / ( float )( boardSize.width + 1 );
				float fx = (float)x / (float)(boardSize.width);
				cv::Point3f e0 = (markers[3] - markers[0]) * (double)fx + markers[0];
				cv::Point3f e1 = (markers[2] - markers[1]) * (double)fx + markers[1];
				cv::Point3f e = (e1 - e0) * (double)fy + e0;
				corners.push_back(e);
			}
		}
	}
};

bool FindChessboardCornersFromImage( const cv::Mat &image, cv::Size boardSize, vector< cv::Point2f > &corners, int winSize = 2 ) {
	if ( !cv::findChessboardCorners( image, boardSize, corners ) ) {
		return false;
	}
	cv::Mat gray;
	cv::cvtColor( image, gray, CV_BGR2GRAY );
	cv::cornerSubPix( gray, corners, cv::Size( winSize, winSize ), cv::Size( -1, -1 ), cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ) );
	return true;
}

void CalibrateCameraWithChessboard(
	const vector< vector< cv::Point2f > > &imgPoints, cv::Size imageSize, const Pattern &pattern,
	CameraIntrinsic &intr, vector< RigidTransform > &extr, int flag = CV_CALIB_FIX_K4|CV_CALIB_FIX_K5
) {
	if ( imgPoints.empty() ){
		return;
	}

	// object points from squares
	vector< vector< cv::Point3f > > objPoints( imgPoints.size() );
	for ( int i = 0; i < imgPoints.size(); i++ ) {
		objPoints[ i ] = pattern.corners;
	}

	if ( flag & CV_CALIB_FIX_ASPECT_RATIO ) {
		intr.cameraMatrix.at<double>( 0, 0 ) = 0.0; // FIXME
	}

	vector< cv::Mat > rvecs, tvecs;
	cv::calibrateCamera(
		objPoints, imgPoints, imageSize,
		intr.cameraMatrix, intr.distCoeffs, rvecs, tvecs, flag
	);

	extr.resize( rvecs.size() );
	for ( int i = 0; i < extr.size(); i++ ) {
		extr[ i ].rvec = rvecs[ i ];
		extr[ i ].tvec = tvecs[ i ];
	}

	intr.InitUndistortMap(imageSize);

	cout << "camera intr :" << endl;
	cout << intr.cameraMatrix << endl;
	cout << intr.distCoeffs << endl;
}

// overflow
void PairPointsRigidRegistration( const vector< cv::Point3f > &src, const vector< cv::Point3f > &dst, cv::Mat &M ) {
	if (src.size() <= 0) {
		printf( "Empty!!!!!\n");
		return;
	}
	
	cv::Point3f src_o = Sum(src) / (float)src.size();
	cv::Point3f dst_o = Sum(dst) / (float)dst.size();

	// maximum distance
	float s = Length(src[0] - src_o);
	for ( int i = 1; i < src.size(); i++ ) {
		double d = Length( src[ i ] - src_o );
		if ( s < d ) {
			s = d;
		}
	}

	cv::Mat H = cv::Mat::zeros( 3, 3, CV_64F );
	for ( int i = 0; i < src.size(); i++ ) {
		H += Outer( ( src[ i ] - src_o ) / s, ( dst[ i ] - dst_o ) / s );
	}

	cv::Mat W, U, V;
	cv::SVDecomp( H, W, U, V, cv::SVD::FULL_UV );

	cv::Mat R = ( U * V ).t();
	cv::Point3f T = dst_o - R * src_o;

	M = TranslateRotate( T, R );

	cout << "==== Rigid Registration ====" << endl;
	cout << "R " << R << endl;
	cout << "T " << T << endl;
}

void ProjectPoints(const vector<cv::Point3f> &objPoints, vector<cv::Point2f> &imgPoints, const CameraIntrinsic &intr, const cv::Mat &extr) {
	vector< cv::Point3f > points( objPoints.size() );
	for ( int i = 0; i < objPoints.size(); i++ ) {
		points[i] = extr * objPoints[i];
	}
	cv::projectPoints( points, cv::Mat::zeros( 3, 1, CV_64F ), cv::Mat::zeros( 3, 1, CV_64F ), intr.cameraMatrix, intr.distCoeffs, imgPoints );
}

void ProjectPoints(const vector< cv::Point3f > &objPoints, vector< cv::Point2f > &imgPoints, const CameraIntrinsic &intr, const RigidTransform &extr) {
	cv::projectPoints(cv::Mat(objPoints), extr.rvec, extr.tvec, intr.cameraMatrix, intr.distCoeffs, imgPoints);
}

double ProjectionError(const vector< cv::Point3f > &objPoints, const vector< cv::Point2f > &imgPoints, const CameraIntrinsic &intr, const RigidTransform &extr) {
	// project
	vector< cv::Point2f > proj;
	ProjectPoints(objPoints, proj, intr, extr);

	// difference
	double err = cv::norm(cv::Mat(imgPoints), cv::Mat(proj), CV_L2);

	// rms
	return std::sqrt(err * err / (double)objPoints.size());
}

double RME(const vector< cv::Point3f > &src, const vector< cv::Point3f > &dst, const cv::Mat &M) {
	float rme = 0.0f;
	for (int i = 0; i < src.size(); i++) {
		cv::Point3f r = M * src[i] - dst[i];
		rme += r.dot(r);
	}

	// rms
	return std::sqrt(rme / (double)src.size());
}

double EstimateCameraPose(const vector< cv::Point3f > &objPoints, const vector< cv::Point2f > imgPoints, const CameraIntrinsic &intr, cv::Mat &extr) {
	RigidTransform m;
	cv::solvePnP(cv::Mat(objPoints), cv::Mat(imgPoints), intr.cameraMatrix, intr.distCoeffs, m.rvec, m.tvec);
	extr = m.Mat();

	double err = ProjectionError(objPoints, imgPoints, intr, m);
	cout << "error : " << err << endl;
	return err;
}

void Distort(float &px, float &py, const double k[]) {
	float k1 = k[0];
	float k2 = k[1];
	float p1 = k[2];
	float p2 = k[3];
	float k3 = k[4];

	float r2 = px * px + py * py;
	float r4 = r2 * r2;
	float r6 = r4 * r2;

	float s = (1 + k1 * r2 + k2 * r4 + k3 * r6);

	float x = px * s + 2 * p1 * px * py + p2 * (r2 + 2 * px * px);
	float y = py * s + 2 * p2 * px * py + p1 * (r2 + 2 * py * py);

	px = x;
	py = y;
}

template <class T>
void UndistortImage(const cv::Mat &src, cv::Mat &dst, const CameraIntrinsic &intr, T zero, int interpolation = cv::INTER_NEAREST) {
#if 1
	printf( "disort\n" );

	for (int y = 0; y < dst.rows; y++) {
		for (int x = 0; x < dst.cols; x++) { 
			float px = (x - intr.IntrVec().at<double>(2)) / intr.IntrVec().at<double>(0);
			float py = (y - intr.IntrVec().at<double>(3)) / intr.IntrVec().at<double>(1);

			Distort(px, py, (double*)(intr.distCoeffs.data));

			
			float qx = px * intr.IntrVec().at<double>(0) + intr.IntrVec().at<double>(2);
			float qy = py * intr.IntrVec().at<double>(1) + intr.IntrVec().at<double>(3);

	//		printf("%d %d, %f, %f\n", x, y, qx, qy);

			if (qx >= 0 && qx < src.cols && qy >= 0 && qy < src.rows ) {
				((T *)dst.ptr())[(int)x + (int)y * dst.cols] = ((T *)src.ptr())[(int)qx + (int)qy * src.cols];
			}
			else {
				((T *)dst.ptr())[(int)x + (int)y * dst.cols] = zero;
			}
		}
	}
#else
	cv::remap( src, dst, intr.undistMap[0], intr.undistMap[1], interpolation );
#endif
}

bool CaptureIRFrame(const cv::Mat &img_ir, const cv::Size &boardSize, int winSize, vector< cv::Point2f > &corners, float scale = 20.0f) {
	cv::Mat cvImg(img_ir.size(), CV_8UC3);
	int num = img_ir.size().area();
	for (int i = 0; i < num; i++) {
		unsigned char c = Clamp(((float*)img_ir.ptr())[i] / scale, 0.0f, 255.0f);
		cvImg.ptr()[i * 3 + 0] = c;
		cvImg.ptr()[i * 3 + 1] = c;
		cvImg.ptr()[i * 3 + 2] = c;
	}

	return FindChessboardCornersFromImage(cvImg, boardSize, corners, winSize);
}

bool CaptureRGBFrame(const cv::Mat &img_rgb, const cv::Size &boardSize, int winSize, vector< cv::Point2f > &corners) {
	return FindChessboardCornersFromImage(img_rgb, boardSize, corners, winSize);
}

void DisplayCorners(cv::Mat &img, const cv::Size &boardSize, const vector< cv::Point2f > &corners, const char *name) {
	cv::drawChessboardCorners(img, boardSize, cv::Mat(corners), true);
	cv::imshow(name, img);
}

void DrawCircles( cv::Mat &image, vector<cv::Point2f> &points ) {
	for (int i = 0; i < points.size(); i++) {
		cv::Scalar color(0, 0, 255);
		cv::circle(image, cv::Point(points[i].x, points[i].y), 3, color, 1, CV_AA, 0);
	}
}

void SaveMat(const char *path, const cv::Mat &mat) {
	FILE *fp = fopen(path, "w");
	if (!fp) {
		return;
	}

	// kinect to camera
	for (int i = 0; i < 16; i++) {
		fprintf(fp, "%f ", ((double*)mat.ptr())[i]);
	}
	fprintf(fp, "\n");

	fclose(fp);
	fp = NULL;
}

void LoadMat(const char *path, cv::Mat &mat) {
	FILE *fp = fopen(path, "r");
	if (!fp) {
		return;
	}

	mat = cv::Mat(4, 4, CV_64F);
	// kinect to camera
	for (int i = 0; i < 16; i++) {
		float v;
		fscanf(fp, "%f", &v);
		((double*)mat.ptr())[i] = v;
	}

	cout << mat << endl;

	fclose(fp);
	fp = NULL;
}

//================
// CameraParameter
//================
class CameraParameter {
public:
	Moca::CameraIntrinsic				intr;
	cv::Mat								extr; // convert to camera space

public:
	CameraParameter() {
		extr = cv::Mat::eye(4, 4, CV_64F);
	}

	void SaveCali(const char *path) {
		FILE *fp = fopen(path, "w");
		if (!fp) {
			return;
		}

		// camera matrix
		for (int i = 0; i < 9; i++) {
			fprintf(fp, "%f ", ((double*)intr.cameraMatrix.ptr())[i]);
		}
		fprintf(fp, "\n");

		// distortion
		for (int i = 0; i < 5; i++) {
			fprintf(fp, "%f ", ((double*)intr.distCoeffs.ptr())[i]);
		}
		fprintf(fp, "\n");

		// kinect to camera
		for (int i = 0; i < 16; i++) {
			fprintf(fp, "%f ", ((double*)extr.ptr())[i]);
		}
		fprintf(fp, "\n");

		fclose(fp);
		fp = NULL;
	}

	void LoadCali(const char *path) {
		FILE *fp = fopen(path, "r");
		if (!fp) {
			return;
		}

		// camera matrix
		for (int i = 0; i < 9; i++) {
			float v;
			fscanf(fp, "%f ", &v);
			((double*)intr.cameraMatrix.ptr())[i] = v;
		}

		// distortion
		for (int i = 0; i < 5; i++) {
			float v;
			fscanf(fp, "%f ", &v);
			((double*)intr.distCoeffs.ptr())[i] = v;
		}

		// kinect to camera
		for (int i = 0; i < 16; i++) {
			float v;
			fscanf(fp, "%f ", &v);
			((double*)extr.ptr())[i] = v;
		}

		fclose(fp);
		fp = NULL;

		intr.InitUndistortMap(cv::Size(512, 424));

		#if 1
		cout << "Load Cali" << endl;
		cout << intr.cameraMatrix << endl;
		cout << intr.distCoeffs << endl;
		cout << extr << endl;
		#endif
	}
};

//================
// Calibrator for Camera
//================
class Calibrator_Camera {
public:
	// frames for calibration
	cv::Size							imgSize;
	vector< vector< cv::Point2f > >		imgPoints;

public:
	void ResetFrames(void) {
		imgPoints.clear();
	}

	void CaptureFrame(const cv::Size &size, const vector< cv::Point2f > &corners) {
		imgPoints.push_back(corners);
		imgSize = size;

		cout << "capture : " << imgPoints.size() << endl;
	}

	void Calibrate(CameraParameter &camera, const Pattern &pattern2d) {
		vector< RigidTransform > m;

		// intr
		Moca::CalibrateCameraWithChessboard(imgPoints, imgSize, pattern2d, camera.intr, m);
	}
};


//================
// Calibrator for Vicon
//================
class Calibrator_Extr {
public:
	// frames for calibration
	vector< cv::Mat >					mats_p_to_v;

public:
	void ResetFrames(void) {
		mats_p_to_v.clear();
	}

	void CaptureFrame(const cv::Mat &p_to_v) {
		mats_p_to_v.push_back(p_to_v);

		cout << "capture vicon : " << mats_p_to_v.size() << endl;
	}

	// P(img<-camera) * M(camera<-kinect) * M(kinect<-vicon) * M(vicon<-pat) * V(pat) = V(img)
	// M(camera<-kinect) = M(camera<-pat) * M(pat<-vicon) * M(vicon<-kinect)
	void Calibrate(CameraParameter &camera, const Pattern &pattern3d, const vector< vector< cv::Point2f > > &imgPoints) {
		// extr mats_k_c
		// kinect space -> camera space
		vector< cv::Point3f > points_obj;
		vector< cv::Point2f > points_img;

		// P(img<-camera) * M(camera<-kinect) * V(kinect) = V(img)
		for (int k = 0; k < imgPoints.size(); k++) {
			for (int i = 0; i < pattern3d.corners.size(); i++) {
				points_obj.push_back((mats_p_to_v[k] * pattern3d.corners[i]));
				points_img.push_back(imgPoints[k][i]);
			}
		}

		Moca::EstimateCameraPose(points_obj, points_img, camera.intr, camera.extr);

		cout << "vicon : " << camera.extr << endl;
	}

	void CalibrateNikon(CameraParameter &camera, const Pattern &pattern3d, const vector< vector< cv::Point2f > > &imgPoints, vector<int> &failList) {
		// extr mats_k_c
		// kinect space -> camera space
		vector< cv::Point3f > points_obj;
		vector< cv::Point2f > points_img;

		// P(img<-camera) * M(camera<-kinect) * V(kinect) = V(img)
		int j = 0;
		bool validFailList = !failList.empty();
		for (int k = 0; k < imgPoints.size(); k++) {
			if (validFailList && k == failList[j]){
				j++;
				break;
			}
			for (int i = 0; i < pattern3d.corners.size(); i++) {
				points_obj.push_back(mats_p_to_v[k] * pattern3d.corners[i]);
				points_img.push_back(imgPoints[k][i]);
			}
		}

		Moca::EstimateCameraPose(points_obj, points_img, camera.intr, camera.extr);

		cout << "vicon : " << camera.extr << endl;
	}
};


//================
// Calibrator for Vicon
//================
class Calibrator_Vicon {
public:
	// frames for calibration
	vector< cv::Mat >					mats_p_to_v;
	vector< cv::Mat >					mats_k_to_v;

public:
	void ResetFrames(void) {
		mats_p_to_v.clear();
		mats_k_to_v.clear();
	}

	void CaptureFrame(const cv::Mat &p_to_v, const cv::Mat &k_to_v) {
		mats_p_to_v.push_back(p_to_v);
		mats_k_to_v.push_back(k_to_v);

		cout << "capture vicon : " << mats_p_to_v.size() << endl;
	}

	// P(img<-camera) * M(camera<-kinect) * M(kinect<-vicon) * M(vicon<-pat) * V(pat) = V(img)
	// M(camera<-kinect) = M(camera<-pat) * M(pat<-vicon) * M(vicon<-kinect)
	void Calibrate(CameraParameter &camera, const Pattern &pattern3d, const vector< vector< cv::Point2f > > &imgPoints) {
		// extr mats_k_c
		// kinect space -> camera space
		vector< cv::Point3f > points_obj;
		vector< cv::Point2f > points_img;

		// P(img<-camera) * M(camera<-kinect) * V(kinect) = V(img)
		for (int k = 0; k < imgPoints.size(); k++) {
			for (int i = 0; i < pattern3d.corners.size(); i++) {
				points_obj.push_back(mats_k_to_v[k].inv() * (mats_p_to_v[k] * pattern3d.corners[i]));
				points_img.push_back(imgPoints[k][i]);
			}
		}
		
		Moca::EstimateCameraPose(points_obj, points_img, camera.intr, camera.extr);

		cout << "vicon : " << camera.extr << endl;
	}
};


class Calibratior_Stereo {
public :
	cv::Size							imgSize0;
	cv::Size							imgSize1;
	vector< vector< cv::Point2f > >		imgPoints0;
	vector< vector< cv::Point2f > >		imgPoints1;

public :
	void ResetFrames(void) {
		imgPoints0.clear();
		imgPoints1.clear();
	}

	void CaptureFrame(const cv::Size &size0, const cv::Size &size1, vector< cv::Point2f > &corners0, const vector< cv::Point2f > &corners1) {
		imgPoints0.push_back(corners0);
		imgPoints1.push_back(corners1);
		imgSize0 = size0;
		imgSize1 = size1;

		cout << "capture stereo : " << imgPoints0.size() << endl;
	}

	// extr: cam0->cam1
	cv::Mat CalibrateCam0To1(const CameraParameter &camera0,  Moca::CameraParameter &camera1, const Pattern &pattern2d) {
		if (imgPoints0.size() <= 0 || imgPoints1.size() <= 0) {
			return cv::Mat::eye(4, 4, CV_64FC1);
		}

		// object points from squares
		vector< vector< cv::Point3f > > objPoints(imgPoints0.size());
		for (int i = 0; i < objPoints.size(); i++) {
			objPoints[i] = pattern2d.corners;
		}

		RigidTransform m;
		cv::Mat R = cv::Mat::ones(3, 3, CV_64F);
		double error = cv::stereoCalibrate(
			objPoints, imgPoints0, imgPoints1,
			camera0.intr.cameraMatrix, camera0.intr.distCoeffs,
			camera1.intr.cameraMatrix, camera1.intr.distCoeffs,
			imgSize0, R, m.tvec, m.evec, m.fvec,
			cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5)
			);

		Rodrigues(R, m.rvec);

		cout << "debug\n" << m.tvec << endl << m.rvec << endl;

		cout << m.Mat() << endl;

		cout << "stereo calibration : " << error << endl;

		return m.Mat();
	}

	// extr: cam0->cam1
	void Calibrate(const CameraParameter &camera0, Moca::CameraParameter &camera1, const Pattern &pattern2d) {
		if (imgPoints0.size() <= 0) {
			return;
		}
		
		// object points from squares
		vector< vector< cv::Point3f > > objPoints(imgPoints0.size());
		for (int i = 0; i < objPoints.size(); i++) {
			objPoints[i] = pattern2d.corners;
		}

		RigidTransform m;
		cv::Mat R = cv::Mat::ones( 3, 3, CV_64F );
		double error = cv::stereoCalibrate(
			objPoints, imgPoints0, imgPoints1,
			camera0.intr.cameraMatrix, camera0.intr.distCoeffs,
			camera1.intr.cameraMatrix, camera1.intr.distCoeffs,
			imgSize0, R, m.tvec, m.evec, m.fvec,
			cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5)
			);

		Rodrigues(R, m.rvec);
		
		cout << "debug\n" << m.tvec << endl << m.rvec << endl;
		
		camera1.extr = m.Mat() * camera0.extr;

		cout << m.Mat() << endl;

		cout << "stereo calibration : " << error << endl << camera1.extr << endl;
	}
};

}; // namespace Moca

#endif // Moca_cameraCalibration_h
