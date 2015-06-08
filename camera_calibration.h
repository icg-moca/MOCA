//
//  moc_cameraCalibration.h
//  Moc
//
//  Created by Wei Li on 4/24/15.
//

#ifndef Moc_cameraCalibration_h
#define Moc_cameraCalibration_h

#include <stdio.h>

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_op.h>

namespace Moc {

//================
// CameraIntrinsic
//================
class CameraIntrinsic {
public :
	cv::Mat		cameraMatrix;
	cv::Mat		distCoef;
	cv::Mat		undistMap[ 2 ];

public :
	CameraIntrinsic() {
		cameraMatrix = cv::Mat::eye  ( 3, 3, CV_64F );
		distCoef     = cv::Mat::zeros( 8, 1, CV_64F );
	}
};

//================
// RigidTransform
//================
class RigidTransform {
public :
	cv::Mat		rvec;
	cv::Mat		tvec;

public :
	RigidTransform() {
		rvec = cv::Mat::zeros( 3, 1, CV_64F );
		tvec = cv::Mat::zeros( 3, 1, CV_64F );
	}

	cv::Mat Mat( void ) const {
		cv::Mat m = cv::Mat::eye( 4, 4, CV_64F );

		m.at<double>( 0, 3 ) = tvec.at<double>( 0, 0 );
		m.at<double>( 1, 3 ) = tvec.at<double>( 1, 0 );
		m.at<double>( 2, 3 ) = tvec.at<double>( 2, 0 );

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

double ReprojectionError(
	const vector< cv::Point3f > &objectPoints,
	const vector< cv::Point2f > &imagePoints,
	const CameraIntrinsic &intr, const RigidTransform &extr
) {
	// project
	vector< cv::Point2f > proj;
	cv::projectPoints( cv::Mat( objectPoints ), extr.rvec, extr.tvec, intr.cameraMatrix, intr.distCoef, proj );

	// difference
	double err = cv::norm( cv::Mat( imagePoints ), cv::Mat( proj ), CV_L2 );

	// rms
	return std::sqrt( err * err / ( double )objectPoints.size() );
}

void UndistortImage( const cv::Mat &src, cv::Mat &dst, const CameraIntrinsic &intr ) {
	cv::remap( src, dst, intr.undistMap[0], intr.undistMap[1], cv::INTER_LINEAR );
}

void CalcChessboardCornersFromSquare( cv::Size boardSize, float squareSize, vector< cv::Point3f > &corners ) {
	corners.clear();
	corners.reserve( boardSize.area() );
	for( int y = 0; y < boardSize.height; y++ ) {
		for( int x = 0; x < boardSize.width; x++ ) {
			corners.push_back( cv::Point3f( x * squareSize, y * squareSize, 0 ) );
		}
	}
}

// corners
// 1 --- 2
// |     |
// 0 --- 3
void CalcChessboardCornersFromMarkers( const vector< cv::Point3f > &markers, cv::Size boardSize, vector< cv::Point3f > &corners ) {
	corners.clear();
	corners.reserve( boardSize.area() );
	for( int y = boardSize.height; y >= 1 ; y-- ) {
		float fy = ( float )y / ( float )( boardSize.height + 1 );
		for( int x = 1; x <= boardSize.width; x++ ) {
			float fx = ( float )x / ( float )( boardSize.width + 1 );
			cv::Point3f e0 = ( markers[3] - markers[0] ) * ( double )fx + markers[0];
			cv::Point3f e1 = ( markers[2] - markers[1] ) * ( double )fx + markers[1];
			cv::Point3f e  = ( e1 - e0 ) * ( double )fy + e0;
			corners.push_back( e );
		}
	}
}

bool FindChessboardCornersFromImage( const cv::Mat &image, cv::Size boardSize, vector< cv::Point2f > &corners ) {
	if ( !cv::findChessboardCorners( image, boardSize, corners ) ) {
		return false;
	}
	cv::Mat gray;
	cv::cvtColor( image, gray, CV_BGR2GRAY );
	cv::cornerSubPix( gray, corners, cv::Size( 2, 2 ), cv::Size( -1, -1 ), cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ) );
	return true;
}

bool FindChessboardCornersFromColorImage( const cv::Mat &image, cv::Size boardSize, vector< cv::Point2f > &corners ) {
	if ( !cv::findChessboardCorners( image, boardSize, corners ) ) {
		return false;
	}

	cv::Mat gray;
//	cv::cvtColor( image, gray, CV_BGR2GRAY );
//	cv::cornerSubPix( gray, corners, cv::Size( 2, 2 ), cv::Size( -1, -1 ), cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ) );
	return true;
}

void CalibrateCameraWithChessboard(
	const vector< vector< cv::Point2f > > &imgPoints, cv::Size imageSize, cv::Size boardSize, float squareSize,
	CameraIntrinsic &intr, vector< RigidTransform > &extr, int flag = CV_CALIB_FIX_K4|CV_CALIB_FIX_K5
) {
	if ( imgPoints.empty() ){
		return;
	}

	// object points from squares
	vector< vector< cv::Point3f > > objPoints;
	objPoints.resize( imgPoints.size() );

	CalcChessboardCornersFromSquare( boardSize, squareSize, objPoints[ 0 ] );
	for ( int i = 1; i < imgPoints.size(); i++ ) {
		objPoints[ i ] = objPoints[ 0 ];
	}

	if ( flag & CV_CALIB_FIX_ASPECT_RATIO ) {
		intr.cameraMatrix.at<double>( 0, 0 ) = 0.0; // FIXME
	}

	vector< cv::Mat > rvecs, tvecs;
	cv::calibrateCamera(
		objPoints, imgPoints, imageSize,
		intr.cameraMatrix, intr.distCoef, rvecs, tvecs, flag
	);

	extr.resize( rvecs.size() );
	for ( int i = 0; i < extr.size(); i++ ) {
		extr[ i ].rvec = rvecs[ i ];
		extr[ i ].tvec = tvecs[ i ];
	}

	// undistort
	cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix( intr.cameraMatrix, intr.distCoef, imageSize, 1, imageSize, 0 );
	cv::initUndistortRectifyMap(
		intr.cameraMatrix, intr.distCoef, cv::Mat(), newCameraMatrix,
		imageSize, CV_16SC2, intr.undistMap[ 0 ], intr.undistMap[ 1 ]
	);

	cout << "camera intr :" << endl;
	cout << intr.cameraMatrix << endl;
	cout << intr.distCoef << endl;
}

// overflow
void PairPointsRigidRegistration( const vector< cv::Point3f > &src, const vector< cv::Point3f > &dst, cv::Mat &M ) {
	cv::Point3f src_o = Sum( src ) / (float)src.size();
	cv::Point3f dst_o = Sum( dst ) / (float)dst.size();

	float s = 1;
	for ( int i = 0; i < src.size(); i++ ) {
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

// P(img<-camera) * M(camera<-kinect) * M(kinect<-vicon) * M(vicon<-pat) * V(pat) = V(img)
// M(camera<-kinect) = M(camera<-pat) * M(pat<-vicon) * M(vicon<-kinect)
double CalibrateTransformFromKinectToCamera(
	const vector< vector< cv::Point2f > > &imgPoints, const vector< cv::Mat > &mats_p_to_w, const vector< cv::Mat > &mats_k_to_w, const vector< cv::Point3f > &markers, cv::Size boardSize, const CameraIntrinsic &intr, cv::Mat &extr
) {
	// object points from markers
	vector< cv::Point3f > corners;
	CalcChessboardCornersFromMarkers( markers, boardSize, corners );

	double minErr = -1.0;
	for ( int i = 0; i < imgPoints.size(); i++ ) {
		RigidTransform m;
		if ( cv::solvePnP( cv::Mat( corners ), cv::Mat( imgPoints[ i ] ), intr.cameraMatrix, intr.distCoef, m.rvec, m.tvec ) ) {
			double err = ReprojectionError( corners, imgPoints[ i ], intr, m );
			if ( minErr < 0 || err < minErr ) {
				minErr = err;
				extr = m.Mat() * ( mats_p_to_w[i].inv() * mats_k_to_w[i] );
			}
		}
	}

	return minErr;
}

bool EstimatePoseWithChessboard(
	const vector< cv::Point3f > &objectPoints, const cv::Mat &image,
	cv::Size boardSize, const CameraIntrinsic &intr, RigidTransform &extr
) {
	vector< cv::Point2f > imagePoints;
	if ( !FindChessboardCornersFromImage( image, boardSize, imagePoints ) ) {
		return false;
	}
	return cv::solvePnP( cv::Mat( objectPoints ), cv::Mat( imagePoints ), intr.cameraMatrix, intr.distCoef, extr.rvec, extr.tvec, false );
}

void ProjectPoints( const vector<cv::Point3f> &objPoints, const CameraIntrinsic &intr, const cv::Mat &extr, vector<cv::Point2f> &imgPoints ) {
	vector< cv::Point3f > points( objPoints.size() );
	for ( int i = 0; i < objPoints.size(); i++ ) {
		points[i] = extr * objPoints[i];
	}
	cv::projectPoints( points, cv::Mat::zeros( 3, 1, CV_64F ), cv::Mat::zeros( 3, 1, CV_64F ), intr.cameraMatrix, intr.distCoef, imgPoints );
}

//================
// Calibrator for Kinect and Vicon
//================
class Calibrator_Kinect_To_Vicon {
public :
	// board
	cv::Size							board_size;
	float								board_square_size;
	vector< cv::Point3f >				board_markers;
	vector< cv::Point3f >				board_corners;
	cv::Size							img_size;

	// frames for calibration
	vector< vector< cv::Point2f > >		img_points;
	vector< cv::Mat >					mats_p_to_v;
	vector< cv::Mat >					mats_k_to_v;

	// camera
	Moc::CameraIntrinsic				cam_intr;
	cv::Mat								cam_extr;
	bool								isCalibrated;

public :
	Calibrator_Kinect_To_Vicon() {
		isCalibrated = false;

		cam_extr = cv::Mat::eye( 4, 4, CV_64F );
	}

	void SetChessboard( cv::Size size, float square_size, const vector< cv::Point3f > &markers ) {
		board_size = size;
		board_square_size = square_size;
		board_markers = markers;

		Moc::CalcChessboardCornersFromMarkers( board_markers, board_size, board_corners );
	}

	void ResetFrames( void ) {
		img_points.clear();
		mats_p_to_v.clear();
		mats_k_to_v.clear();

		isCalibrated = false;
	}

	void CaptureFrame( const cv::Mat &img_ir, const cv::Mat &p_to_v, const cv::Mat &k_to_v ) {
		cv::Mat cvImg( img_ir.size(), CV_8UC3 );
		int num = img_ir.size().area();
		for ( int i = 0; i < num; i++ ) {
			unsigned char c = EPH_Math::Clamp( ((float*)img_ir.ptr())[ i ] / 20.0f, 0.0f, 255.0f );
			cvImg.ptr()[ i * 3 + 0 ] = c;
			cvImg.ptr()[ i * 3 + 1 ] = c;
			cvImg.ptr()[ i * 3 + 2 ] = c;
		}

		vector< cv::Point2f > corners;
		if ( Moc::FindChessboardCornersFromImage( cvImg, board_size, corners ) ) {
			cv::drawChessboardCorners( cvImg, board_size, cv::Mat( corners ), true );
			img_points.push_back( corners );
			img_size = cvImg.size();

			mats_p_to_v.push_back( p_to_v );
			mats_k_to_v.push_back( k_to_v );

			cout << "capture : " << img_points.size() << endl;
		}
		cv::imshow( "corner", cvImg );
	}

	void Calibrate( void ) {
		vector< RigidTransform > extr;

		// intr
		Moc::CalibrateCameraWithChessboard( img_points, img_size, board_size, board_square_size, cam_intr, extr );

		// extr
		Moc::CalibrateTransformFromKinectToCamera( img_points, mats_p_to_v, mats_k_to_v, board_markers, board_size, cam_intr, cam_extr );

		cout << cam_extr << endl;
		isCalibrated = true;
	}

	void SaveCali( const char *path ) {
		FILE *fp = fopen( path, "w" );
		if ( !fp ) {
			return;
		}

		// camera matrix
		for ( int i = 0; i < 9; i++ ) {
			fprintf( fp, "%f ",  ((double*)cam_intr.cameraMatrix.ptr())[i] );
		}
		fprintf( fp, "\n" );

		// distortion
		for ( int i = 0; i < 5; i++ ) {
			fprintf( fp, "%f ",  ((double*)cam_intr.distCoef.ptr())[i] );
		}
		fprintf( fp, "\n" );

		// kinect to camera
		for ( int i = 0; i < 16; i++ ) {
			fprintf( fp, "%f ",  ((double*)cam_extr.ptr())[i] );
		}
		fprintf( fp, "\n" );
		
		fclose( fp );
		fp = NULL;
	}

	void LoadCali( const char *path ) {
		FILE *fp = fopen( path, "r" );
		if ( !fp ) {
			return;
		}

		// camera matrix
		for ( int i = 0; i < 9; i++ ) {
			float v;
			fscanf( fp, "%f ",  &v );
			((double*)cam_intr.cameraMatrix.ptr())[i] = v;
		}

		// distortion
		for ( int i = 0; i < 5; i++ ) {
			float v;
			fscanf( fp, "%f ",  &v );
			((double*)cam_intr.distCoef.ptr())[i] = v;
		}

		// kinect to camera
		for ( int i = 0; i < 16; i++ ) {
			float v;
			fscanf( fp, "%f ",  &v );
			((double*)cam_extr.ptr())[i] = v;
		}

		fclose( fp );
		fp = NULL;

		cout << "Load Cali" << endl;
		cout << cam_intr.cameraMatrix << endl;
		cout << cam_intr.distCoef << endl;
		cout << cam_extr << endl;

		isCalibrated = true;
	}
};

//================
// Calibrator for Kinect to Kinect
//================
class Calibrator_Kinect {
public :
	// board
	cv::Size							board_size;
	float								board_square_size;
	cv::Size							img_size;

	// frames for calibration
	vector< vector< cv::Point2f > >		img_points;

	// camera
	Moc::CameraIntrinsic				cam_intr;
	vector< RigidTransform >			cam_extr;

public :
	void SetChessboard( cv::Size size, float square_size ) {
		board_size = size;
		board_square_size = square_size;
	}

	void ResetFrames( void ) {
		img_points.clear();
		cam_extr.clear();
	}

	void CaptureFrame( const cv::Mat &img_ir ) {
		cv::Mat cvImg( img_ir.size(), CV_8UC3 );
		int num = img_ir.size().area();
		for ( int i = 0; i < num; i++ ) {
			unsigned char c = EPH_Math::Clamp( ((float*)img_ir.ptr())[ i ] / 20.0f, 0.0f, 255.0f );
			cvImg.ptr()[ i * 3 + 0 ] = c;
			cvImg.ptr()[ i * 3 + 1 ] = c;
			cvImg.ptr()[ i * 3 + 2 ] = c;
		}

		vector< cv::Point2f > corners;
		if ( Moc::FindChessboardCornersFromImage( cvImg, board_size, corners ) ) {
			cv::drawChessboardCorners( cvImg, board_size, cv::Mat( corners ), true );
			img_points.push_back( corners );
			img_size = cvImg.size();

			cout << "capture : " << img_points.size() << endl;
		}
		cv::imshow( "corner", cvImg );
	}

	void Calibrate( void ) {
		Moc::CalibrateCameraWithChessboard( img_points, img_size, board_size, board_square_size, cam_intr, cam_extr );
	}
};

}; // namespace Moc

#endif // Moc_cameraCalibration_h
