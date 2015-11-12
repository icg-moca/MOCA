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

void sample_main_vicon_track( void ) {
	const char *ip = "192.168.10.1";
	VRPN_Tracker vrpn_pattern, vrpn_kinect;
	vrpn_pattern.Create( "PATTERN", ip, &vrpn_pattern, sample_callback );
	vrpn_kinect .Create( "KINECT0", ip, &vrpn_kinect , sample_callback );

	while( 1 ) {
		// update vrpn
		vrpn_pattern.Loop();
		vrpn_kinect.Loop();
	}
}

void sample_main_kinect( void ) {
	//=====================
	// Kinect
	//=====================
	Freenect kinect;
	kinect.InitDevices();

	//=====================
	// Loop
	//=====================
	while( 1 ) {
		// update kinect
		kinect.Loop();
		cv::imshow( "ir", kinect.img_ir / 2000.0f );
		cv::waitKey(1);
	}
}

cv::Mat vrpn_data_to_mat(vrpn_TRACKERCB data) {
	return TranslateRotate(cv::Point3d(data.pos[0], data.pos[1], data.pos[2]), QuatToMat(data.quat[0], data.quat[1], data.quat[2], data.quat[3]));
}

static void VRPN_CALLBACK sample_cali_callback(void* user_data, const vrpn_TRACKERCB tData) {
	cv::Mat *mat = (cv::Mat*)user_data;

	*mat = vrpn_data_to_mat(tData);
}

void sample_main_calibrate_vicon_kinect( void ) {
	//=====================
	// Vicon
	//=====================
	const char *ip = "192.168.10.1";
	VRPN_Tracker vrpn_pattern, vrpn_kinect;
	cv::Mat mat_pattern = cv::Mat_<double>(4, 4);
	cv::Mat mat_kinect = cv::Mat_<double>(4, 4);
	vrpn_pattern.Create("PATTERN", ip, &mat_pattern, sample_cali_callback);
	vrpn_kinect.Create("KINECT0", ip, &mat_kinect, sample_cali_callback);

	//=====================
	// Kinect
	//=====================
	Freenect kinect;
	kinect.InitDevices();

	//=====================
	// Calibrator
	//=====================
	Moc::Calibrator_Kinect_To_Vicon cali;

	// 1 --- 2
	// |     |
	// 0 --- 3
	// size : inner corner
	// square_size : meter

	// init chessboard
	// FIXME : unit vsk coordinates is mm
	vector< cv::Point3f > markers(4);

	cv::Point3f vec_1, vec_2, norm_pattern;

	markers[0].x = -306.10238647460937;
	markers[0].y = -51.083488464355469;
	markers[0].z = -392.67129516601562;

	markers[1].x = -304.46176147460938;
	markers[1].y = 78.798492431640625;
	markers[1].z = 305.35824584960937;

	markers[2].x = 503.78604125976562;
	markers[2].y = 21.791416168212891;
	markers[2].z = 314.55368041992187;

	markers[3].x = 502.2266845703125;
	markers[3].y = -108.68138885498047;
	markers[3].z = -383.49655151367187;

	//subtract the height of the marker
	vec_1 = markers[0] - markers[1];
	vec_2 = markers[2] - markers[1];

	norm_pattern = vec_1.cross(vec_2) / (float)(cv::norm(vec_1.cross(vec_2))) * 10.0f;

	for (int i = 0; i < markers.size(); i++) {
		markers[i] = ( markers[i] - norm_pattern ) * 0.001f;
	}

	cali.SetChessboard( cv::Size( 7, 6 ), 0.1016, markers );

	// reset frames
	cali.ResetFrames();

	//=====================
	// Loop
	//=====================
	while( 1 ) {
		// update vrpn
		vrpn_pattern.Loop();
		vrpn_kinect.Loop();

		// update kinect
		kinect.Loop();

		if ( cali.isCalibrated ) {
			vector<cv::Point2f> proj;
			Moc::ProjectPoints( cali.board_corners, cali.cam_intr, cali.cam_extr * mat_kinect.inv() * mat_pattern, proj );
			for ( int i = 0; i < proj.size(); i++ ) {
				cv::Point pt( proj[i].x, proj[i].y );
				cv::Scalar color( 0, 0, 255 );
				cv::circle( kinect.img_ir, pt, 3, color, 1, CV_AA, 0 );
			}
		}

		cv::imshow( "ir", kinect.img_ir / 2000.0f );
		int key = cv::waitKey(1);

		// capture new frame
		if ( key == '0' ) {
			cali.CaptureFrame( kinect.img_ir, mat_pattern, mat_kinect );
		}

		// calibrate kinect and save config
		if ( key == '1' ) {
			cali.Calibrate();
			string name = vrpn_kinect.m_id + "_cali.txt";
			cali.SaveCali( name.c_str() );
		}
	}
}

void sample_main_calibrate_kinect_kinect( void ) {
	//=====================
	// Kinect
	//=====================
	Freenect kinect;
	kinect.InitDevices();

	//=====================
	// Calibrator
	//=====================
	vector< Moc::Calibrator_Kinect > cali;
	cali.resize( 2 );

	for ( int i = 0; i < cali.size(); i++ ) {
		// init chessboard
		cali[ i ].SetChessboard( cv::Size( 8, 7 ), 0.1016 );

		// reset frames
		cali[ i ].ResetFrames();
	}

	//=====================
	// Loop
	//=====================
	while( 1 ) {
		// update kinect
		kinect.Loop();
		cv::imshow( "ir", kinect.img_ir / 2000.0f );
		int key = cv::waitKey(1);

		// capture new frame
		if ( key == '0' ) {
			for ( int i = 0; i < cali.size(); i++ ) {
				cali[ i ].CaptureFrame( kinect.img_ir );
			}
		}

		// calibrate kinect and save config
		if ( key == '1' ) {
			for ( int i = 0; i < cali.size(); i++ ) {
				cali[ i ].Calibrate();
			}
		}
	}

	vector< cv::Mat > extr;
	extr.resize( cali.size() );
	extr[0] = cv::Mat::eye( 4, 4, CV_64F );
	for ( int i = 1; i < extr.size(); i++ ) {
		// improve by projection error
		extr[ i ] = cali[ 0 ].cam_extr[ 0 ].Mat() * cali[ 1 ].cam_extr[ 0 ].Mat().inv();
	}
}

void sample_main_calibrate_vicon_velodyne( void ) {
	// velodyne
	vector< cv::Point3f > src;
	src.push_back( cv::Point3f(344.876770, 21.191553, -8.421365) );
	src.push_back( cv::Point3f(329.321472, 21.196106, 112.414787) );
	src.push_back( cv::Point3f(273.297333, -54.085236, 103.468338) );
	src.push_back( cv::Point3f(283.640472, -49.967205, -16.086639) );

	// vicon
	vector< cv::Point3f > dst;
	dst.push_back( cv::Point3f( 57.7072, 108.886, 180.456 ) );
	dst.push_back( cv::Point3f( 176.59, 108.459, 192.929 ) );
	dst.push_back( cv::Point3f( 170.349, 38.8436, 247.97 ) );
	dst.push_back( cv::Point3f( 51.4353, 40.1155, 236.71 ) );

	cv::Mat M;
	Moc::PairPointsRigidRegistration( src, dst, M );
}

