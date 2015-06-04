//
//  sample.h
//  Moc
//
//  Created by Wei Li on 6/4/15.
//

#ifndef Moc_sample_h
#define Moc_sample_h

#include "cameraCalibration.h"
#include "viconTrack.h"
#include "freenect.h"
#include "fusion.h"

void sample_main_vicon_track( void ) {
	VRPN_Tracker vrpn_pattern, vrpn_kinect;
	vrpn_pattern.Create( "PATTERNLARGE@192.168.1.3" );
	vrpn_kinect.Create( "KINECT@192.168.1.3" );

	while( 1 ) {
		// update vrpn
		vrpn_pattern.Loop();
		vrpn_kinect.Loop();

		vrpn_pattern.Log();
		vrpn_kinect.Log();

		if ( getchar() == 'Q' ) {
			break;
		}
	}
}

void sample_main_calibrate_vicon_kinect( void ) {
	//=====================
	// Vicon
	//=====================
	VRPN_Tracker vrpn_pattern, vrpn_kinect;
	vrpn_pattern.Create( "PATTERNLARGE@192.168.1.3" );
	vrpn_kinect.Create( "KINECT@192.168.1.3" );

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
	vector< cv::Point3d > markers(4);
	markers[1].x = -0.226152709960938;
	markers[1].y = -0.383085083007813;
	markers[1].z = -0.281757385253906;

	markers[3].x = 0.140245666503906;
	markers[3].y = -0.396933837890625;
	markers[3].z = 0.177871490478516;

	markers[2].x = 0.153394592285156;
	markers[2].y = 0.385762634277344;
	markers[2].z = 0.190735992431641;

	markers[0].x = -0.213301147460938;
	markers[0].y = 0.400469909667969;
	markers[0].z = -0.270611022949219;
	cali.SetChessboard( cv::Size( 11, 8 ), 0.0655, markers );

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
		cv::imshow( "ir", kinect.img_ir / 2000.0f );

		// capture new frame
		if ( getchar() == 'F' ) {
			cali.CaptureFrame( kinect.img_ir, vrpn_pattern.Mat(), vrpn_kinect.Mat() );
		}

		// calibrate kinect and save config
		if ( getchar() == 'C' ) {
			cali.Calibrate();
			cali.SaveCali( "cam_cali.txt" );
			break;
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
		cali[ i ].SetChessboard( cv::Size( 11, 8 ), 0.0655 );

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

		// capture new frame
		if ( getchar() == 'F' ) {
			for ( int i = 0; i < cali.size(); i++ ) {
				cali[ i ].CaptureFrame( kinect.img_ir );
			}
		}

		// calibrate kinect and save config
		if ( getchar() == 'C' ) {
			for ( int i = 0; i < cali.size(); i++ ) {
				cali[ i ].Calibrate();
			}
			break;
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
	vector< cv::Point3d > src;
	src.push_back( cv::Point3d(344.876770, 21.191553, -8.421365) );
	src.push_back( cv::Point3d(329.321472, 21.196106, 112.414787) );
	src.push_back( cv::Point3d(273.297333, -54.085236, 103.468338) );
	src.push_back( cv::Point3d(283.640472, -49.967205, -16.086639) );

	// vicon
	vector< cv::Point3d > dst;
	dst.push_back( cv::Point3d( 57.7072, 108.886, 180.456 ) );
	dst.push_back( cv::Point3d( 176.59, 108.459, 192.929 ) );
	dst.push_back( cv::Point3d( 170.349, 38.8436, 247.97 ) );
	dst.push_back( cv::Point3d( 51.4353, 40.1155, 236.71 ) );

	cv::Mat M;
	Moc::PairPointsRigidRegistration( src, dst, M );
}

#endif
