//
//  viconTrack.h
//  Moc
//
//  Created by Wei Li on 4/24/15.
//

#ifndef Moc_viconTrack_h
#define Moc_viconTrack_h

#include <memory>
using namespace std;

#include <vrpn_Tracker.h>

class VRPN_Tracker {
public :
	string									m_name;
	vrpn_TRACKERCB							m_data;
	shared_ptr< vrpn_Tracker_Remote >		m_tracker;

public :
	VRPN_Tracker() {
	}

	VRPN_Tracker( const char *name ) {
		Create( name );
	}

	void Create( const char *name ) {
		m_name = name;
		m_tracker.reset( new vrpn_Tracker_Remote( m_name.c_str() ) );
		if ( m_tracker ) {
			m_tracker->register_change_handler( &m_data, callback );
		}
	}

	void Delete( void ) {
		m_name.clear();
		m_tracker.reset();
	}

	void Loop( void ) {
		if ( m_tracker ) {
			m_tracker->mainloop();
		}
	}

	cv::Mat Mat( void ) const {
		return TranslateRotate( cv::Point3d( m_data.pos[0], m_data.pos[1], m_data.pos[2] ), QuatToMat( m_data.quat[0], m_data.quat[1], m_data.quat[2], m_data.quat[3] ) );
	}

	void Log( void ) const {
//		printf( "time : %ld\n", m_data.msg_time.tv_sec );
//		printf( "t : %f %f %f\n", m_data.pos[0], m_data.pos[1], m_data.pos[2] );
//		printf( "q : %f %f %f %f\n", m_data.quat[0], m_data.quat[1], m_data.quat[2], m_data.quat[3] );
	}

public :
	static void VRPN_CALLBACK callback(void* user_data, const vrpn_TRACKERCB tData) {
		memcpy( user_data, &tData, sizeof( tData ) );
		//printf("time : %ld\n", tData.msg_time.tv_sec);
		//printf("t : %f %f %f\n", tData.pos[0], tData.pos[1], tData.pos[2]);
		//printf("q : %f %f %f %f\n", tData.quat[0], tData.quat[1], tData.quat[2], tData.quat[3]);
	}
};

#endif
