//
//  viconTrack.h
//  Moc
//
//  Created by Wei Li on 4/24/15.
//

#ifndef Moca_viconTrack_h
#define Moca_viconTrack_h

#include <memory>
#include <string>
#include <vector>

#include <vrpn_Tracker.h>

class VRPN_Tracker {
public:
	std::string									m_name;
	std::string									m_id;
	std::string									m_ip;
	std::shared_ptr< vrpn_Tracker_Remote >		m_tracker;

public:
	VRPN_Tracker() {
	}

	VRPN_Tracker(const char *id, const char *ip, void *data, vrpn_TRACKERCHANGEHANDLER handler) {
		Create(id, ip, data, handler);
	}

	void Create(const char *id, const char *ip, void *data, vrpn_TRACKERCHANGEHANDLER handler) {
		m_id = id;
		m_ip = ip;
		m_name = m_id + "@" + m_ip;

		m_tracker.reset(new vrpn_Tracker_Remote(m_name.c_str()));
		if (m_tracker) {
			m_tracker->register_change_handler(data, handler);
		}
	}

	void Loop(void) {
		if (m_tracker) {
			m_tracker->mainloop();
		}
	}
};

static void VRPN_CALLBACK sample_callback(void* user_data, const vrpn_TRACKERCB tData) {
	VRPN_Tracker *tracker = (VRPN_Tracker*)user_data;

	printf("[Sensor %d : %s]: \n", tData.sensor, tracker->m_id.c_str());
	printf("time : %ld %ld\n", tData.msg_time.tv_sec, tData.msg_time.tv_usec);
	printf("t : %f %f %f\n", tData.pos[0], tData.pos[1], tData.pos[2]);
	printf("q : %f %f %f %f\n\n", tData.quat[0], tData.quat[1], tData.quat[2], tData.quat[3]);
}

/*
	void Log( void ) const {
		printf( "time : %ld\n", m_data.msg_time.tv_sec );
		printf( "t : %f %f %f\n", m_data.pos[0], m_data.pos[1], m_data.pos[2] );
		printf( "q : %f %f %f %f\n", m_data.quat[0], m_data.quat[1], m_data.quat[2], m_data.quat[3] );
	}
	 
public :
	static void VRPN_CALLBACK callback(void* user_data, const vrpn_TRACKERCB tData) {
		memcpy( user_data, &tData, sizeof( tData ) );
		printf("time : %ld\n", tData.msg_time.tv_sec);
		printf("t : %f %f %f\n", tData.pos[0], tData.pos[1], tData.pos[2]);
		printf("q : %f %f %f %f\n", tData.quat[0], tData.quat[1], tData.quat[2], tData.quat[3]);
	}
};
*/
#endif
