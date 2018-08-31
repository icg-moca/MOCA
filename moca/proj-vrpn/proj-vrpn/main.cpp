#include "vicon_tracker.h"


//=====================
// Test Vicon
//=====================
void sample_main_vicon_track(void) {
	const char *ip = "192.168.10.1";
	VRPN_Tracker vrpn_pattern, vrpn_kinect;
	vrpn_pattern.Create("PATTERN", ip, &vrpn_pattern, sample_callback);
	vrpn_kinect.Create("KINECT0", ip, &vrpn_kinect, sample_callback);

	while (1) {
		// update vrpn
		vrpn_pattern.Loop();
		vrpn_kinect.Loop();
	}
}