#pragma once

#include <windows.h>
#include <iostream>
class Timer {
	// Getting time duration
public:
	double start;
	double inv;

	SYSTEMTIME lt;
public:
	Timer() {
		LARGE_INTEGER freq;
		if (!QueryPerformanceFrequency(&freq)) {
			inv = 0.0;
		}
		else {
			inv = 1.0 / (double)freq.QuadPart;
		}
	}

	void Start(void) {
		start = GlobalTime();
	}

	void Print(const char *str) const {
		printf(">> %s : %f\n", str, (GlobalTime() - start) * 1000);
	}

	double Record(){
		return ((GlobalTime() - start) * 1000);
	}

	double GlobalTime(void) const {
		LARGE_INTEGER current;
		if (!QueryPerformanceCounter(&current)) {
			return 0.0;
		}
		return (double)current.QuadPart * inv;
	}
	// Getting current time stamp (For global syn)

};
/*
private:
	boost::posix_time::time_duration start_time;
	boost::posix_time::ptime t;
public:
	
	std::string now_str()
	{
		// Get current time from the clock, using microseconds resolution
		const boost::posix_time::ptime now =
			boost::posix_time::microsec_clock::local_time();

		// Get the time offset in current day
		const boost::posix_time::time_duration td = now.time_of_day();

		//
		// Extract hours, minutes, seconds and milliseconds.
		//
		// Since there is no direct accessor ".milliseconds()",
		// milliseconds are computed _by difference_ between total milliseconds
		// (for which there is an accessor), and the hours/minutes/seconds
		// values previously fetched.
		//
		const long hours = td.hours();
		const long minutes = td.minutes();
		const long seconds = td.seconds();
		const long milliseconds = td.total_milliseconds() -
			((hours * 3600 + minutes * 60 + seconds) * 1000);

		//
		// Format like this:
		//
		//      hh:mm:ss.SSS
		//
		// e.g. 02:15:40:321
		//
		//      ^          ^
		//      |          |
		//      123456789*12
		//      ---------10-     --> 12 chars + \0 --> 13 chars should suffice
		//  
		// 
		char buf[40];
		//sprintf(buf, "%02ld%02ld%02ld%03ld",
		//	hours, minutes, seconds, milliseconds);
		std::cout << " <<< " << td.total_milliseconds() << std::endl;

		return buf;
	}

	long cal_dur(){
		boost::posix_time::ptime t2 = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff = t2 - t;
		boost::posix_time::time_duration diff1 = t2.time_of_day();
		std::cout << " <<< record "<< diff1.total_milliseconds() << std::endl;
	//	std::cout << " <<< duration " << diff.total_milliseconds() << std::endl;

		//return ( diff1.total_milliseconds() - start_time.total_microseconds());
		return diff.total_milliseconds();
	}
};
//*/