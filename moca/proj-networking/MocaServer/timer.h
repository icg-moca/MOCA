#pragma once

#include <windows.h>

class Timer {
public :
	double start;
	double inv;

public :
	Timer() {
		LARGE_INTEGER freq;
		if ( !QueryPerformanceFrequency( &freq ) ) {
			inv = 0.0;
		} else {
			inv = 1.0 / ( double )freq.QuadPart;
		}
	}

	void Start( void ) {
		start = GlobalTime();
	}

	void Print( const char *str ) const {
		printf( ">> %s : %f\n", str, (GlobalTime() - start)*1000 );
	}

	double GlobalTime( void ) const {
		LARGE_INTEGER current;
		if ( !QueryPerformanceCounter( &current ) ) {
			return 0.0;
		}
		return ( double )current.QuadPart * inv;
	}

	double Record() {
		return ((GlobalTime() - start) * 1000);
	}
};