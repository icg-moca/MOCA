
template< class T >
int BinarySearch( int num, const T *data, const T &v ) {
	if ( num <= 0 ) {
		return -1;
	}

	if ( v < data[0] ) { // - 1
		return -1;
	}

	if ( v > data[num - 1] ) { // num
		return -1; // FIXME
	}

	// range : [start, end)
	int start = 0;
	int end = num;

	while ( end > start ) {
		int half = ( start + end ) / 2;

		if ( v < data[half] ) {
			end = half;
		} else {
			start = half + 1;
		}
	}

	return start;
}