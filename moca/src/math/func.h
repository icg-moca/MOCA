int ceil( int x, int m ) {
	x += m - 1;
	return x - x % m;
}