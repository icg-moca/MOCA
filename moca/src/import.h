bool read_text_file( const char *path, std::vector<char> &text ) {
	text.clear();

	FILE *fp = fopen( path, "rb" );
	if ( !fp ) {
		printf( "failed to read text file: %s\n", path );
		return false;
	}

	fseek( fp, 0, SEEK_END );
	size_t size = ftell( fp );

	text.resize( size + 1 );
	fseek( fp, SEEK_SET, 0 );
	fread( text.data(), 1, size, fp );
	text.push_back( 0 );

	fclose( fp );
	return true;
}