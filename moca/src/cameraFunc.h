void LoadCamera( const char *path, EPH_Vec4 intr[], EPH_Mat4 extr[] ) {
	EPH_LoadBitFile file;
	file.Open( path );

	for ( int k = 0; k < 16; k++ ) {
		for ( int i = 0; i < 4; i++ ) {
			double v;
			file.Read(v);

			intr[k].Ptr()[i] = v;

		}

		for ( int i = 0; i < 16; i++ ) {
			double v;
			file.Read(v);

			extr[k].Ptr()[i] = v;
		}

		printf( "================ %d\n", k );
		intr[k].Print();
		extr[k][0].Print();
		extr[k][1].Print();
		extr[k][2].Print();
		extr[k][3].Print();
	}
}

void SaveCamera( const char *path, EPH_Vec4 intr[], EPH_Mat4 extr[] ) {
	EPH_SaveBitFile file;
	file.Open( path );
	
	printf( "save camera : %s\n", path );

	for ( int k = 0; k < 16; k++ ) {

		for ( int i = 0; i < 4; i++ ) {
			double v = intr[k].Ptr()[i];
			file.Write(v);
		}

		for ( int i = 0; i < 16; i++ ) {
			double v = extr[k].Ptr()[i];
			file.Write(v);
		}
	}
}