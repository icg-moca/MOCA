
	bool ExportPly( const char *path, const EPH_List<EPH_Vec4> &points, const EPH_List<EPH_Vec4> &normals ) {
		FILE *fp = fopen( path, "w" );
		if ( !fp ) {
			return false;
		}

		// type
		fprintf( fp, "ply\n" );
		
		// version
		fprintf( fp, "format ascii 1.0\n" );
		
		int count = 0;
		for ( int i = 0; i < points.Num(); i++ ) {
			count += ( points[i] != 0 );
		}
		
		// vertex
		fprintf( fp, "element vertex %d\n", count );
		fprintf( fp, "property float x\n" );
		fprintf( fp, "property float y\n" );
		fprintf( fp, "property float z\n" );
		fprintf( fp, "property float nx\n" );
		fprintf( fp, "property float ny\n" );
		fprintf( fp, "property float nz\n" );
//		fprintf( fp, "property uchar red\n" );
//		fprintf( fp, "property uchar green\n" );
//		fprintf( fp, "property uchar blue\n" );
		
//		fprintf( fp, "property list uchar int vertex_index\n" );
		
		fprintf( fp, "end_header\n" );
		
		// verts
		for ( int i = 0; i < points.Num(); i++ ) {
			if ( points[i] != 0 ) {
#if 1
				fprintf( fp, "%f %f %f %f %f %f\n",
					points [i][0], points [i][1], points [i][2],
					normals[i][0], normals[i][1], normals[i][2]
				);
#else
				EPH_Vec3 col = ( normals[i] * 0.5 + 0.5 ).Sub<3>();

				col[0] = pow( col[0], 1.0f ) * 255.0f;
				col[1] = pow( col[1], 1.0f ) * 255.0f;
				col[2] = pow( col[2], 1.0f ) * 255.0f;

				fprintf( fp, "%f %f %f %d %d %d\n",
					points [i][0], points [i][1], points [i][2],
					(int)(col[0]), (int)(col[1]), (int)(col[2])
				);
#endif
			}
		}
		
		fclose( fp );
		
		return true;
	}