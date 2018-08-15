
// save
void SaveDepthMap( const char *path_format, int id, const Image< unsigned short > &depth )	{
	EPH_SaveBitFile file;
	file.Open( EPH_String().SetWithFormat( path_format, id ) );

	file.WriteArray( depth.Data(), depth.NumPixels() );
}

bool LoadDepthMap( const char *path, Image<unsigned short> &points ) {
	EPH_LoadBitFile file;
	if ( !file.Open( path ) ) {
		return false;
	}

	file.ReadArray(points.Data(), 512 * 424 );
	return true;
}

bool LoadTimeStamp( const char *path, EPH_List<double> &timeStamps) {
	EPH_TextFile file;
	if ( !file.Load( path ) ) {
		return false;
	}

	EPH_Lexer lexer( file.Text() );

	timeStamps.Empty();
	timeStamps.EnsureSize( 1024 );
	while( !lexer.ReachEnd() ) {
		double v;
		if ( !lexer.ParseDouble( v ) ) {
			return false;
		}
		
		timeStamps.Append( v );
	}

	return true;
}


void Dilation( const EPH_List< unsigned short > &src, EPH_List< unsigned short > &dst ) {
	for ( int y = 1; y < 424 - 1; y++ ) {
		for ( int x = 1; x < 512 - 1; x++ ) {
			int index = x + y * 512;
			if ( src[index] == 0 ) {
				dst[index] = EPH_Math::Max( EPH_Math::Max( src[index - 1], src[index + 1] ), EPH_Math::Max( src[index - 512], src[index + 512] ) );						
			} else {
				dst[index] = src[index];
			}
		}
	}
}

void Erosion( const Image< unsigned short > &src, Image< unsigned short > &dst ) {
	for ( int y = 1; y < 424 - 1; y++ ) {
		for ( int x = 1; x < 512 - 1; x++ ) {
			int index = x + y * 512;
			if ( src[index - 1] && src[index + 1] && src[index - 512] && src[index + 512 ] ) {
				dst[index] = src[index];
			} else {
				dst[index] = 0;
			}
		}
	}
}

void DepthConsistency( Image< unsigned short > &src ) {
	for ( int y = 0; y < src.Height(); y++ ) {
	for ( int x = 0; x < src.Width(); x++ ) {
		int index = x + y * src.Width();

		if ( x < 3 || y < 3 || x > src.Width() - 3 || y > src.Height() - 3 ) {
			src[index] = 0;
			continue;
		}

		
		int d = src[index];

		int count = 0;
		for ( int ly = -3; ly <= 3; ly++ ) {
		for ( int lx = -3; lx <= 3; lx++ ) {
			if ( abs( d - src[index + lx + ly * src.Width()] ) < 30 ) {
				count++;
			}
		}}

		if ( count < 20 ) {
			src[index] = 0;
		}
	}}
}

void Grow(Image< unsigned short > &depth, Image< int > &flags, EPH_List<int> &counts, EPH_List<EPH_Vec2i> &queue, EPH_Vec2i src_p, int src_d, int ep_d ) {
	int begin = 0, end = 0;

	int src_id = counts.Append(0);

	flags[src_p] = src_id;
	++counts[src_id];
	queue[end++] = src_p;

	while( begin < end ) {
		EPH_Vec2i p = queue[begin++];
		int src_d = depth[p];

		int num = 0;
		EPH_Vec2i nei[4];
		
		if ( p.x > 0 ) {
			nei[num++] = p - EPH_Vec2i( 1, 0 );
		}
		if ( p.x < depth.Width() - 1 ) {
			nei[num++] = p + EPH_Vec2i( 1, 0 );
		}
		if ( p.y > 0 ) {
			nei[num++] = p - EPH_Vec2i( 0, 1 );
		}
		if ( p.y < depth.Height() - 1 ) {
			nei[num++] = p + EPH_Vec2i( 0, 1 );
		}

		for ( int i = 0; i < num; i++ ) {
			EPH_Vec2i q = nei[i];

			int id = flags(q);
			int d = depth(q);

			if ( !id && d > 0 && abs( d - src_d ) <= ep_d ) {
				flags[q] = src_id;
				++counts[src_id];
				queue[end++] = q;
			}
		}
	}
}

void RegionGrow( Image< unsigned short > &depth, int ep_d, int ep_c ) {
	EPH_List<int> counts(1);
	counts.EnsureSize(depth.NumPixels()+1);

	Image<int> flags( depth.Size() );
	flags.ZeroBuffer();

	EPH_List<EPH_Vec2i> queue(depth.NumPixels());

	for ( int y = 0; y < depth.Height(); y++ ) {
	for ( int x = 0; x < depth.Width() ; x++ ) {
		EPH_Vec2i p(x, y);

		if ( flags(p) ) {
			continue;
		}

		unsigned short d = depth(p);
		if ( d <= 0 ) {
			continue;
		}

		Grow( depth, flags, counts, queue, p, d, ep_d );
	}}

	for ( int y = 0; y < depth.Height(); y++ ) {
	for ( int x = 0; x < depth.Width() ; x++ ) {
		EPH_Vec2i p(x, y);
		
		int flag = flags(p);
		
		if ( flag && counts[flag] < ep_c ) {
			depth(p) = 0;
		}
	}}
}

void ProcessDepthMap(const char *path_format, int num, float dev_threshold, const EPH_Vec4 &intr, float distort[8], int numEros = 1 ) {
	struct float16 {
		float data[16];
	};

	// read depth maps
	EPH_List< Image< unsigned short > > depthMap;
	depthMap.EnsureSize(num);

	Image< unsigned short > depth2(EPH_Vec2i(512, 424));

	for (int k = 0; k < num; k++) {
		EPH_LoadBitFile file;
		if (!file.Open(EPH_String().SetWithFormat(path_format, k))) {
			printf("faild to open file %s\n", EPH_String().SetWithFormat(path_format, k).Ptr());
			break;
		}

		depthMap.Append();
		depthMap[k].Create(EPH_Vec2i(512, 424));
		file.ReadArray(depthMap[k].Data(), depthMap[k].NumPixels());

		for ( int i = 0; i < 1; i++ ) {
			Erosion(depthMap[k], depth2);
			Erosion(depth2, depthMap[k]);
		}

		DepthConsistency(depthMap[k]);
	}

	num = depthMap.Num();
	if (num <= 0) {
		return;
	}

	// mean & var
	Image< unsigned short > depth(EPH_Vec2i(512, 424));
	depth.ZeroBuffer();

	int flag = 0;

	int m = 0;
	EPH_List< float > pixelDepth(num);
	for (int i = 0; i < 512 * 424; i++) {
		// read pixel depth array and validate
		pixelDepth.ZeroBuffer();
		int count = 0;
		for (int k = 0; k < num; k++) {
			unsigned short v = depthMap[k][i];
			if (v < 20 || v > 6000) {
				continue;
			}
			pixelDepth[count++] = v;
		}

		m = EPH_Math::Max( m, count );
		if (count <= 20) {
		//	continue;
		}

		// mean
		float mean = 0;
		for (int k = 0; k < count; k++) {
			mean += pixelDepth[k];
		}
		mean /= count;

#if 0
		// deviation
		float dev = 0;
		for (int k = 0; k < count; k++) {
			float r = pixelDepth[k] - mean;
			dev += r * r;
		}
		dev = sqrtf(dev / count);

		if (dev > dev_threshold) {
			continue;
		}

		// mean cut-off
		float mean_trim = 0;
		int count_trim = 0;
		for (int k = 0; k < count; k++) {
			if (fabs(pixelDepth[k] - mean) <= dev * 2.0f ) {
				mean_trim += pixelDepth[k];
				count_trim++;
			}
		}
		if (count_trim <= 0) {
			continue;
		}
		mean_trim /= (float)count_trim;

		depth[i] = EPH_Math::Clamp(EPH_Math::RoundHalfUp(mean_trim), (int)EPH_Type::Integer<unsigned short>::min, (int)EPH_Type::Integer<unsigned short>::max);
#else
		depth[i] = EPH_Math::Clamp(EPH_Math::RoundHalfUp(mean), (int)EPH_Type::Integer<unsigned short>::min, (int)EPH_Type::Integer<unsigned short>::max);
#endif

	}
	//printf( "MAX : %d\n", m );

//	UndistortImage(depth, depth2, intr, distort);

	RegionGrow( depth, 100, 500 );

	// save
	SaveDepthMap(path_format, -1, depth);
}
/*
void ProcessDepthMap2(const char *path_format, int num, float dev_threshold ) {
	struct float16 {
		float data[16];
	};

	// read depth maps
	EPH_List< EPH_List< unsigned short > > depthMap;
	depthMap.EnsureSize(num);

	for (int k = 0; k < num; k++) {
		EPH_LoadBitFile file;
		if (!file.Open(EPH_String().SetWithFormat(path_format, k))) {
			printf("faild to open file %s\n", EPH_String().SetWithFormat(path_format, k).Ptr());
			break;
		}

		depthMap.Append();
		depthMap[k].SetNum(512 * 424);
		file.ReadArray(depthMap[k].Ptr(), depthMap[k].Num());
	}

	num = depthMap.Num();
	if (num <= 0) {
		return;
	}

	// mean & var
	EPH_List< unsigned short > depth(512 * 424);
	depth.ZeroBuffer();

	int flag = 0;

	EPH_List< float > pixelDepth(num);
	for (int i = 0; i < 512 * 424; i++) {
		// read pixel depth array and validate
		pixelDepth.ZeroBuffer();
		int count = 0;
		for (int k = 0; k < num; k++) {
			unsigned short v = depthMap[k][i];
			if (v < 50 || v > 6000) {
				continue;
			}
			pixelDepth[count++] = v;
		}

		if (count <= 0) {
			continue;
		}

		// mean
		float mean = 0;
		for (int k = 0; k < count; k++) {
			mean += pixelDepth[k];
		}
		mean /= count;

#if 1
		// deviation
		float dev = 0;
		for (int k = 0; k < count; k++) {
			float r = pixelDepth[k] - mean;
			dev += r * r;
		}
		dev = sqrtf(dev / count);

		if (dev > dev_threshold) {
			continue;
		}

		// mean cut-off
		float mean_trim = 0;
		int count_trim = 0;
		for (int k = 0; k < count; k++) {
			if (fabs(pixelDepth[k] - mean) <= dev * 3.0f) {
				mean_trim += pixelDepth[k];
				count_trim++;
			}
		}
		if (count_trim <= 0) {
			continue;
		}
		mean_trim /= (float)count_trim;

		depth[i] = EPH_Math::Clamp(EPH_Math::RoundHalfUp(mean_trim), (int)EPH_Type::Integer<unsigned short>::min, (int)EPH_Type::Integer<unsigned short>::max);
#else
		depth[i] = EPH_Math::Clamp(EPH_Math::RoundHalfUp(mean), (int)EPH_Type::Integer<unsigned short>::min, (int)EPH_Type::Integer<unsigned short>::max);
#endif

		
	}

	EPH_List< unsigned short > depth2(512 * 424);
	// dilation
	for (int i = 0; i < 2; i++) {
		//	Dilation( depth, depth2 );
		//	Dilation( depth2, depth );
	}

	{
		// erosion
		for (int i = 0; i < 1; i++) {
			Erosion(depth, depth2);
			Erosion(depth2, depth);
		}
	}

	// save
	SaveDepthMap(path_format, -1, depth);
}
*/