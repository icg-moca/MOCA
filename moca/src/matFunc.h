EPH_Mat3 Block( const EPH_Mat4 &M ) {
	EPH_Mat3 R;
	for ( int y = 0; y < 3; y++ ) {
		for ( int x = 0; x < 3; x++ ) {
			R[y][x] = M[y][x];
		}
	}
	
	return R;
}

EPH_Mat4 Inv( const EPH_Mat4 &M ) {
	return EPH_CreateMatrix::RotateTranslate( Block( M ).Transpose(), -M.Column(3).Sub<3>() );
}


EPH_Vec2 IntrTransform( const EPH_Vec4 &K, const EPH_Vec2 &v ) {
	return v * K.Sub<2>() + K.Sub<2>( 2 );
}

EPH_Vec2 IntrTransform( const EPH_Vec4 &K, const EPH_Vec3 &v ) {
	return v.Sub<2>() / v.z * K.Sub<2>() + K.Sub<2>( 2 );
}

EPH_Vec2 IntrInvTransform( const EPH_Vec4 &K, const EPH_Vec2 &v ) {
	return ( v - K.Sub<2>( 2 ) ) / K.Sub<2>();
}

EPH_Vec3 IntrInvTransform( const EPH_Vec4 &K, const EPH_Vec2 &v, float d ) {
	return ( ( v - K.Sub<2>( 2 ) ) / K.Sub<2>() ).Cat( 1 ) * d;
}

EPH_Vec2 ReprojectRay( const EPH_Vec2 &src_v, const EPH_Vec4 &K_src, const EPH_Vec4 &K_dst, const EPH_Mat3 &M_src_to_dst ){
	float px = ( src_v.x - K_src.z ) / K_src.x;
	float py = ( src_v.y - K_src.w ) / K_src.y;
	
	EPH_Vec3 q = M_src_to_dst * EPH_Vec3( px, py, 1 );
	
	float qx = ( q.x / q.z * K_dst.x + K_dst.z );
	float qy = ( q.y / q.z * K_dst.y + K_dst.w );
	
	return EPH_Vec2( qx, qy );
}

void AdjustRectifyRect( const EPH_Vec2i &size, const EPH_Vec4 &K, const EPH_Mat3 &M, EPH_Vec2 &bmin, EPH_Vec2 &bmax ) {
	EPH_Vec2 corner[4];
	corner[0] = ReprojectRay( EPH_Vec2( 0,		0		), K, EPH_Vec4( 1, 1, 0, 0 ), M );
	corner[1] = ReprojectRay( EPH_Vec2( size.x, 0		), K, EPH_Vec4( 1, 1, 0, 0 ), M );
	corner[2] = ReprojectRay( EPH_Vec2( size.x, size.y	), K, EPH_Vec4( 1, 1, 0, 0 ), M );
	corner[3] = ReprojectRay( EPH_Vec2( 0,		size.y	), K, EPH_Vec4( 1, 1, 0, 0 ), M );
	bmin = corner[0].Min( corner[1] ).Min( corner[2].Min( corner[3] ) );
	bmax = corner[0].Max( corner[1] ).Max( corner[2].Max( corner[3] ) );
}
/*
void Rectify(
	const EPH_Vec2i &size,
	const EPH_Vec4 &K_L, const EPH_Mat4 &M_L,
	const EPH_Vec4 &K_R, const EPH_Mat4 &M_R,

	const EPH_Vec2i &size_new,
	EPH_Vec4 &K_L_new, EPH_Mat4 &M_L_new,
	EPH_Vec4 &K_R_new, EPH_Mat4 &M_R_new,

	EPH_Vec2 &depth_to_disparity, // x - ( Bf / d + ( c1 - c2 ) )
	const float ratio
) {
	// rectify
	const EPH_Vec3 o0 = M_L.Column(3).Sub<3>();
	const EPH_Vec3 o1 = M_R.Column(3).Sub<3>();

	float baseline = ( o0 - o1 ).Length();

	EPH_Vec3 right = ( o1 - o0 ).Normalize();
	EPH_Vec3 up = EPH_Vec3( 0, -1, 0 );

	EPH_Mat3 rot( right, up, right.Cross( up ) );
	rot.Normalize(); //rot[2] = -rot[2];
	rot = rot.Transpose();

	M_L_new = rot.CatCol( o0 ).CatRow( EPH_Vec4( 0, 0, 0, 1 ) );
	M_R_new = rot.CatCol( o1 ).CatRow( EPH_Vec4( 0, 0, 0, 1 ) );
	
	EPH_Vec2 bmin_L, bmax_L;
	AdjustRectifyRect( size, K_L, (rot.Transpose().To<double>() * Block( M_L ).To<double>()).To<float>(), bmin_L, bmax_L );
	printf( "L\n" );
	K_L.Print();
	bmin_L.Print();
	bmax_L.Print();

	EPH_Vec2 bmin_R, bmax_R;
	AdjustRectifyRect( size, K_R, (rot.Transpose().To<double>() * Block( M_R ).To<double>()).To<float>(), bmin_R, bmax_R );
	printf( "R\n" );
	K_R.Print();
	bmin_R.Print();
	bmax_R.Print();

	float ymin = EPH_Math::Max( bmin_L.y, bmin_R.y );
	float ymax = EPH_Math::Min( bmax_L.y, bmax_R.y );

	bmin_L.y = ymin;
	bmax_L.y = ymax;

	bmin_R.y = ymin;
	bmax_R.y = ymax;

	EPH_Vec2 f = size_new.y * ratio / ( ymax - ymin );

	EPH_Vec2 c_L = size_new.To<float>() * 0.5f  - ( bmin_L + bmax_L )  * 0.5f * f;
	EPH_Vec2 c_R = size_new.To<float>() * 0.5f  - ( bmin_R + bmax_R )  * 0.5f * f;

	K_L_new = f.Cat( c_L );
	K_R_new = f.Cat( c_R );

	depth_to_disparity.Set( baseline * f.x, K_L_new.z - K_R_new.z );

	printf( "rectfy\n" );
	K_L_new.Print();
	K_R_new.Print();
}*/