
class Sensor {
public :
	// parameters
	EPH_Vec4 intr;
	EPH_Mat4 extr, r_extr;
	float distort[8];

public :
	Sensor() {
		intr.Set(1, 1, 0, 0);
		extr.Identity();
		memset(distort, 0, sizeof(distort));
	}
	EPH_Vec2 Project( const EPH_Vec4 &v ) const {
		float x = v.x / v.z * intr[0] + intr[2];
		float y = v.y / v.z * intr[1] + intr[3];
		return EPH_Vec2( x, y );
	}

	EPH_Vec3 Unproject( const EPH_Vec2 &v, float d ) const {
		float x = ( v.x - intr[2] ) / intr[0];
		float y = ( v.y - intr[3] ) / intr[1];
		return EPH_Vec3( x, y, 1 ) * d;
	}
};

EPH_Mat4 inv_rigid_transform(const EPH_Mat4 &M) {
	return EPH_CreateMatrix::RotateTranslate( M.Block<3, 3>( 0, 0 ).Transpose(), -M.Column(3).Sub<3>() );
}

void LoadCamera( const char *path, EPH_List<Sensor> &sensors ) {
	EPH_LoadBitFile file;
	file.Open( path );
	
	sensors.SetNum( file.Size() / ( sizeof( double ) * ( 4 + 16 ) ) );

	file.SetOffset( 0 );

	for ( int k = 0; k < sensors.Num(); k++ ) {
		for ( int i = 0; i < 4; i++ ) {
			double v;
			file.Read(v);

			sensors[k].intr.Ptr()[i] = v;

		}

#if 0
		for (int i = 0; i < 5; i++) {
			double v;
			file.Read(v);

			sensors[k].distort[i] = v;

		}
#endif

		for ( int i = 0; i < 16; i++ ) {
			double v;
			file.Read(v);

			sensors[k].extr.Ptr()[i] = v;
		}

		sensors[k].r_extr = inv_rigid_transform( sensors[k].extr );

		printf( "================ %d\n", k );
		sensors[k].intr.Print();
		sensors[k].extr[0].Print();
		sensors[k].extr[1].Print();
		sensors[k].extr[2].Print();
		sensors[k].extr[3].Print();
	}
}

void SaveCamera(const char *path, EPH_List<Sensor> &sensors) {
	EPH_SaveBitFile file;
	file.Open( path );
	
	printf( "save camera : %s\n", path );

	for ( int k = 0; k < sensors.Num(); k++ ) {

		for ( int i = 0; i < 4; i++ ) {
			double v = sensors[k].intr.Ptr()[i];
			file.Write(v);
		}

		for (int i = 0; i < 5; i++) {
			double v = sensors[k].distort[i];
			file.Write(v);
		}

		for ( int i = 0; i < 16; i++ ) {
			double v = sensors[k].extr.Ptr()[i];
			file.Write(v);
		}
	}
}

class SensorMap {
public :
	EPH_Vec2i size;

	// host
	Image< unsigned short > depth;
	Image< EPH_Vec4 > points;
	Image< EPH_Vec4 > normals;
	Image< EPH_Vec4 > colors;

	EPH_List< double > timeStamps;

	bool isValid, isVisible;

	// gpu
	CL::Mem mem_d, mem_p, mem_n, mem_c, mem_dist;

	SensorMap() :
		isValid(false),
		isVisible(true)
	{
	}

	void Create( const EPH_Vec2i &newSize ) {
		size = newSize;

		depth  .Create( size );
		points .Create( size );
		normals.Create( size );
		colors .Create( size );

		isValid = false;
	}

	void CreateBuffer(CL::Context &context) {
		int total = 512 * 424;

		mem_d.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_ushort));
		mem_p.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float4));
		mem_n.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float4));
		mem_c.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float4));
		mem_dist.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float));
	}

	int SearchFrame( double time, double &dt ) const {
		if ( timeStamps.IsEmpty() ) {
			return -1;
		}
		
		dt = 0;
		int frame = BinarySearch( timeStamps.Num(), timeStamps.Ptr(), time );
		if ( frame < 0 ) {
			return -1;
		}

		dt = ( time - timeStamps[frame] );

		double dt2 = dt, dt3 = dt;
		int frame2 = frame, frame3 = frame;
		if ( frame < timeStamps.Num() - 1 ) {
			dt2 = ( time - timeStamps[frame + 1] );
			frame2 = frame + 1;
		}
		if ( frame > 0 ) {
			dt3 = ( time - timeStamps[frame - 1] );
			frame3 = frame - 1;
		}
		
		if ( fabs( dt ) > fabs( dt2 ) ) {
			dt = dt2;
			frame = frame2;
		}
		if ( fabs( dt ) > fabs( dt3 ) ) {
			dt = dt3;
			frame = frame3;
		}

		return frame;
	}
};