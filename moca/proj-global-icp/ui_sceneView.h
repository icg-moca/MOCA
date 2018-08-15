#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <iostream>
#include <fstream>
#include <vector>

#include "depthMap.h"
#include "ply.h"

class Sensor {
public :
	// parameters
	EPH_Vec4 intr;
	EPH_Mat4 extr;
};

class SensorMap {
public :
	EPH_Vec2i size;

	// host
	EPH_List< unsigned short > depth;
	EPH_List< EPH_Vec4 > points;
	EPH_List< EPH_Vec4 > normals;

	// gpu
	CL::Mem mem_d, mem_p, mem_n;

	void Create( const EPH_Vec2i &newSize ) {
		size = newSize;

		int total = size.Area();
		depth.SetNum(total);
		points.SetNum(total);
		normals.SetNum(total);
	}
};

// R(x)
Eigen::Matrix3f rot_x_mat( float rad ) {
	float c = cos( rad );
	float s = sin( rad );

	Eigen::Matrix3f m;
	m <<
		1, 0, 0,
		0, c, -s,
		0, s, c
		;

	return m;
}

// R(y)
Eigen::Matrix3f rot_y_mat( float rad ) {
	float c = cos( rad );
	float s = sin( rad );

	Eigen::Matrix3f m;
	m <<
		c, 0, s,
		0, 1, 0,
		-s, 0, c
		;

	return m;
}

// R(z)
Eigen::Matrix3f rot_z_mat( float rad ) {
	float c = cos( rad );
	float s = sin( rad );

	Eigen::Matrix3f m;
	m <<
		c, -s, 0,
		s, c, 0,
		0, 0, 1
		;

	return m;
}

// R(z) * R(y) * R(x)
Eigen::Matrix3f euler_to_mat( const Eigen::Vector3f &euler ) {
	return rot_z_mat( euler.z() ) * rot_y_mat( euler.y() ) * rot_x_mat( euler.x() );
}

Eigen::Matrix3f euler_to_skew( const Eigen::Vector3f &euler ) {
	float x = euler.x();
	float y = euler.y();
	float z = euler.z();

	Eigen::Matrix3f r;
	r <<
		1.0f, -z, +y,
		+z, 1.0f, -x,
		-y, +x, 1.0f;

	return r;
}

EPH_Mat4 Pack( const Eigen::VectorXf &x, int k ) {
	k *= 6;

	Eigen::Vector3f euler;
	euler << x[k + 0], x[k + 1], x[k + 2];
	Eigen::Matrix3f r = euler_to_mat( euler );

	// translation
	Eigen::Vector3f t;
	t << x[k + 3], x[k + 4], x[k + 5];
		
	EPH_Mat4 T;
	T[0][0] = r(0, 0); T[1][0] = r(1, 0); T[2][0] = r(2, 0); T[3][0] = 0;
	T[0][1] = r(0, 1); T[1][1] = r(1, 1); T[2][1] = r(2, 1); T[3][1] = 0;
	T[0][2] = r(0, 2); T[1][2] = r(1, 2); T[2][2] = r(2, 2); T[3][2] = 0;
	T[0][3] = t(0)   ; T[1][3] = t(1)   ; T[2][3] = t(2);	 T[3][3] = 1;
	return T;
}

EPH_Mat3 Block(const EPH_Mat4 &M) {
	EPH_Mat3 R;
	for (int y = 0; y < 3; y++) {
		for (int x = 0; x < 3; x++) {
			R[y][x] = M[y][x];
		}
	}
	return R;
}

EPH_Mat4 Inv(const EPH_Mat4 &M) {
	return EPH_CreateMatrix::RotateTranslate( Block( M ).Transpose(), -M.Column(3).Sub<3>() );
}


//================
// UI_SceneView
//================
class UI_SceneView: public UI_3DView {
	EPH_UI_CLASS_PROTOTYPE( UI_SceneView )
	
public :
	EPH_String m_folder;

	CL::Context context;

	EPH_List<Sensor> sensors;
	EPH_List<Sensor> sensors_rgb;
	EPH_List<SensorMap> sensorMaps;

		
	// linear euqation solver
	EPH_List<cl_float8> buffer_j0, buffer_j1;
	CL::Mem mem_j0, mem_j1, mem_A;

	CL::Program program;

	CL::Kernel kn_cor;
	CL::Kernel kn_icp;
	CL::Kernel kn_d2p;
	CL::Kernel kn_p2n;
	CL::Kernel kn_jacobian, kn_reduction_j, kn_reduction_E;
	
	CL::CommandQueue cq;

	void InitCL( void ) {
		// platform
		EPH_List< CL::Platform > platforms;
		CL::System::EnumPlatforms( platforms );
		if ( platforms.IsEmpty() ) {
			return;
		}
		
		// context
		CL::System::CreateContext( platforms[1], context );

		// queue
		cq.Create( context );


		// program
		CompileProgram();
	}

	void CompileProgram(void) {
		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		EPH_TextFile source;
		source.Load( EPH_FileSystem::CurrentPath() + "/shader/icp.txt" );
		program.Create( context, source.Text(), flags );

		// kernel
		kn_d2p.Create( program, "depth_to_point" );
		kn_p2n.Create( program, "point_to_normal" );
		kn_jacobian.Create(program, "jacobian");
		kn_reduction_j.Create(program, "reduction_j");
		kn_reduction_E.Create(program, "reduction_E");
	}

	void BackProjection( Sensor &sensor, SensorMap &sensorMap ) {
		cq.WriteBuffer( sensorMap.mem_d, CL_TRUE, sensorMap.depth );
		
		cq.NDRangeKernel<2>((kn_d2p << EPH_Vec2i(512, 424), sensorMap.mem_d, sensorMap.mem_p, sensor.intr), 512, 16);
		cq.NDRangeKernel<2>((kn_p2n << EPH_Vec2i(512, 424), sensorMap.mem_p, sensorMap.mem_n), 512, 16);

		cq.ReadBuffer(sensorMap.mem_p, CL_TRUE, sensorMap.points);
		cq.ReadBuffer(sensorMap.mem_n, CL_TRUE, sensorMap.normals);
	}


	void Eval(Eigen::MatrixXf &A, Eigen::VectorXf &b, float &E, float &N, int k0, int k1, int fix[]) {
		if (sensorMaps[k0].points.IsEmpty() || sensorMaps[k1].points.IsEmpty()) {
			return;
		}

		cq.NDRangeKernel<1>(
			(kn_jacobian <<
			mem_j0, mem_j1,
			sensorMaps[k0].mem_p, sensorMaps[k0].mem_n,
			sensorMaps[k1].mem_p, sensorMaps[k1].mem_n,
			sensors[k0].extr.Transpose(), sensors[k1].extr.Transpose(), Inv( sensors[k1].extr ).Transpose(), sensors[k1].intr,
			512 * 424
			),
			512 * 424, 256
			);

		Eigen::MatrixXf J00(6, 6), J11(6, 6), J01(6, 6);
		Eigen::VectorXf J0r(6), J1r(6);

		J00 = Eigen::MatrixXf::Zero(6, 6);
		J11 = Eigen::MatrixXf::Zero(6, 6);
		J01 = Eigen::MatrixXf::Zero(6, 6);
		J0r = Eigen::VectorXf::Zero(6);
		J1r = Eigen::VectorXf::Zero(6);

#if 1
		if (!fix[k0]) {
			for (int i = 0; i < 6; i++) {
				cq.NDRangeKernel<1>((kn_reduction_j << mem_j0, mem_j0, i, mem_A, 512 * 424), 256, 256);

				cl_float8 J;
				cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

				for (int k = 0; k < 6; k++) { J00(i, k) = J.s[k]; }
			}
		}

		if (!fix[k1]) {
			for (int i = 0; i < 6; i++) {
				cq.NDRangeKernel<1>((kn_reduction_j << mem_j1, mem_j1, i, mem_A, 512 * 424), 256, 256);

				cl_float8 J;
				cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

				for (int k = 0; k < 6; k++) { J11(i, k) = J.s[k]; }
			}
		}

		if (!fix[k0] && !fix[k1]) {
			for (int i = 0; i < 6; i++) {
				cq.NDRangeKernel<1>((kn_reduction_j << mem_j0, mem_j1, i, mem_A, 512 * 424), 256, 256);

				cl_float8 J;
				cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

				for (int k = 0; k < 6; k++) { J01(i, k) = J.s[k]; }
			}
		}

		if (!fix[k0]) {
			{
				cq.NDRangeKernel<1>((kn_reduction_j << mem_j1, mem_j0, 6, mem_A, 512 * 424), 256, 256);

				cl_float8 J;
				cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

				for (int k = 0; k < 6; k++) { J0r(k) = J.s[k]; }
			}
		}

		if (!fix[k1]) {
			{
				cq.NDRangeKernel<1>((kn_reduction_j << mem_j0, mem_j1, 6, mem_A, 512 * 424), 256, 256);

				cl_float8 J;
				cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

				for (int k = 0; k < 6; k++) { J1r(k) = J.s[k]; }
			}
		}

		{
			cq.NDRangeKernel<1>((kn_reduction_E << mem_j0, mem_A, 512 * 424), 256, 256);

			cl_float8 J;
			cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

			E += J.s0;
			N += J.s1;
		}


#else
		buffer_j0.SetNum(512 * 424);
		buffer_j1.SetNum(512 * 424);

		cq.ReadBuffer(mem_j0, CL_TRUE, buffer_j0);
		cq.ReadBuffer(mem_j1, CL_TRUE, buffer_j1);

		for (int i = 0; i < 512 * 424; i++) {
			cl_float8 j0 = buffer_j0[i];
			cl_float8 j1 = buffer_j1[i];

			if (j0.s7 <= 0.0f) {
				continue;
			}

			float r = j0.s6;
			float w = 1.0f;

			Eigen::VectorXf J0(6), J1(6);
			J0 << j0.s0, j0.s1, j0.s2, j0.s3, j0.s4, j0.s5;
			J1 << j1.s0, j1.s1, j1.s2, j1.s3, j1.s4, j1.s5;

			J00 += J0 * J0.transpose() * w;
			J11 += J1 * J1.transpose() * w;
			J01 += J0 * J1.transpose() * w;

			J0r += J0 * r * w;
			J1r += J1 * r * w;

			E += r * r;
			N += 1;
		}


#endif
		if (!fix[k0]) A.block(6 * k0, 6 * k0, 6, 6) += J00;
		if (!fix[k0]) A.block(6 * k1, 6 * k1, 6, 6) += J11;
		if (!fix[k0] && !fix[k1]) A.block(6 * k0, 6 * k1, 6, 6) += J01;
		if (!fix[k0] && !fix[k1])	A.block(6 * k1, 6 * k0, 6, 6) += J01.transpose();

		if (!fix[k0]) b.block(6 * k0, 0, 6, 1) += J0r;
		if (!fix[k1]) b.block(6 * k1, 0, 6, 1) += J1r;
	
#if 0
		std::cout << "J00\n" << J00 << std::endl;
		std::cout << "J01\n" << J01 << std::endl;
		std::cout << "J11\n" << J11 << std::endl;
		std::cout << "J0r\n" << J0r << std::endl;
		std::cout << "J1r\n" << J1r << std::endl;
#endif
	}



	
	void ICP( int numIters, int id, int fix[], int origin ) {
		for ( int k = 0; k < numIters; k++ ) {
			Eigen::MatrixXf A = Eigen::MatrixXf::Identity( 6 * sensors.Num(), 6 * sensors.Num() ) * 0.001f; // FIXME
			Eigen::VectorXf b = Eigen::VectorXf::Zero( 6 * sensors.Num() );
			Eigen::VectorXf x = Eigen::VectorXf::Zero( 6 * sensors.Num() );
			float E = 0;
			float N = 0;

			for ( int i = 0; i < sensors.Num(); i++ ) {
				for ( int j = 0; j < sensors.Num(); j++ ) {
					if ( i == j ) {
						continue;
					}
					Eval(A, b, E, N, i, j, fix);
				}
			}

			x = A.fullPivHouseholderQr().solve( b );

			std::cout << "E : " << N << " : " << sqrt(E/(float)(N)) * 1000.0f << std::endl;

			EPH_Mat4 inv = Inv( Pack( x, origin ) );
			for ( int i = 0; i < sensors.Num(); i++ ) {
				if ( i != origin ) {
					sensors[i].extr = inv * Pack( x, i ) * sensors[i].extr;
				}
			}
		}
	}

public :
	void GenPoints( void ) {
		for ( int i = 0; i < sensors.Num(); i++ ) {
			sensorMaps[i].Create( EPH_Vec2i( 512, 424 ) );			
			if ( !LoadDepthMap( EPH_String().SetWithFormat( m_folder + "/K%d/Pose_999.png", i ), sensorMaps[i].depth ) ) {
				continue;
			}
			BackProjection( sensors[i], sensorMaps[i] );
			
		}
	}

	void LoadCamera( const char *path, EPH_List<Sensor> &sensors ) {
		EPH_LoadBitFile file;
		file.Open( path );

		sensors.SetNum( 8 );

		for ( int k = 0; k < sensors.Num(); k++ ) {
			for ( int i = 0; i < 4; i++ ) {
				double v;
				file.Read(v);

				sensors[k].intr.Ptr()[i] = v;

			}

			for ( int i = 0; i < 16; i++ ) {
				double v;
				file.Read(v);

				sensors[k].extr.Ptr()[i] = v;
			}

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

			for ( int i = 0; i < 16; i++ ) {
				double v = sensors[k].extr.Ptr()[i];
				file.Write(v);
			}
		}
	}

	virtual void PrepareGL() {
		// folder
		{
			// filter
			const char *filter =
				"All Files (*.*)\0"					"*.*\0"
				"\0";

			EPH_String path;
			if (!((UI_Window *)(m_window))->m_osWindow->OpenFileDialog(m_folder, path, filter)) {
				return;
			}
		}
		
		LoadCamera(m_folder + "/sensor-ir-x.mat", sensors);
		LoadCamera(m_folder + "/sensor-rgb-x.mat", sensors_rgb);

		for (int i = 0; i < sensors.Num(); i++) {
			ProcessDepthMap(m_folder + "/K" + i + "/Pose_%d.png");
		}

		InitCL();
		
		// mem
		int total = 512 * 424;

		sensorMaps.SetNum( sensors.Num() );
		for (int i = 0; i < sensorMaps.Num(); i++) {
			sensorMaps[i].mem_d.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_ushort));
			sensorMaps[i].mem_p.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float4));
			sensorMaps[i].mem_n.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float4));
		}

		mem_j0.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float8));
		mem_j1.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float8));
		
		mem_A .CreateBuffer(context, CL_MEM_READ_WRITE, sizeof(cl_float8));

		GenPoints();
	}

public :
	void DrawPoints( Sensor &sensor, SensorMap &sensorMap, EPH_RGBA8 color ) {
		glPointSize( 1 );
		glColor3ubv( color.Ptr() );
		
		glPushMatrix();
		glMultMatrixf( sensor.extr.Transpose().Ptr() );
		glVertexPointer(3, GL_FLOAT, sizeof(EPH_Vec4), sensorMap.points.Ptr() );
		glDrawArrays(GL_POINTS, 0, 512 * 424);
		glPopMatrix();
	}

	EPH_Bound m_bound;
	
	virtual void Render( void ) {
		m_camera.AdjustDepth( EPH_Bound( EPH_Vec3( -1000, 0, -1000 ), EPH_Vec3( 1000, 1000, 1000 ) ) );
		m_camera.SetDepth( 0.001f, 100000.0f );
		glViewport( m_rect.x0, m_rect.y0, m_rect.Width(), m_rect.Height() );

		glMatrixMode( GL_PROJECTION );
		glLoadMatrixf( m_camera.view.ProjMat().Transpose().Ptr() );

		glMatrixMode( GL_MODELVIEW );
		glLoadMatrixf( m_camera.view.ViewMat().Transpose().Ptr() );

		EPH_AuxPrimitive aux;

		aux.Color3ub( 90, 100, 120 );
		aux.GridZX( 100, 1 );
		aux.Draw();

		aux.AxisDefault( 20 );
		aux.Draw();

		m_bound.Set( (0 - EPH_Vec3( 0.5, -0.02, -0.2 )), (1.80f - EPH_Vec3( 0.5, -0.02, -0.2 )) );
		
		// draw frame box
		aux.FrameBox( m_bound[0], m_bound[1] );
		aux.Draw();

		// darw points
		glEnableClientState( GL_VERTEX_ARRAY );		
		{
			for ( int i = 0; i < sensors.Num(); i++ ) {
				DrawPoints(sensors[i], sensorMaps[i], *(EPH_RGBA8*)(&eph_mac_palette[ i * 10 + 20 ]) );
			}
		}
		glDisableClientState( GL_VERTEX_ARRAY );
	}

public :
	virtual bool KeyDown(const EPH_KeyState &keyState) {

		for ( int i = 0; i <= 9; i++ ) {
			
			if (keyState.IsKey(EPH_KEY_0 + i)) {
				EPH_List<int> fix( sensors.Num() );
				for (int k = 0; k < sensors.Num(); k++) {
					fix[k] = 0;
				}
			
				fix[i] = 1;

				ICP(1, 0, fix, i);
				return true;
			}
		}

		if (keyState.IsKey(EPH_KEY_Y)) {
			SaveCamera(m_folder + "/sensor-ir.mat", sensors);
			SaveCamera(m_folder + "/sensor-rgb.mat", sensors_rgb);
			return true;
		}

		if (keyState.IsKey(EPH_KEY_H)) {
			CompileProgram();
			return true;
		}

		if (keyState.IsKey(EPH_KEY_O)) {
			LoadCamera(m_folder + "/sensor-ir-x.mat", sensors);
			return true;
		}
		return UI_3DView::KeyDown( keyState );
	}

	// FIXME : not a good way
	virtual bool MouseDown( const EPH_MouseState &mouseState ) {
		if ( UI_3DView::MouseDown( mouseState ) ) {
			return true;
		}
		// FIXME
		if ( m_isPressed ) {
			return false;
		}

		return false;
	}


	// FIXME : any better way? notification?
	virtual void TimeTick( const EPH_TimeState &timeState ) {
		UI_3DView::TimeTick( timeState );
	}
};
