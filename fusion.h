//
//  fusion.h
//  Moc
//
//  Created by carmack on 5/14/15.
//  Copyright (c) 2015 EPH. All rights reserved.
//

#ifndef Moc_fusion_h
#define Moc_fusion_h

#include <cl.h>

class Fusion {
	public :
	cl::int2			image_size;		// depth map size
	cl::int3			volume_size;	// volume size - volume space
	cl::float3			bound[2];		// volume size - world space

	cl::size2			global_size2;
	cl::size2			local_size2;

	cl::size3			global_size3;
	cl::size3			local_size3;

	// program
	cl::Context			context;
	cl::CommandQueue	cq;
	cl::Program			program;

	// kernel
	cl::Kernel			kn_d2p; // depth map to point map
	cl::Kernel			kn_p2n; // point map to normal map
	cl::Kernel			kn_p2w; // point map -view space to world space

	cl::Kernel			kn_df_zero; // clear tsdf
	cl::Kernel			kn_df_sphr; // gen sphere tsdf - debug
	cl::Kernel			kn_df_fuse; // fuse depth map into tsdf

	cl::Kernel			kn_ray_cast; // ray cast tsdf

	// mem
	cl::Mem				mem_d; // depth map
	cl::Mem				mem_p; // point map - view space
	cl::Mem				mem_n; // normal map
	cl::Mem				mem_w; // point map - world space

	cl::Mem				mem_df; // tsdf volume - fuse

	cl::Mem				mem_c; // color frame - render

	public :
	void Create( void ) {
		// context
		cl::System::CreateContext( context );

		// queue
		cq.Create( context, CL_QUEUE_PROFILING_ENABLE );

		// mem
		mem_d.CreateBuffer( context, CL_MEM_READ_WRITE, image_size.area() * sizeof( cl_float  ) );
		mem_p.CreateBuffer( context, CL_MEM_READ_WRITE, image_size.area() * sizeof( cl_float4 ) );
		mem_n.CreateBuffer( context, CL_MEM_READ_WRITE, image_size.area() * sizeof( cl_float4 ) );
		mem_w.CreateBuffer( context, CL_MEM_READ_WRITE, image_size.area() * sizeof( cl_float4 ) );

		mem_df.CreateBuffer( context, CL_MEM_READ_WRITE, volume_size.volume() * sizeof( cl_float2 ) );

		mem_c.CreateBuffer( context, CL_MEM_READ_WRITE, ( 512 * 512 ) * sizeof( cl_float4 ) );
	}

	void UpdateShader( const char *shader ) {
		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		program.Create( context, shader, flags );

		// kernel
		kn_d2p.Create( program, "depth_to_point" );
		kn_p2n.Create( program, "point_to_normal" );
		kn_p2w.Create( program, "point_to_world" );

		kn_df_zero.Create( program, "df_clear" );
		kn_df_sphr.Create( program, "df_sphere" );
		kn_df_fuse.Create( program, "df_fuse" );

		kn_ray_cast.Create( program, "ray_cast" );
	}

	void Clear( void ) {
		// init
		cq.NDRangeKernel3( ( kn_df_zero << mem_df, volume_size, cl::float2( 0 ) ), global_size3, local_size3 );
	}

	// cali.cam_intr.cameraMatrix, kinect_mat * cali.cam_extr.inv()
	void FuseFrame( const cv::Mat &intr, const cv::Mat &extr, const float *depth ) {
		// K
		vector<float> K;
		MatToVec( intr, K );

		// M
		double rep[] = {
			1, 0, 0, 0,
			0, 0, 1, 0,
			0, -1, 0, 0,
			0, 0, 0, 1 };
		vector<float> M;
		MatToVec( ( cv::Mat( 4, 4, CV_64F, rep ) * extr ).t(), M );

		// depth
		cq.WriteBuffer( mem_d, CL_TRUE, 0, image_size.area() * sizeof( cl_float ), depth );

		// fuse
		cq.NDRangeKernel2( ( kn_d2p << mem_d, mem_p, image_size, K	), global_size2, local_size2 );
		cq.NDRangeKernel2( ( kn_p2n << mem_p, mem_n, image_size		), global_size2, local_size2 );
		cq.NDRangeKernel2( ( kn_p2w << mem_p, mem_w, image_size, M	), global_size2, local_size2 );

		cl::float4 L = cl::float4( bound[0].x, bound[0].y, bound[0].z, ( bound[1].x - bound[0].x ) / volume_size.x );
		cq.NDRangeKernel3( ( kn_df_fuse << mem_df, volume_size, mem_d, mem_n, image_size, K, M, L ), global_size3, local_size3 );
		//cq.NDRangeKernel<3>( kn_df_sphr( mem_df, volume_size, 60.0f				), global_size3, local_size3 ); // FIMXE : param format
	}

	// cali.cam_intr.cameraMatrix, kinect_mat * cali.cam_extr.inv()
	void BackProjectPoints( const cv::Mat &intr, const cv::Mat &extr, const float *depth, vector< cl::float4 > &points, vector< cl::float4 > &normals ) {
		// K
		vector<float> K;
		MatToVec( intr, K );

		// M
		double rep[] = {
			1, 0, 0, 0,
			0, 0, 1, 0,
			0, -1, 0, 0,
			0, 0, 0, 1 };
		vector<float> M;
		MatToVec( ( cv::Mat( 4, 4, CV_64F, rep ) * extr ).t(), M );

		// depth
		cq.WriteBuffer( mem_d, CL_TRUE, 0, image_size.area() * sizeof( cl_float ), depth );

		// back projection
		cq.NDRangeKernel2( ( kn_d2p << mem_d, mem_p, image_size, K	), global_size2, local_size2 );
		cq.NDRangeKernel2( ( kn_p2w << mem_p, mem_w, image_size, M	), global_size2, local_size2 );
		cq.NDRangeKernel2( ( kn_p2n << mem_w, mem_n, image_size		), global_size2, local_size2 );

		points.resize( image_size.area() );
		cq.ReadBuffer( mem_w, CL_TRUE, points );

		normals.resize( image_size.area() );
		cq.ReadBuffer( mem_n, CL_TRUE, normals );
	}

	void RayCast( float fov, const cv::Mat &extr, vector< cl::float4 > &color ) {
		// L
		cl::float4 L = cl::float4( bound[0].x, bound[0].y, bound[0].z, ( bound[1].x - bound[0].x ) / volume_size.x );

		// K
		cl::float2 size( 512, 512 );
		size.x *= 0.5;
		size.y *= 0.5;

		float t = tan( fov * 0.5 * 3.14159265 / 180.0 );
		cl::float4 K( size.x, size.y, size.x / t, size.y / t );

		// M
		vector< float > M( 16 );
		MatToVec( extr.t(), M );

		cq.NDRangeKernel2( ( kn_ray_cast << mem_c, cl::int2( 512 ), mem_df, volume_size, K, M, L ), cl::size2( 512 ), cl::size2( 4 ) );

		color.resize( 512 * 512 );
		cq.ReadBuffer( mem_c, CL_TRUE, color );
	}
};

#endif
