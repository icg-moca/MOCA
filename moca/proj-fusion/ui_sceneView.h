#include <precompiled.h>
#include <compute/eph_cl.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <matFunc.h>
#include <imageProc.h>
#include <cameraFunc.h>

#include "search.h"
#include "cameraSensor.h"
#include "depthMap.h"
#include "distanceMap.h"

#include "ply.h"

#include <FreeImage.h>
bool LoadAnyImage( Image<EPH_RGBA8> &image, const EPH_String &path ) {
	FREE_IMAGE_FORMAT formato = FreeImage_GetFileType(path);//Automatocally detects the format(from over 20 formats!)
	FIBITMAP* imagen = FreeImage_Load(formato, path);
	if ( !imagen ) {
		printf( "FreeImage : failed to open image : %s\n", path.Ptr() );
		return false;
	}
 
	FIBITMAP* temp = imagen;
	imagen = FreeImage_ConvertTo32Bits(imagen);
	FreeImage_Unload(temp);
 
	int w = FreeImage_GetWidth(imagen);
	int h = FreeImage_GetHeight(imagen);
	char *pixels = (char*)FreeImage_GetBits(imagen);

	image.Create( w, h );
	memcpy( image.Data(), pixels, image.BufferSize() );

	FreeImage_Unload(imagen);
	return true;
}

//================
// UI_SceneView
//================
class UI_SceneView: public UI_3DView {
	EPH_UI_CLASS_PROTOTYPE( UI_SceneView )
	
public :
	// OpenCL
	CL::Context context;

	CL::Mem mem_src_d, mem_src_dist;
	CL::Mem mem_dst_d;
	CL::Mem mem_src_p;
	CL::Mem mem_dst_p;
	CL::Mem mem_src_n;
	CL::Mem mem_dst_n;

	CL::Mem mem_tsdf, mem_img_tsdf;
	CL::Mem mem_image, mem_image_d;

	CL::Mem mem_tile_task;

	CL::Program program;

	CL::Kernel kn_depth_to_points;
	CL::Kernel kn_points_to_normals;
	CL::Kernel kn_point_transform;
	CL::Kernel kn_clip_points;
	CL::Kernel kn_gen_tsdf, kn_fuse_tsdf, kn_vis_tsdf;
	CL::Kernel kn_ray_cast_tsdf, kn_ray_cast_tsdf_d;
	
	CL::CommandQueue cq;

	EPH_Bound m_bound;
	EPH_Vec3i m_volumeSize;
	EPH_Vec2i m_renderSize, m_renderSize_d;

	void InitCL( void ) {
		//m_bound.Set( (0 - EPH_Vec3( 0.7, 0.1, -0.6 )), (1.50f - EPH_Vec3( 0.7, 0.1, -0.6 )) );
		m_bound.Set( (0 - EPH_Vec3( 0.95, 0.1, -0.2 )), (2.00f - EPH_Vec3( 0.95, 0.1, -0.2 )) );

	//	EPH_Vec3 center = m_bound.Center();
//
//		m_bound.max = center;

	//	m_bound += EPH_Vec3( m_bound.Size().x, m_bound.Size().y, 0 );

		//m_bound.Set( (0 - EPH_Vec3( 1.01, -0.02, 0.73 )), (1.80f - EPH_Vec3( 1.01, -0.02, 0.73 )) );
		m_volumeSize = 512;
		m_renderSize.Set( 1024 );
		m_renderSize_d.Set( 1920, 1080 );

		// platform
		EPH_List< CL::Platform > platforms;
		CL::System::EnumPlatforms( platforms );
		if ( platforms.IsEmpty() ) {
			return;
		}
		
		// context
		CL::System::CreateContext( platforms[0], context );

		// queue
		cq.Create( context );

		// program
		CompileProgram();

		int total = 512 * 424;

		// mem
		mem_src_d.CreateBuffer( context, CL_MEM_READ_WRITE, total * sizeof( cl_ushort ) );
		mem_src_dist.CreateBuffer( context, CL_MEM_READ_WRITE, total * sizeof( cl_float ) );
		mem_src_p.CreateBuffer( context, CL_MEM_READ_WRITE, total * sizeof( cl_float4 ) );
		mem_src_n.CreateBuffer( context, CL_MEM_READ_WRITE, total * sizeof( cl_float4 ) );
		
		mem_dst_d.CreateBuffer( context, CL_MEM_READ_WRITE, total * sizeof( cl_ushort ) );
		mem_dst_p.CreateBuffer( context, CL_MEM_READ_WRITE, total * sizeof( cl_float4 ) );
		mem_dst_n.CreateBuffer( context, CL_MEM_READ_WRITE, total * sizeof( cl_float4 ) );

		mem_tsdf.CreateBuffer( context, CL_MEM_READ_WRITE, m_volumeSize.Volume() * sizeof( cl_float2 ) );
		mem_image.CreateBuffer( context, CL_MEM_READ_WRITE, m_renderSize.Area() * sizeof( cl_float4 ) );
		mem_image_d.CreateBuffer( context, CL_MEM_READ_WRITE, m_renderSize_d.Area() * sizeof( cl_float ) );

		mem_tile_task.CreateBuffer( context, CL_MEM_READ_WRITE, (m_volumeSize/8).Volume() * sizeof( cl_int3 ) );

		cl_image_format format = { CL_RG, CL_FLOAT };

		cl_image_desc desc = { CL_MEM_OBJECT_IMAGE3D, m_volumeSize.x, m_volumeSize.y, m_volumeSize.z, 0, 0, 0, 0, 0 };

		printf( "creatImage : %d\n", mem_img_tsdf.CreateImage( context, CL_MEM_READ_ONLY, format, desc ) );

		// clear tsdf
		cq.NDRangeKernel<3>( (kn_gen_tsdf << m_volumeSize.Cat(0), mem_tsdf), m_volumeSize.To<size_t>(), 8 );
	}

	void CompileProgram( void ) {
		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		EPH_TextFile source;
		source.Load( EPH_FileSystem::CurrentPath() + "/icp.txt" );
		program.Create( context, source.Text(), flags );

		// kernel
		kn_point_transform.Create( program, "point_transform" );
		kn_depth_to_points.Create( program, "depth_to_point" );
		kn_points_to_normals.Create( program, "point_to_normal" );
		kn_clip_points.Create( program, "clip_points" );
		kn_gen_tsdf.Create( program, "gen_tsdf" );
		kn_fuse_tsdf.Create( program, "fuse_tsdf" );
		kn_vis_tsdf.Create( program, "vis_tsdf" );
		
		kn_ray_cast_tsdf.Create( program, "ray_cast_tsdf" );
		
		kn_ray_cast_tsdf_d.Create( program, "ray_cast_tsdf_d" );
	}

	void LoadShader() {
		cq.Finish();

		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		EPH_TextFile source;
		source.Load( EPH_FileSystem::CurrentPath() + "/icp.txt" );
		program.Create( context, source.Text(), flags );

		// kernel
		kn_point_transform.Create( program, "point_transform" );
		kn_depth_to_points.Create( program, "depth_to_point" );
		kn_points_to_normals.Create( program, "point_to_normal" );
		kn_clip_points.Create( program, "clip_points" );
		kn_gen_tsdf.Create( program, "gen_tsdf" );
		kn_fuse_tsdf.Create( program, "fuse_tsdf" );
		kn_vis_tsdf.Create( program, "vis_tsdf" );
		kn_ray_cast_tsdf.Create( program, "ray_cast_tsdf" );
		kn_ray_cast_tsdf_d.Create( program, "ray_cast_tsdf_d" );
	}

	EPH_Grid<unsigned char> mask;
	int countTiles;

	void FuseEstimate( Image<unsigned short> &depth, const EPH_Vec4 &intr, const EPH_Mat4 &extr, EPH_Grid<unsigned char> &mask, int &count ) {
		EPH_Mat4 M = extr.Transpose();
		EPH_Vec4 K = intr;

		int area = 0;

		EPH_Vec3i size = mask.size * 8;
		{
			for ( int z = 0; z < mask.size.z; z++ ) {
			for ( int y = 0; y < mask.size.y; y++ ) {
			for ( int x = 0; x < mask.size.x; x++ ) {
				EPH_Vec3 v( x, y, z );

				v = ( v * 8 + 4 );

				v = v / size.To<float>() * m_bound.Size() + m_bound.min;

				v = M.Block<3,3>(0,0) * (v - M[3].Sub<3>());

				if ( v.z <= 0.00f ) {
					continue;
				}

				float r = 17;

				r = ( r / size.To<float>() * m_bound.Size() ).MaxEntry();
				float s = ( r / v.z * K.Sub<2>() ).MaxEntry();

				EPH_Vec2 u( v.x / v.z, v.y / v.z );
				
				u.x = u.x * K[0] + K[2];
				u.y = u.y * K[1] + K[3];

				EPH_Vec2i a = (u - s).To<int>();
				EPH_Vec2i b = (u + s).To<int>();

				a = a.Max( 0 );
				b = b.Min( depth.Size() - 1 );

				if ( ( a > b ).Any() ) {
					continue;
				}

				if ( s > area ) {
					area = s;
				}

				float dmin = 10000.0f;
				float dmax = 0.0f;
				for ( int j = a.y; j <= b.y; j++ ) {
				for ( int i = a.x; i <= b.x; i++ ) {
					float d = depth[i + j * depth.Width()] * 0.001f;
					if ( d <= 0.0f ) {
						continue;
					}
					if ( d < dmin ) {
						dmin = d;
					}
					if ( d > dmax ) {
						dmax = d;
					}
				}}

				if ( dmin > dmax ) {
					continue;
				}

				if ( v.z - r > dmax ) {
					continue;
				}

				if ( v.z + r < dmin && mask[EPH_Vec3i(x, y, z)] == 0 ) {
					continue;
				}
				
				++count;
				mask[EPH_Vec3i(x, y, z)] = 1;
			}}}
		}
	}

	void Fuse( Image<unsigned short> &depth, const EPH_Vec4 &intr, const EPH_Mat4 &extr, Image< float > &dist, EPH_Grid<unsigned char> &mask, int &count ) {
		// write depth map
		cq.WriteBuffer( mem_src_d, CL_TRUE, 0, depth.Pixels().BufferSize(), depth.Pixels().Ptr() );
		
		// write distance map
		cq.WriteBuffer( mem_src_dist, CL_TRUE, dist.Pixels() );

		// point map
		cq.NDRangeKernel<2>( (kn_depth_to_points << depth.Size(), mem_src_d, mem_src_p, intr), depth.Size().MaxEntry(), 16);

		// normal map
		cq.NDRangeKernel<2>( (kn_points_to_normals << depth.Size(), mem_src_p, mem_src_n), depth.Size().MaxEntry(), 16);

		EPH_Mat4 M = extr.Transpose();
		EPH_Vec4 K = intr;

		int area = 0;

		EPH_Vec3i size = mask.size * 8;

		EPH_List<EPH_Vec4i> task;
		task.EnsureSize((m_volumeSize/8).Volume());
		{
			for ( int z = 0; z < mask.size.z; z++ ) {
			for ( int y = 0; y < mask.size.y; y++ ) {
			for ( int x = 0; x < mask.size.x; x++ ) {
#if 1
				EPH_Vec3 v( x, y, z );

				v = ( v * 8 + 4 );

				v = v / size.To<float>() * m_bound.Size() + m_bound.min;;

				v = M.Block<3,3>(0,0) * (v - M[3].Sub<3>());

				if ( v.z <= 0.00f ) {
					continue;
				}

				float r = 13;

				r = ( r / size.To<float>() * m_bound.Size() ).MaxEntry();
				float s = ( r / v.z * K.Sub<2>() ).MaxEntry();

				EPH_Vec2 u( v.x / v.z, v.y / v.z );
				
				u.x = u.x * K[0] + K[2];
				u.y = u.y * K[1] + K[3];

				EPH_Vec2i a = (u - s).To<int>();
				EPH_Vec2i b = (u + s).To<int>();

				a = a.Max( 0 );
				b = b.Min( depth.Size() - 1 );

				if ( ( a > b ).Any() ) {
					continue;
				}

				if ( s > area ) {
					area = s;
				}

				float dmin = 10000.0f;
				float dmax = 0.0f;
				for ( int j = a.y; j <= b.y; j++ ) {
					for ( int i = a.x; i <= b.x; i++ ) {
						float d = depth[i + j * depth.Width()] * 0.001f;
						if ( d <= 0.0f ) {
							continue;
						}
						if ( d < dmin ) {
							dmin = d;
						}
						if ( d > dmax ) {
							dmax = d;
						}
					}
				}

				if ( dmin > dmax ) {

					continue;
				}

				if ( v.z - r > dmax ) {
					continue;
				}

				if ( v.z + r < dmin && mask[EPH_Vec3i(x, y, z)] == 0 ) {
					continue;
				}
				
				++count;
				mask[EPH_Vec3i(x, y, z)] = 1;
#endif
				task.Push( EPH_Vec4i(x, y, z, 0) );
			}}}
		}
		cq.WriteBuffer( mem_tile_task, CL_TRUE, task );

		
		// fuse
		cq.NDRangeKernel<1>( (kn_fuse_tsdf << m_volumeSize.Cat(0), mem_tsdf, task.Num(), mem_tile_task, depth.Size(), mem_src_d, mem_src_n, mem_src_dist, K, M, m_bound[0].Cat(1), m_bound[1].Cat(1) ), task.Num() * (8*8*8), 8*8*8 );
	}
	
	EPH_List<Sensor> sensors, sensors_rgb;
	EPH_List<SensorMap> sensorMaps;

	EPH_List<EPH_Vec4> v_points, v_normals;
	
	EPH_String folder;

	void Fusion( void ) {
		mask.Create( m_volumeSize / 8 );
		mask.ZeroBuffer();
		countTiles = 0;

		EPH_Grid<unsigned char> mask_estimate( 1024 / 8 );
		mask_estimate.ZeroBuffer();
		int count_estimate = 0;
	
		{
			// filter
			const char *filter =
			"All Files (*.*)\0"					"*.*\0"
			"\0";
			
			EPH_String path;
			if ( !(( UI_Window * )( m_window ))->m_osWindow->OpenFileDialog( folder, path, filter ) ) {
				return;
			}
		}

		// clear tsdf
		cq.NDRangeKernel<3>( (kn_gen_tsdf << m_volumeSize.Cat(0), mem_tsdf), m_volumeSize.To<size_t>(), 8 );
				
		LoadCamera( folder + "/sensor-ir.mat", sensors );
		LoadCamera( folder + "/sensor-rgb.mat", sensors_rgb );

		//SaveCamera( folder + "/sensor-ir-x.mat", sensors );
		//SaveCamera( folder + "/sensor-rgb-x.mat", sensors_rgb );

		//SaveCamera( folder + "/sensor-ir.mat", sensors );
		//SaveCamera( folder + "/sensor-rgb.mat", sensors_rgb );

		sensorMaps.SetNum( sensors.Num() );

		int countImages = 0;
		for ( int i = 0; i < sensors.Num(); i++ ) {
			if ( i != 3 ) {
			//	continue;
			}

			sensorMaps[i].Create( EPH_Vec2i( 512, 424 ) );	

			// ProcessDepthMap( folder + "/K" + i + "/Pose_%d.png", 10, 30, 2 );
			
			if ( LoadDepthMap( EPH_String().SetWithFormat( folder + "/K%d/Pose_-1.png", i ), sensorMaps[i].depth ) ) {
				Image< float > dist;
				GenEdgeMap( sensorMaps[i].depth, dist );
				SavePGM( dist, folder + "/edge-" + i + ".pgm" );

				Image<EPH_RGBA8> srcImage;
				Image<EPH_RGBA8> dstImage;
				dstImage.Create( sensorMaps[i].depth.Size() );
				if (!LoadAnyImage(srcImage, EPH_String().SetWithFormat(folder + "/K%d/Pose_%d.jpeg", i, 0))) {
					continue;
				}
				GenColorMap( sensorMaps[i].depth, srcImage, dstImage, sensors[i], sensors_rgb[i] );
				GenDistanceMap( sensorMaps[i].depth, dist );

				EPH_PNG png;
				png.SaveImage( dstImage.Width(), dstImage.Height(), EPH_PIXEL_FORMAT_RGBA8, dstImage.Data(), folder + "/rgb-" + i + ".png" );

				SavePGM( dist, folder + "/dist-" + i + ".pgm" );

				FuseEstimate(sensorMaps[i].depth, sensors[i].intr, sensors[i].extr, mask_estimate, count_estimate );
				Fuse(sensorMaps[i].depth, sensors[i].intr, sensors[i].extr, dist, mask, countTiles );
				countImages++;
			//	ExportPointCloud( EPH_String( "pcl-1-" ) + i + ".ply", sensorMaps[i].points, sensorMaps[i].normals );
			} else {
				printf( "Failed : %d : %s\n", i, EPH_String().SetWithFormat( folder + "/K%d/Pose_-1.png", i ).Ptr() );
			}
		}

		int count = 0;
		for ( int z = 0; z < mask.Size().z; z++ ) {
		for ( int y = 0; y < mask.Size().y; y++ ) {
		for ( int x = 0; x < mask.Size().x; x++ ) {
			if ( mask[EPH_Vec3i(x, y, z)] ) {
				count++;
			}
		}}}

		int count_allocated_estimate = 0;
		for ( int z = 0; z < mask_estimate.Size().z; z++ ) {
		for ( int y = 0; y < mask_estimate.Size().y; y++ ) {
		for ( int x = 0; x < mask_estimate.Size().x; x++ ) {
			if ( mask_estimate[EPH_Vec3i(x, y, z)] ) {
				count_allocated_estimate++;
			}
		}}}
		printf( "estim : %d (%.2f) : %.2f (%.2f) : %d\n",
			count_allocated_estimate, 100 * (float)count_allocated_estimate/(float)mask_estimate.size.Volume(),
			(float)count_estimate/(float)countImages, 100 * (float)count_estimate/(float)mask_estimate.size.Volume()/(float)countImages,
			mask_estimate.size.Volume());

		float total = (m_volumeSize/8).Volume();
		printf( "count : %d (%.2f) : %.2f (%.2f) : %d\n", count, count/total * 100, countTiles/(float)countImages, countTiles/(float)countImages/total * 100, (m_volumeSize/8).Volume() );

		printf( "copy buffer : %d\n", cq.CopyBufferToImage( mem_tsdf, mem_img_tsdf, 0, 0, m_volumeSize.To<size_t>() ) );

		EvalImage();
	}

	void ExportPointCloud4X( void ) {
		EPH_List<EPH_Vec2> volume( m_volumeSize.Volume() );
		cq.ReadBuffer( mem_tsdf, CL_TRUE, volume );

		const float BAND = (1.50f / 512.0f * 2.0f);//1.80f / 512.0f * 3.0f * 0.25;

		v_points.SetGranularity( m_volumeSize.Volume() );
		v_normals.SetGranularity( m_volumeSize.Volume() );
		v_points.Empty();
		v_normals.Empty();

		EPH_Vec3i step( 1, m_volumeSize.x, m_volumeSize.x * m_volumeSize.y );

		const int subSize = 4;

		EPH_Grid<float> sub( subSize + 1 );

		for ( int z = 1; z < m_volumeSize.z - 1; z++ ) {
			for ( int y = 1; y < m_volumeSize.y - 1; y++ ) {
				for ( int x = 1; x < m_volumeSize.x - 1; x++ ) {
					int index = x + y * step.y + z * step.z;

					EPH_Vec2 vox[8];

					vox[0] = volume[index];
					vox[1] = volume[index + 1];
					vox[2] = volume[index + step.y];
					vox[3] = volume[index + step.y + 1];
					vox[4] = volume[index + step.z];
					vox[5] = volume[index + step.z + 1];
					vox[6] = volume[index + step.z + step.y];
					vox[7] = volume[index + step.z + step.y + 1];

					if ( vox[0].y > 0 ) {
						//	vox[0].Print();
					}

					int flag = 0;
					int outside = 0;
					for ( int i = 0; i < 8; i++ ) {
						if ( vox[i].y <= 0.0f ) {
							outside = 1;
							break;
						}

						if ( vox[i].x > 0.0f ) {
							flag |= EPH_BIT( i );
						}
					}

					if ( outside || flag == 0 || flag == 255 ) {
						continue;
					}

					for ( int lx = 0; lx <= subSize; lx++ ) {
						float e = (float)lx / (float)subSize;
						sub[EPH_Vec3i(lx, 0, 0)]				= vox[0].x + ( vox[1].x - vox[0].x ) * e;
						sub[EPH_Vec3i(lx, subSize, 0)]			= vox[2].x + ( vox[3].x - vox[2].x ) * e;
						sub[EPH_Vec3i(lx, 0, subSize)]			= vox[4].x + ( vox[5].x - vox[4].x ) * e;
						sub[EPH_Vec3i(lx, subSize, subSize)]	= vox[6].x + ( vox[7].x - vox[6].x ) * e;
					}

					// y
					for ( int lx = 0; lx <= subSize; lx++ ) {
						float a0 = sub[ EPH_Vec3i( lx, 0, 0 ) ];
						float b0 = sub[ EPH_Vec3i( lx, subSize, 0 ) ] - a0;

						float a1 = sub[ EPH_Vec3i( lx, 0, subSize ) ];
						float b1 = sub[ EPH_Vec3i( lx, subSize, subSize ) ] - a1;

						for ( int ly = 1; ly < subSize; ly++ ) {
							float e = (float)ly / (float)subSize;

							sub[ EPH_Vec3i( lx, ly, 0 ) ] = a0 + b0 * e;
							sub[ EPH_Vec3i( lx, ly, subSize ) ] = a1 + b1 * e;
						}
					}

					// z
					for ( int ly = 0; ly <= subSize; ly++ ) {
						for ( int lx = 0; lx <= subSize; lx++ ) {
							float a = sub[ EPH_Vec3i( lx, ly, 0 ) ];
							float b = sub[EPH_Vec3i(lx, ly, subSize)] - a;
							for (int lz = 1; lz < subSize; lz++) {
								float e = (float)lz / (float)subSize;

								sub[EPH_Vec3i(lx, ly, lz)] = a + b * e;
							}
	}}

					{
						for (int lz = 0; lz < subSize; lz++) {
							for (int ly = 0; ly < subSize; ly++) {
								for (int lx = 0; lx < subSize; lx++) {
									float v[8];
									v[0] = sub[EPH_Vec3i(lx + 0, ly + 0, lz + 0)];
									v[1] = sub[EPH_Vec3i(lx + 1, ly + 0, lz + 0)];
									v[2] = sub[EPH_Vec3i(lx + 0, ly + 1, lz + 0)];
									v[3] = sub[EPH_Vec3i(lx + 1, ly + 1, lz + 0)];
									v[4] = sub[EPH_Vec3i(lx + 0, ly + 0, lz + 1)];
									v[5] = sub[EPH_Vec3i(lx + 1, ly + 0, lz + 1)];
									v[6] = sub[EPH_Vec3i(lx + 0, ly + 1, lz + 1)];
									v[7] = sub[EPH_Vec3i(lx + 1, ly + 1, lz + 1)];

									int mask = 0;
									mask |= ((v[0] >= 0) << 0);
									mask |= ((v[1] >= 0) << 1);
									mask |= ((v[2] >= 0) << 2);
									mask |= ((v[3] >= 0) << 3);
									mask |= ((v[4] >= 0) << 4);
									mask |= ((v[5] >= 0) << 5);
									mask |= ((v[6] >= 0) << 6);
									mask |= ((v[7] >= 0) << 7);

									if (mask == 0 || mask == 255) {
										continue;
									}

									EPH_Vec3 p = (EPH_Vec3(lx, ly, lz) + 0.5f) / subSize + EPH_Vec3(x, y, z);
									p = p / m_volumeSize.To<float>() * m_bound.Size() + m_bound.min;

									v_points.Append(p.Cat(0));

									EPH_Vec3 grad;
									grad[0] = ((v[1] + v[3] + v[5] + v[7]) - (v[0] + v[2] + v[4] + v[6]));
									grad[1] = ((v[2] + v[3] + v[6] + v[7]) - (v[0] + v[1] + v[4] + v[5]));
									grad[2] = ((v[4] + v[5] + v[6] + v[7]) - (v[0] + v[1] + v[2] + v[3]));

									//	grad.Print();
									v_normals.Append(grad.Normalize().Cat(0));
								}
							}
						}
					}
		}}}

		EPH_SaveBitFile file;

		file.Open(folder + "/fused_points-4x.raw");
		file.Write(v_points.Num());
		for (int i = 0; i < v_points.Num(); i++) {
			file.Write(v_points[i]);
			file.Write(v_normals[i]);
		}

		ExportPly(folder + "/volume-4x.ply", v_points, v_normals);
	}

	void ExportPointCloud() {
		EPH_List<EPH_Vec2> volume( m_volumeSize.Volume() );
		cq.ReadBuffer( mem_tsdf, CL_TRUE, volume );

		const float BAND = (1.50f / 512.0f * 2.0f);//1.80f / 512.0f * 3.0f * 0.25;

		v_points.SetGranularity( m_volumeSize.x * m_volumeSize.y );
		v_normals.SetGranularity( m_volumeSize.x * m_volumeSize.y );
		v_points.Empty();
		v_normals.Empty();

		EPH_Vec3i step( 1, m_volumeSize.x, m_volumeSize.x * m_volumeSize.y );

		for ( int z = 1; z < m_volumeSize.z - 1; z++ ) {
		for ( int y = 1; y < m_volumeSize.y - 1; y++ ) {
		for ( int x = 1; x < m_volumeSize.x - 1; x++ ) {
			int index = x + y * step.y + z * step.z;

			EPH_Vec2 vox[8];
			
			vox[0] = volume[index];
			vox[1] = volume[index + 1];
			vox[2] = volume[index + step.y];
			vox[3] = volume[index + step.y + 1];
			vox[4] = volume[index + step.z];
			vox[5] = volume[index + step.z + 1];
			vox[6] = volume[index + step.z + step.y];
			vox[7] = volume[index + step.z + step.y + 1];

			if ( vox[0].y > 0 ) {
			//	vox[0].Print();
			}

			int flag = 0;
			int outside = 0;
			for ( int i = 0; i < 8; i++ ) {
				if ( vox[i].y <= 0.0f ) {
					outside = 1;
					break;
				}

				if ( vox[i].x > 0.0f ) {
					flag |= EPH_BIT( i );
				}
			}

			if ( outside || flag == 0 || flag == 255 ) {
				continue;
			}
			


			{
			//if ( vox.x <= BAND && vox.x >= 0.0f && vox.y > 0.3f ) {
				EPH_Vec3 p( x, y, z );
				p += 0.5f;

				p = p / m_volumeSize.To<float>() * m_bound.Size() + m_bound.min;

				v_points.Append( p.Cat(0) );

				EPH_Vec3 grad;
				grad[0] = ((vox[1] + vox[3] + vox[5] + vox[7]) - (vox[0] + vox[2] + vox[4] + vox[6])).x;
				grad[1] = ((vox[2] + vox[3] + vox[6] + vox[7]) - (vox[0] + vox[1] + vox[4] + vox[5])).x;
				grad[2] = ((vox[4] + vox[5] + vox[6] + vox[7]) - (vox[0] + vox[1] + vox[2] + vox[3])).x;

			//	grad.Print();
				v_normals.Append( grad.Normalize().Cat(0) );
			}
		}}}

		EPH_SaveBitFile file;

		file.Open(folder + "/fused_points.raw" );
		file.Write( v_points.Num() );
		for ( int i = 0; i < v_points.Num(); i++ ) {
			file.Write( v_points[i] );
			file.Write( v_normals[i] );
		}

		ExportPly( folder + "/volume.ply", v_points, v_normals );
	}

	virtual void PrepareGL() {
		InitCL();

		m_camera.SetFocus( m_bound.Center() );
		m_camera.SetRadius( m_bound.Size().Length() * 0.8f );
	}

	EPH_List<EPH_Vec4> image;

	Image<float> image_d;

	void RenderDepthImage( const EPH_Vec4 &intr, const EPH_Mat4 &extr ) {
		cq.NDRangeKernel<2>( (kn_ray_cast_tsdf_d<<m_renderSize_d, mem_image_d, intr, extr.Transpose(), m_volumeSize.Cat(1), mem_img_tsdf, m_bound[0].Cat(0), m_bound[1].Cat(0)), m_renderSize_d.To<size_t>(), 8 );

		image_d.Create( m_renderSize_d );
		cq.ReadBuffer( mem_image_d, CL_TRUE, 0, image_d.BufferSize(), image_d.Data() );
	}

	EPH_Camera m_renderCamera;

	void EvalImage( void ) {
		EPH_Vec4 K( m_renderSize.x * 0.5f / tan( EPH_Math::Radian(45*0.5f) ), m_renderSize.y * 0.5f / tan( EPH_Math::Radian(45*0.5f) ), m_renderSize.x * 0.5f, m_renderSize.y * 0.5f );
			EPH_Mat4 M = EPH_CreateMatrix::TranslateRotate( m_camera.view.origin, m_camera.view.axis.Transpose() ).Transpose();

			cq.NDRangeKernel<2>( (kn_ray_cast_tsdf<<m_renderSize, mem_image, K, M, m_volumeSize.Cat(1), mem_img_tsdf, m_bound[0].Cat(0), m_bound[1].Cat(0)), m_renderSize.To<size_t>(), 8 );

			image.SetNum( m_renderSize.Area() );
			cq.ReadBuffer( mem_image, CL_TRUE, image );
	}

	virtual void Render( void ) {
#if 1
		if ( !m_renderCamera.view.origin.Equals( m_camera.view.origin ) || !m_renderCamera.view.axis.Equals( m_camera.view.axis ) ) {
			m_renderCamera = m_camera;
			
			EvalImage();
		}

		glMatrixMode( GL_PROJECTION );
		glLoadIdentity();

		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();

		glRasterPos2i( -1, -1 );
		glDrawPixels(m_renderSize.x, m_renderSize.y, GL_RGBA, GL_FLOAT, image.Ptr() );

//		EPH_AuxPrimitive aux;
	//	aux.FrameBox( m_bound[0], m_bound[1] );
	//	aux.Draw();
#else
		 
		glEnable( GL_DEPTH_TEST );

		m_camera.AdjustDepth( EPH_Bound( EPH_Vec3( -1000, 0, -1000 ), EPH_Vec3( 1000, 1000, 1000 ) ) );
		m_camera.SetDepth( 0.001f, 100.0f );
		glViewport( m_rect.x0, m_rect.y0, m_rect.Width(), m_rect.Height() );

		glMatrixMode( GL_PROJECTION );
		glLoadMatrixf( m_camera.view.ProjMat().Transpose().Ptr() );

		glMatrixMode( GL_MODELVIEW );
		glLoadMatrixf( m_camera.view.ViewMat().Transpose().Ptr() );

		EPH_AuxPrimitive aux;

		aux.FrameBox( m_bound[0], m_bound[1] );
		aux.Draw();

		aux.Color3ub( 90, 100, 120 );
		aux.GridZX( 100, 1 );
		aux.Draw();

		glColor3f( 1.0f, 1.0f, 1.0f );
		glEnableClientState( GL_VERTEX_ARRAY );
		glEnableClientState( GL_COLOR_ARRAY );
		glVertexPointer(3, GL_FLOAT, 0, v_points.Ptr() );
		glColorPointer(3, GL_FLOAT, 0, v_normals.Ptr() );
		glDrawArrays(GL_POINTS, 0, v_points.Num() );
		glDisableClientState( GL_VERTEX_ARRAY );
		glDisableClientState( GL_COLOR_ARRAY );
#endif


#if 0
		glPointSize( 3 );
		glEnableClientState( GL_VERTEX_ARRAY );
		glEnableClientState( GL_COLOR_ARRAY );
		for ( int i = 0; i <= 7; i++ ) {
			DrawPoints(i);
		}
#endif

		glDisable( GL_DEPTH_TEST );
	}

	virtual bool KeyDown( const EPH_KeyState &keyState ) {
		if ( keyState.IsKey( EPH_KEY_H ) ) {
			LoadShader();
		}

		if ( keyState.IsKey( EPH_KEY_F ) ) {
			Fusion();
		}

		if ( keyState.IsKey( EPH_KEY_T ) ) {
			ExportPointCloud();
		}

		if ( keyState.IsKey( EPH_KEY_T, EPH_KEYMASK_CTRL ) ) {
			ExportPointCloud4X();
		}

		if ( keyState.IsKey( EPH_KEY_G ) ) {
			for ( int i = 0; i < sensors_rgb.Num(); i++ ) {
				RenderDepthImage( sensors_rgb[i].intr, sensors_rgb[i].extr );

				Image< unsigned short > depth( image_d.Size() );
				for ( int k = 0; k < image_d.NumPixels(); k++ ) {
					depth[k] = EPH_Math::Clamp( image_d[k], 0.0f, 10.0f ) * 1000.0f;
				}

				{
					EPH_SaveBitFile file;
					if ( file.Open( EPH_String().SetWithFormat( "K%d/rgb_depth.raw", i ) ) ) {
						file.WriteArray( depth.Data(), depth.NumPixels() );
						file.Close();
					}
				}

				SavePGM( image_d, EPH_String().SetWithFormat( "K%d/rgb_depth.pgm", i ) );
			}
		}

		if ( keyState.IsKey( EPH_KEY_6 ) ) {
			EPH_List<EPH_RGBA8> pic( m_renderSize.Area() );
			
			for ( int i = 0; i < pic.Num(); i++ ) {
				pic[i].FromVec3( image[i].Sub<3>() );
			}

			EPH_PNG png;
			png.SaveImage( m_renderSize.x, m_renderSize.y, EPH_PIXEL_FORMAT_RGBA8, pic.Ptr(), "sample.png" );
		}

		return UI_3DView::KeyDown( keyState );
	}

public :
	UI_SceneView()
	{}

public :

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
