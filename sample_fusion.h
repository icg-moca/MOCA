#include <camera_calibration.h>
#include <fusion.h>


void LoadFrame( int index, cv::Mat &kinect_mat, cv::Mat &depth ) {
	char path[ 256 ];
	sprintf( path, "fusion sample data/frame%d.raw", index );

	FILE *fp = fopen( path, "rb" );
	if ( !fp ) {
		return;
	}

	kinect_mat = cv::Mat::eye( 4, 4, CV_64F );
	fread( kinect_mat.ptr(), sizeof( double ), 4 * 4, fp );

	int width, height;
	fread( &width , sizeof( int ), 1, fp );
	fread( &height, sizeof( int ), 1, fp );

	depth = cv::Mat( height, width, CV_32F );
	fread( depth.ptr(), sizeof( float ), width * height, fp );

	cout << "load frame " << index << endl;
	cout << kinect_mat << endl;
	cout << width << " " << height << endl;

	fclose( fp );
}


void LoadText( vector<char> &str, const char *path ) {
	FILE *fp = fopen( path, "rb" );
	if ( !fp ) {
		return;
	}

	fseek( fp, 0, SEEK_END );
	size_t size = ftell( fp );
	fseek( fp, 0, SEEK_SET );

	str.resize( size + 1 );
	fread( str.data(), 1, size, fp );
	str[ size ] = '\0';

	fclose( fp );
}

void sample_main_fusion() {
	// cali
	Moc::Calibrator_Kinect_To_Vicon cali;
	cali.LoadCali( "cam_cali.txt" );

	// fusion setup
	Fusion fusion;
	fusion.image_size = cl::int2( 512, 424 );
	fusion.volume_size = cl::int4( 256 );

	fusion.bound[0] = cl::float4( -1.28, 0.00 - 0.5, -1.28, 0 );
	fusion.bound[1] = cl::float4( +1.28, 2.56 - 0.5, +1.28, 0 );

	fusion.global_size2 = 512;
	fusion.local_size2 = 4;

	fusion.global_size3 = 256;
	fusion.local_size3 = 8;

	fusion.Create();
	{
		vector< char > shader;
		LoadText( shader, "fusion.cl" );
		fusion.UpdateShader( shader.data() );
	}

	// fuse frame
	cv::Mat depth_map;
	cv::Mat kinect_mat;
	fusion.Clear();
	for ( int i = 75; i < 80; i++ ){
		LoadFrame( i, kinect_mat, depth_map );
		// fusion.ICP( kinect_mat * cali.cam_extr.inv() );
		fusion.FuseFrame( cali.cam_intr.Intr(), kinect_mat * cali.cam_extr.inv(), (float*)depth_map.ptr() );
		//vector<cl::float4> points, normals;
		//fusion.BackProjectPoints(cali.cam_intr.Intr(), kinect_mat * cali.cam_extr.inv(), (float*)depth_map.ptr(), points, normals );
	}

	// render
	double temp[16] = {
		1, -0, 0, 0,
		0, 0.8660253882408142, 0.5, 1.803818464279175,
		-0, -0.5, 0.8660253882408142, 3.124305009841919,
		0, 0, 0, 1
	};
	cv::Mat M = cv::Mat( 4, 4, CV_64F, temp );

	vector< cl::float4 > color_map;
	fusion.RayCast( 60.0f, M, color_map );

	cv::Mat color_mat = cv::Mat( 512, 512, CV_32FC4, color_map.data() );
	cv::flip( color_mat, color_mat, 0 );
	//	cv::flip( color_mat, color_mat, 1 );
	cv::imshow( "ray cast", color_mat );
	cv::waitKey(-1);
}