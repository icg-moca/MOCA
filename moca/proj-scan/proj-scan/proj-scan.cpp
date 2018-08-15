#include "stdafx.h"

#include <math.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <stdio.h>

#include <api/cl_wrapper.h>

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

int ceil_mod( int v, int m ) {
	v += m - 1;
	return v - v % m;
}

int _tmain(int argc, _TCHAR* argv[])
{
	std::vector<char> source;
	if ( read_text_file( "../../src/cl-shader/reduction.txt", source ) ) {
	
		printf( "source : %s\n", source.data() );
	}
	
	const int groupSize = 8;
	const int numGroups = 2;

	// host
	vector< float > host_src( numGroups * groupSize + 13 );
	for ( int i = 0; i < host_src.size(); i++ ) {
		host_src[i] = rand();
		printf( "%d : %f\n", i, host_src[i] );
	}

	// scan-cpu
	float sum_cpu = 0;
	for ( int i = 0; i < host_src.size(); i++ ) {
		sum_cpu += host_src[i];
	}

	// scan-gpu
	float sum_gpu = 0;
	{
		// context
		cl::Context context;
		cl::System::CreateContext( context );

		// mem
		cl::Mem mem_src( context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, host_src );

		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		cl::Program program( context, source.data(), flags );

		// kernel
		cl::Kernel kernel( program, "reduction" );

		// queue
		cl::CommandQueue cq( context );

		// cq : kernel
		cq.NDRangeKernel1( ( kernel << (int)host_src.size(), (int)numGroups, mem_src ), groupSize * numGroups, groupSize );

		// cq : read buffer
		cq.ReadBuffer( mem_src, CL_TRUE, 0, sizeof( float ), &sum_gpu );

		// finish
		cq.Finish();
	}

	printf( "==== Result ====\n" );
	printf( "cpu : %f\n", sum_cpu );
	printf( "gpu : %f\n", sum_gpu );
	return 0;
}

