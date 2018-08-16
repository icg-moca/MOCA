#include "stdafx.h"

#include <math.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <stdio.h>

#include <import.h>
#include <api/cl_wrapper.h>

void cl_reduction( cl::CommandQueue &cq, cl::Kernel &kernel, cl::Mem &mem, int elem_num, int global_work_size, int local_work_size ) {
	int group_num = ( global_work_size + local_work_size - 1 ) / local_work_size;
	global_work_size = local_work_size * group_num;
	while ( elem_num > 1 ) { // 
		int activeGroupNum = std::min( group_num, ( elem_num + local_work_size - 1 ) / local_work_size );
		cq.NDRangeKernel1( ( kernel << (int)elem_num, (int)activeGroupNum, cl::Arg( sizeof( int ) * local_work_size ), mem ), local_work_size * activeGroupNum, local_work_size );
		elem_num = activeGroupNum;
	}
}

int main(int argc, _TCHAR* argv[])
{
	// host
	vector< int > host_src( 400000 );
	for ( int i = 0; i < host_src.size(); i++ ) {
		host_src[i] = rand() % 256;
	//	printf( "%d : %f\n", i, host_src[i] );
	}

	// reduction-cpu
	int sum_cpu = 0;
	{
		for ( int i = 0; i < host_src.size(); i++ ) {
			sum_cpu += host_src[i];
		}
	}
	
	// reduction-gpu
	int sum_gpu;
	{
		// context
		cl::Context context;
		cl::System::CreateContext( context );

		// mem
		cl::Mem mem_src( context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, host_src );
		
		// read source
		std::vector<char> source;
		read_text_file( "../../src/cl-shader/reduction.txt", source );
		const char *sources[] = {
			"#define T int\n",
			source.data(),
			0
		};

		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		cl::Program program;
		program.Create( context, 2, sources, 0, flags );

		// kernel
		cl::Kernel kn_reduction( program, "reduction" );
		program.Release();

		// queue
		cl::CommandQueue cq( context );

		cl_reduction( cq, kn_reduction, mem_src, host_src.size(), host_src.size(), 256 );

		// cq : read buffer
		cq.ReadBuffer( mem_src, CL_TRUE, 0, sizeof( int ), &sum_gpu );

		// finish
		cq.Finish();
	}

	printf( "==== Result ====\n" );
	printf( "cpu : %d\n", sum_cpu );
	printf( "gpu : %d\n", sum_gpu );
	return 0;
}

