#include <cl.h>

void sample_cl_main( void ) {
	// shader
	const char *source =
	"__kernel void add(__global float *dst, __global float *src0, __global float *src1, int iNumElements ) {"
	// find position in global arrays
	"int x = get_global_id(0);"
	// bound check (equivalent to the limit on a 'for' loop for standard/serial C code
	"if (x >= iNumElements) {"
	"   return;"
	"}"

	// process
	"dst[x] = src0[x] * src1[x];"
	"}"
	;

	// host
	vector< float > host_src0( 9 ), host_src1( 9 ), host_dst( 9 );
	for ( int i = 0; i < 9; i++ ) {
		host_src0[ i ] = i;
		host_src1[ i ] = i+1;
		host_dst [ i ] = 0;
	}

	// context
	cl::Context context;
	cl::System::CreateContext( context );

	// mem
	cl::Mem mem_src0( context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, host_src0 );
	cl::Mem mem_src1( context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, host_src1 );
	cl::Mem mem_dst ( context, CL_MEM_WRITE_ONLY, host_dst.size() * sizeof( float ) );

	// program
	const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
	cl::Program program( context, source, flags );

	// kernel
	cl::Kernel kernel( program, "add" );

	// queue
	cl::CommandQueue cq( context );

	// cq : kernel
	cq.NDRangeKernel1( ( kernel << mem_dst, mem_src0, mem_src1, (int)host_dst.size() ), 16, 4 );

	// cq : read buffer
	cq.ReadBuffer( mem_dst, CL_TRUE, host_dst );

	// finish
	cq.Finish();

	printf( "==== Result ====\n" );
	for ( int i = 0; i < 9; i++ ) {
		printf( "%f\n", host_dst[ i ] );
	}
}
