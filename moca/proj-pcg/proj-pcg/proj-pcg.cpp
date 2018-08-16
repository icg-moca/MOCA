#include "stdafx.h"

#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <iostream>

#include <math/matrix.h>
#include <math/SparseBlockSquareMatrix.h>

#include <import.h>
#include <api/cl_wrapper.h>

#include <time.h>

Eigen::VectorXf pcg( const SparseBlockSquareMatrix<12> &A, const Eigen::VectorXf &b, int maxIters, float threshold = 1e-6 ) {
	Eigen::VectorXf M = A.diagonal();
	for ( int i = 0; i < M.rows(); i++ ) {
		if ( fabs( M[i] ) > threshold ) {
			M[i] = 1.0f / M[i];
		} else {
			M[i] = 1.0f;
		}
	}

	Eigen::VectorXf x = Eigen::VectorXf::Zero(A.size());
	Eigen::VectorXf p = Eigen::VectorXf::Zero(A.size());
	Eigen::VectorXf r = b; // b - A * x

	int numIters = 0;
	clock_t begin = clock();
	for ( int i = 0; i < maxIters; i++ ) {
		numIters++;

		Eigen::VectorXf Mr = M.array() * r.array();
		float r_rMr = std::max(1.0f / r.dot( Mr ), 1e-6f);
		p += Mr * r_rMr;

		Eigen::VectorXf Ap = A * p;
		float r_pAp = std::max(1.0f / p.dot( Ap ), 1e-6f );
		x +=  p * r_pAp;
		r -= Ap * r_pAp;

		//float rme = sqrt( r.dot( r ) / A.size() );
		//if ( rme < 1e-6 ) {
		//	break;
		//}
	}
	clock_t end = clock();
	double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
	printf( "cpu pcg time %fms : %d\n", time_spent * 1000, numIters );

	return x;
}

void cl_reduction(
	cl::CommandQueue &cq,
	cl::Kernel &kn_reduction,
	cl::Mem &mem,
	int elem_num,
	int global_work_size, int local_work_size
) {
	int group_num = ( global_work_size + local_work_size - 1 ) / local_work_size;
	global_work_size = local_work_size * group_num;
	while ( elem_num > 1 ) { // 
		int activeGroupNum = std::min( group_num, ( elem_num + local_work_size - 1 ) / local_work_size );
		cq.Kernel1D( ( kn_reduction << (int)elem_num, (int)activeGroupNum, cl::Arg( sizeof( int ) * local_work_size ), mem ), local_work_size * activeGroupNum, local_work_size );
		elem_num = activeGroupNum;
	}
}

float cl_dot_product(
	cl::CommandQueue &cq,
	cl::Kernel &kn_mul_v_v, cl::Kernel &kn_reduction,
	cl::Mem &mem_src0, cl::Mem &mem_src1, cl::Mem &mem_dst,
	int elem_num,
	int global_work_size, int local_work_size
) {
	// rMr = r * Mr;
	cq.Kernel1D( (kn_mul_v_v << elem_num, mem_src0, mem_src1, mem_dst ), global_work_size, local_work_size );
		
	// rMr' = r dot rMr;
	cl_reduction( cq, kn_reduction, mem_dst, elem_num, global_work_size, local_work_size );
	
	float dot = 0.0f;
	cq.ReadBuffer( mem_dst, CL_TRUE, 0, sizeof( float ), &dot );
	return dot;

	/*
	int group_num = ( global_work_size + local_work_size - 1 ) / local_work_size;
	global_work_size = local_work_size * group_num;

	// dst = src0 * src1
	cq.Kernel1D( (kn_mul_v_v << mem_src0, mem_src1, mem_dst ), global_work_size, local_work_size );

	// sum( dst )
	while ( elem_num > 1 ) {
		int activeGroupNum = std::min( group_num, ( elem_num + local_work_size - 1 ) / local_work_size );
		cq.Kernel1D( ( kn_reduction << (int)elem_num, (int)activeGroupNum, cl::Arg( sizeof( int ) * local_work_size ), mem_dst ), local_work_size * activeGroupNum, local_work_size );
		elem_num = activeGroupNum;
	}*/
}

int ceil( int x, int m ) {
	x += m - 1;
	return x - x % m;
}

Eigen::VectorXf cl_pcg( SparseBlockSquareMatrix<12> &A, Eigen::VectorXf &b, int maxIters, float threshold = 1e-6 ) {
	// context
	cl::Context context;
	cl::System::CreateContext( context );

	float bufferSize = sizeof( float ) * b.size();

	// mem
	
	// static matrix
	cl::Mem mem_M				( context, CL_MEM_READ_WRITE, bufferSize ); // A diagonal
	cl::Mem mem_A_blocks		( context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, A.blocks );
	cl::Mem mem_A_blockInfos	( context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, A.blockInfos );
	cl::Mem mem_A_rowScan		( context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, A.rowScan );

	// updating vector
	cl::Mem mem_x	( context, CL_MEM_READ_WRITE, bufferSize );
	cl::Mem mem_p	( context, CL_MEM_READ_WRITE, bufferSize );
	cl::Mem mem_r	( context, CL_MEM_READ_WRITE, bufferSize );
	
	// temp vector
	cl::Mem mem_Mv	( context, CL_MEM_READ_WRITE, bufferSize ); // Mr, Ap
	cl::Mem mem_dot	( context, CL_MEM_READ_WRITE, bufferSize ); // rMr, pAp
	
	// kernel
	cl::Kernel kn_reduction;
	cl::Kernel kn_zero_v;
	cl::Kernel kn_mul_v_v;
	cl::Kernel kn_mad_v_s;
	cl::Kernel kn_mul_m_v;

	// read source
	{
		std::vector<char> source;
		read_text_file( "../../src/cl-shader/reduction.txt", source );
		const char *sources[] = {
			"#define T float\n",
			source.data(),
			0
		};

		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		cl::Program program;
		program.Create( context, 2, sources, 0, flags );

		// kernal
		kn_reduction.Create( program, "reduction" );
	}

	{
		std::vector<char> source;
		read_text_file( "../../src/cl-shader/vector.txt", source );
		const char *sources[] = {
			source.data(),
			0
		};

		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		cl::Program program;
		program.Create( context, 1, sources, 0, flags );

		// kernal
		kn_zero_v.Create( program, "zero_v" );
		kn_mul_v_v.Create( program, "mul_v_v" );
		kn_mad_v_s.Create( program, "mad_v_s" );

		kn_mul_m_v.Create( program, "mul_m_v" );
	}

//	return Eigen::VectorXf::Zero(A.size());
	// queue
	cl::CommandQueue cq( context );

	int local_work_size = 256;
	int global_work_size = ( A.size() + local_work_size - 1 ) / local_work_size * local_work_size;

	Eigen::VectorXf M = A.diagonal();
	for ( int i = 0; i < M.rows(); i++ ) {
		if ( fabs( M[i] ) > threshold ) {
			M[i] = 1.0f / M[i];
		} else {
			M[i] = 1.0f;
		}
	}

	cq.WriteBuffer( mem_M, CL_TRUE, 0, bufferSize, M.data() );

	Eigen::VectorXf x = Eigen::VectorXf::Zero(A.size());
	Eigen::VectorXf p = Eigen::VectorXf::Zero(A.size());
	Eigen::VectorXf r = b; // b - A * x

	// x = 0;
	cq.Kernel1D( (kn_zero_v << A.size(), mem_x), global_work_size, local_work_size );
	// p = 0;
	cq.Kernel1D( (kn_zero_v << A.size(), mem_p), global_work_size, local_work_size );
	// r = b = b - A * x
	cq.WriteBuffer( mem_r, CL_TRUE, 0, bufferSize, b.data() );
	
	int numIters = 0;
	
	clock_t begin = clock();
	for ( int i = 0; i < maxIters; i++ ) {
		numIters++;

		// Mr = M * r;
		cq.Kernel1D( (kn_mul_v_v << A.size(), mem_M, mem_r, mem_Mv ), global_work_size, local_work_size );

		// rMr = r dot Mr;
		float rMr = cl_dot_product( cq, kn_mul_v_v, kn_reduction, mem_r, mem_Mv, mem_dot, A.size(), global_work_size, local_work_size );
		rMr = std::max( rMr, 1e-6f );

		// p += Mr / rMr'
		cq.Kernel1D( (kn_mad_v_s << A.size(), mem_Mv, 1.0f / rMr, mem_p ), global_work_size, local_work_size );

		// Ap = A * p
		cq.Kernel1D( (kn_zero_v << A.size(), mem_Mv), global_work_size, local_work_size );
		//cq.Kernel1D( (kn_mul_m_v << A.gridSize, mem_A_rowScan, mem_A_blockInfos, mem_A_blocks, mem_p, mem_Mv ), ceil( A.gridSize, 16 ), 16 );
		cq.Kernel1D( (kn_mul_m_v << A.gridSize, mem_A_rowScan, mem_A_blockInfos, mem_A_blocks, mem_p, mem_Mv ), A.gridSize * 64, 64 );

		// pAp = p dot Ap
		float pAp = cl_dot_product( cq, kn_mul_v_v, kn_reduction, mem_p, mem_Mv, mem_dot, A.size(), global_work_size, local_work_size );
		pAp = std::max( pAp, 1e-6f );

		// r -= Ap / pAp;
		cq.Kernel1D( (kn_mad_v_s << A.size(), mem_Mv, -1.0f / pAp, mem_r), global_work_size, local_work_size );

		// x +=  p / pAp;
		cq.Kernel1D( (kn_mad_v_s << A.size(), mem_p , +1.0f / pAp, mem_x), global_work_size, local_work_size );

//		float rme =  cl_dot_product( cq, kn_mul_v_v, kn_reduction, mem_r, mem_r, mem_dot, A.size(), global_work_size, local_work_size );
//		rme = sqrt( rme / A.size() );
//		printf( "PCG : %d : %f\n", i, rme );
//		if ( rme < 1e-6 ) {
		//	break;
//		}
	}
	cq.Finish();

	clock_t end = clock();
	double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

	printf( "time pcg gpu %fms : %d\n", time_spent * 1000, numIters );

	cq.ReadBuffer( mem_x, CL_TRUE, 0, bufferSize, x.data() );
	return x;
}

void TestPCG( void ) {
	int dim = 12 * 120;

	// A : positive definite matrix
	Eigen::MatrixXf A = Eigen::MatrixXf::Random( dim, dim );
	A = A * A.transpose();
	A += Eigen::MatrixXf::Identity( dim, dim ) * dim;
	printf( "Random A\n" );
	//std::cout << A << std::endl;

	// x
	Eigen::VectorXf x = Eigen::VectorXf::Random( dim );

	// b
	Eigen::VectorXf b = A * x;

#if 0
	{
		clock_t begin = clock();
		Eigen::VectorXf xx = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
		clock_t end = clock();
		double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
		printf( "cpu eigen time %fms\n", time_spent * 1000 );
		
		std::cout << ( xx - x ).dot( xx - x ) / xx.rows() << std::endl << std::endl;
	}
#endif

	{
		// fill A and b
		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower|Eigen::Upper> cg;
		clock_t begin = clock();
		
		cg.compute(A);
		x = cg.solve(b);
		
		clock_t end = clock();
		double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
		printf( "cpu eigen time %fms\n", time_spent * 1000 );

		std::cout << "#iterations:     " << cg.iterations() << std::endl;
		std::cout << "estimated error: " << cg.error()      << std::endl;
	}

	{
		SparseBlockSquareMatrix<12> B;
		B = A;
		Eigen::VectorXf xx = cl_pcg( B, b, 1000 );
		std::cout << ( xx - x ).dot( xx - x ) / xx.rows() << std::endl << std::endl;
	}

	{
		SparseBlockSquareMatrix<12> B;
		B = A;
		Eigen::VectorXf xx = pcg( B, b, 1000 );
		std::cout << ( xx - x ).dot( xx - x ) / xx.rows() << std::endl << std::endl;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	TestPCG();
	
	return 0;
}

