#include "stdafx.h"

#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <iostream>

#include <math/func.h>
#include <math/matrix.h>
#include <math/SparseBlockSquareMatrix.h>

#include <import.h>
#include <api/cl_wrapper.h>

#include <time.h>

Eigen::VectorXf pcg( const SparseBlockSquareMatrix &A, const Eigen::VectorXf &b, int maxIters, float threshold = 1e-4 ) {
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

	int m_numIters = 0;
	clock_t begin = clock();
	for ( int i = 0; i < maxIters; i++ ) {
		m_numIters++;

		Eigen::VectorXf Mr = M.array() * r.array();
		float rMr = r.dot( Mr );
		if ( rMr < threshold * A.size() ) {
			break;
		}
		if ( threshold <= 0 && rMr < 1e-6 ) {
			rMr = 0.0f;
		} else {
			rMr = 1.0f / rMr;
		}
		p += Mr * rMr;

		Eigen::VectorXf Ap = A * p;
		float pAp = p.dot( Ap );
		if ( threshold <= 0 && pAp < 1e-6 ) {
			pAp = 0.0f;
		} else {
			pAp = 1.0f / pAp;
		}
		x +=  p * pAp;
		r -= Ap * pAp;

		//float rme = sqrt( r.dot( r ) / A.size() );
		//if ( rme < 1e-6 ) {
		//	break;
		//}
	}
	clock_t end = clock();
	double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
	printf( "pcg cpu (ms) : %.3f : %.3f : %d\n", time_spent * 1000, time_spent * 1000 / m_numIters, m_numIters );

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
}

class cl_block_pcg {
public :
	int m_size, m_blockSize, m_gridSize;

	int m_numIters;
	
	// mem
	// static matrix
	cl::Mem mem_M;
	cl::Mem mem_A_blocks;
	cl::Mem mem_A_blockInfos;
	cl::Mem mem_A_rowScan;

	// updating vector
	cl::Mem mem_x;
	cl::Mem mem_p;
	cl::Mem mem_r;
	
	// temp vector
	cl::Mem mem_Mv;
	cl::Mem mem_dot;
	
	// kernel
	cl::Kernel kn_reduction;
	cl::Kernel kn_zero_v;
	cl::Kernel kn_mul_v_v;
	cl::Kernel kn_mad_v_s;
	cl::Kernel kn_mul_m_v;
	
	void genTopology( cl::Context &context, SparseBlockSquareMatrix &A ) {
		m_size = A.size();
		m_blockSize = A.blockSize();
		m_gridSize = A.gridSize();

		size_t bufferSize = sizeof( float ) * m_size;
		
		// static topology		
		mem_A_blockInfos.CreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, A.m_blockInfos );
		mem_A_rowScan.CreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, A.m_rowScan );

		// matrix
		mem_A_blocks.CreateBuffer( context, CL_MEM_READ_WRITE, A.m_blocks.size() * sizeof( float ) );
		mem_M.CreateBuffer( context, CL_MEM_READ_WRITE, bufferSize ); // A diagonal

		// updating vector
		mem_x.CreateBuffer( context, CL_MEM_READ_WRITE, bufferSize );
		mem_p.CreateBuffer( context, CL_MEM_READ_WRITE, bufferSize );
		mem_r.CreateBuffer( context, CL_MEM_READ_WRITE, bufferSize );
	
		// temp vector
		mem_Mv.CreateBuffer( context, CL_MEM_READ_WRITE, bufferSize ); // Mr, Ap
		mem_dot.CreateBuffer( context, CL_MEM_READ_WRITE, bufferSize ); // rMr, pAp
	}

	void initMatrix( cl::CommandQueue &cq, SparseBlockSquareMatrix &A ) {
		size_t bufferSize = sizeof( float ) * m_size;
		int local_work_size = 256;
		int global_work_size = ceil( m_size, local_work_size );

		Eigen::VectorXf M = A.diagonal();
		for ( int i = 0; i < M.rows(); i++ ) {
			if ( fabs( M[i] ) > 1e-6f ) {
				M[i] = 1.0f / M[i];
			} else {
				M[i] = 1.0f;
			}
		}

		cq.WriteBuffer( mem_A_blocks, CL_TRUE, A.m_blocks );
		cq.WriteBuffer( mem_M, CL_TRUE, 0, bufferSize, M.data() );
	}

	void initVector( cl::CommandQueue &cq, Eigen::VectorXf &b ) {
		size_t bufferSize = sizeof( float ) * m_size;
		int local_work_size = 256;
		int global_work_size = ceil( m_size, local_work_size );
		
		// x = 0;
		cq.Kernel1D( (kn_zero_v << m_size, mem_x), global_work_size, local_work_size );
		//cq.WriteBuffer( mem_x, CL_TRUE, 0, bufferSize, x.data() );

		// p = 0;
		cq.Kernel1D( (kn_zero_v << m_size, mem_p), global_work_size, local_work_size );

		// r = b = b - A * x
		cq.WriteBuffer( mem_r, CL_TRUE, 0, bufferSize, b.data() );
	}

	void getResult( cl::CommandQueue &cq, Eigen::VectorXf &x ) {
		x = Eigen::VectorXf::Zero( m_size );
		size_t bufferSize = sizeof( float ) * m_size;
		cq.ReadBuffer( mem_x, CL_TRUE, 0, bufferSize, x.data() );
	}

	void compute(
		cl::CommandQueue &cq,
		int maxIters,
		float threshold,
		int local_work_size,
		int local_work_size_A
	) {
		int global_work_size = ceil( m_size, local_work_size );

		m_numIters = 0;	
		clock_t begin = clock();
		for ( int i = 0; i < maxIters; i++ ) {
			m_numIters++;

			// Mr = M * r;
			cq.Kernel1D( (kn_mul_v_v << m_size, mem_M, mem_r, mem_Mv ), global_work_size, local_work_size );

			// rMr = r dot Mr;
			float rMr = cl_dot_product( cq, kn_mul_v_v, kn_reduction, mem_r, mem_Mv, mem_dot, m_size, global_work_size, local_work_size );
			if ( rMr < threshold * m_size ) {
				break;
			}
			if ( threshold <= 0 && rMr < 1e-6 ) {
				rMr = 0.0f;
			} else {
				rMr = 1.0f / rMr;
			}

			// p += Mr / rMr'
			cq.Kernel1D( (kn_mad_v_s << m_size, mem_Mv, rMr, mem_p ), global_work_size, local_work_size );

			// Ap = A * p
			cq.Kernel1D( (kn_zero_v << m_size, mem_Mv), global_work_size, local_work_size );
			//cq.Kernel1D( (kn_mul_m_v << m_gridSize, mem_A_rowScan, mem_A_blockInfos, mem_A_blocks, mem_p, mem_Mv ), ceil( m_gridSize, 256 ), 256 );
			cq.Kernel1D( (kn_mul_m_v << m_gridSize, m_blockSize, mem_A_rowScan, mem_A_blockInfos, mem_A_blocks, mem_p, mem_Mv, cl::Arg( m_blockSize * local_work_size_A * sizeof( float ) ) ), m_gridSize *local_work_size_A, local_work_size_A );

			// pAp = p dot Ap
			float pAp = cl_dot_product( cq, kn_mul_v_v, kn_reduction, mem_p, mem_Mv, mem_dot, m_size, global_work_size, local_work_size );
			if ( threshold <= 0 && pAp < 1e-6 ) {
				pAp = 0.0f;
			} else {
				pAp = 1.0f / pAp;
			}

			// r -= Ap / pAp;
			cq.Kernel1D( (kn_mad_v_s << m_size, mem_Mv, -pAp, mem_r), global_work_size, local_work_size );

			// x +=  p / pAp;
			cq.Kernel1D( (kn_mad_v_s << m_size, mem_p , pAp, mem_x), global_work_size, local_work_size );

#if 0
			float rme =  cl_dot_product( cq, kn_mul_v_v, kn_reduction, mem_r, mem_r, mem_dot, m_size, global_work_size, local_work_size );
			rme = sqrt( rme / m_size );
			printf( "PCG : %d : %.3f\n", i, rme );
			if ( rme < 1e-6 ) {
				break;
			}
#endif
		}
		cq.Finish();

		clock_t end = clock();
		double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

		printf( "pcg gpu (ms) : %.3f : %.3f : %d\n", time_spent * 1000, time_spent * 1000 / m_numIters, m_numIters );
	}

	void compileProgram( cl::Context &context ) {
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
	}
};

void TestPCG( void ) {
	int dim = 12 * 100;

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

	std::cout << std::endl << "=============== Eigen ===============" << std::endl;
	{
		// fill A and b
		Eigen::ConjugateGradient<Eigen::MatrixXf, Eigen::Lower|Eigen::Upper> cg;
		clock_t begin = clock();
		
		cg.compute(A);
		x = cg.solve(b);
		
		clock_t end = clock();
		double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
		printf( "cpu eigen (ms) : %.3f : %.3f : %d\n", time_spent * 1000, time_spent * 1000 / cg.iterations(), cg.iterations() );
		std::cout << cg.error() << std::endl;
	}

	SparseBlockSquareMatrix B;
	B.createFromDenseMatrix( A, 12 );
	
	float threshold = 0.0f;

	std::cout << std::endl << "=============== CPU ===============" << std::endl;
	{
		Eigen::VectorXf xx = pcg( B, b, 100, threshold );
		std::cout << ( xx - x ).dot( xx - x ) / xx.rows() << std::endl << std::endl;
	}

	std::cout << std::endl << "=============== GPU ===============" << std::endl;
	{
		// context
		cl::Context context;
		cl::System::CreateContext( context );
		cl::CommandQueue cq( context );

		cl_block_pcg pcg;
		pcg.compileProgram( context );

		pcg.genTopology( context, B );

		pcg.initMatrix( cq, B );
		pcg.initVector( cq, b );

		pcg.compute( cq, 100, threshold, 256, 256 );

		Eigen::VectorXf xx;
		pcg.getResult( cq, xx );
		std::cout << ( xx - x ).dot( xx - x ) / xx.rows() << std::endl << std::endl;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	TestPCG();
	
	return 0;
}

