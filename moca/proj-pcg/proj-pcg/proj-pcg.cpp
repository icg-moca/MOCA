#include "stdafx.h"

#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <api/cl_wrapper.h>

#include <math/misc.h>
#include <math/SparseBlockSquareMatrix.h>

void pcg_cpu( const SparseBlockSquareMatrix<12> &A, const Eigen::VectorXf &b, Eigen::VectorXf &x, int maxIters, float threshold = 1e-6 ) {
	Eigen::VectorXf M = A.diagonal();
	for ( int i = 0; i < M.rows(); i++ ) {
		if ( fabs( M[i] ) > threshold ) {
			M[i] = 1.0f / M[i];
		} else {
			M[i] = 1.0f;
		}
	}

	x = Eigen::VectorXf::Zero(A.size());

	Eigen::VectorXf p = Eigen::VectorXf::Zero(A.size());
	Eigen::VectorXf r = b; // b - A * x

	for ( int i = 0; i < maxIters; i++ ) {
		float rme = sqrt( r.dot( r ) / A.size() );
		std::cout << i << " : " << rme << std::endl;
		if ( rme < 1e-6 ) {
			break;
		}
		Eigen::VectorXf Mr = M.array() * r.array();
		float r_rMr = 1.0f / r.dot( Mr );
		p += Mr * r_rMr;

		Eigen::VectorXf Ap = A * p;
		float r_pAp = 1.0f / p.dot( Ap );
		x +=  p * r_pAp;
		r -= Ap * r_pAp;
	}
}

void test_pcg_eigen( const Eigen::MatrixXf &A, const Eigen::MatrixXf &b, Eigen::VectorXf &x ) {
	x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

void test_pcg_cpu( const Eigen::MatrixXf &A, const Eigen::MatrixXf &b, Eigen::VectorXf &x ) {
	SparseBlockSquareMatrix<12> B;
	B = A;
	pcg_cpu( B, b, x, 100 );
}

void test_pcg_gpu( const Eigen::MatrixXf &A, const Eigen::MatrixXf &b, Eigen::VectorXf &x ) {
	SparseBlockSquareMatrix<12> B;
	B = A;


	pcg_cpu( B, b, x, 100 );
}

int _tmain(int argc, _TCHAR* argv[])
{
	const int size = 48;
	
	// A
	Eigen::MatrixXf A;	
	random_positive_definite_matrix( size, A );
	
	// x
	Eigen::VectorXf x = Eigen::VectorXf::Random( size );

	// b
	Eigen::VectorXf b = A * x;

	
	Eigen::VectorXf xx;
	// eigen
	test_pcg_eigen( A, b, xx );
	std::cout << "eigen : " << rme( x, xx ) << std::endl;

	// cpu
	test_pcg_cpu( A, b, xx );
	std::cout << "cpu : " << rme( x, xx ) << std::endl;
	
	//test_pcg_gpu( const Eigen::MatrixXf &A, const Eigen::MatrixXf &b, Eigen::VectorXf &x )

	return 0;
}

