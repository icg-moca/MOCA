// proj-marker-transform.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


#include <vector>
#include <iostream>
#include <time.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <math/func.h>
#include <math/matrix.h>
#include <import.h>

#include <math/SparseBlockSquareMatrix.h>

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
		//	break;
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

		float rme = sqrt( r.dot( r ) / A.size() );
		if ( rme < 1e-6 ) {
			break;
		}
	}
	clock_t end = clock();
	double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
	printf( "pcg cpu (ms) : %.3f : %.3f : %d\n", time_spent * 1000, time_spent * 1000 / m_numIters, m_numIters );

	return x;
}

float icp_point_to_point_svd(
	const std::vector< Eigen::Vector3f > &src_points,
	const std::vector< Eigen::Vector3f > &dst_points,
	Eigen::Matrix3f &out_r, // rotation
	Eigen::Vector3f &out_t  // translation
) {
	if ( src_points.empty() ) {
		out_r.setIdentity();
		out_t.setZero();
		return 0.0f;
	}
		
	// E(src)
	Eigen::Vector3f src_mean = mean( src_points );

	// E(dst)
	Eigen::Vector3f dst_mean = mean( dst_points );

	// A = Sum( ( src - E(src) ) * ( dst - E(dst) )' )
	Eigen::Matrix3f A = Eigen::Matrix3f::Identity() * 1e-6f;
	for ( int i = 0; i < src_points.size(); i++ ) {
		A += ( src_points[i] - src_mean ) * ( dst_points[i] - dst_mean ).transpose();
	}

	// SVD : A = U * D * V'
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// R = V * U'
	out_r = svd.matrixV() * svd.matrixU().transpose();

	// T = E(dst) - R * E(src)
	out_t = dst_mean - out_r * src_mean;

	// rme
	return rme( src_points, dst_points, out_r, out_t );
}

void init_affine_transforms( int size, Eigen::VectorXf &x, const Eigen::Matrix3f &rot, const std::vector< Eigen::Vector3f > &src, const std::vector< Eigen::Vector3f > &dst ) {
	for ( int i = 0; i < size; i++ ) {
		x.block( 12 * i + 3 * 0, 0, 3, 1 ) = rot.col(0); Eigen::Vector3f( 1, 0, 0 );
		x.block( 12 * i + 3 * 1, 0, 3, 1 ) = rot.col(1); Eigen::Vector3f( 0, 1, 0 );
		x.block( 12 * i + 3 * 2, 0, 3, 1 ) = rot.col(2); Eigen::Vector3f( 0, 0, 1 );
		x.block( 12 * i + 3 * 3, 0, 3, 1 ) = dst[i] - src[i];
	}
}

void EvalRigidTerm( int size, Eigen::MatrixXf &JtJ, Eigen::VectorXf &Jtr, Eigen::VectorXf &x, std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &dst, float &E ) {	
	// E-rigid
	float w_rigid = 1.0f;
	{
		for ( int i = 0; i < size; i++ ) {
			Eigen::Vector3f M[4];
			M[0] = x.block( i * 12 + 0 * 3, 0, 3, 1 );
			M[1] = x.block( i * 12 + 1 * 3, 0, 3, 1 );
			M[2] = x.block( i * 12 + 2 * 3, 0, 3, 1 );
			M[3] = x.block( i * 12 + 3 * 3, 0, 3, 1 );

			// ck dot ck - 1
			for ( int k = 0; k < 3; k++ ) {
				float r = M[k].dot( M[k] ) - 1;
			
				Eigen::VectorXf J = Eigen::VectorXf::Zero( 12 );
				J.block( k * 3, 0, 3, 1 ) = M[k] + M[k];

				E += r * r * w_rigid;
				JtJ.block( i * 12, i * 12, 12, 12 ) += J * J.transpose() * w_rigid; // block (i, i)
				Jtr.block( i * 12, 0, 12, 1 ) += J * r * w_rigid; // block ( i )
			}

			// ck dot cj
			for ( int k = 0; k < 3; k++ ) {
				int j = ( k + 1 ) % 3;

				float r = M[k].dot( M[j] );

				E += r * r * w_rigid;

				Eigen::VectorXf J = Eigen::VectorXf::Zero( 12 );

				J.block( k * 3, 0, 3, 1 ) = M[j];
				J.block( j * 3, 0, 3, 1 ) = M[k];

				JtJ.block( i * 12, i * 12, 12, 12 ) += J * J.transpose() * w_rigid; // block (i, i)
				Jtr.block( i * 12, 0, 12, 1 ) += J * r * w_rigid; // block ( i )
			}
		}
	}
	printf( "E-rigid : %f\n", E );
}

void EvalSmoothTerm( int size, Eigen::MatrixXf &JtJ, Eigen::VectorXf &Jtr, Eigen::VectorXf &x, std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &dst, float &out_E, const std::vector<std::vector<int>> &links ) {	
	// E-smooth
	float E = 0;
	float w_smooth = 1.0f;
	{
		for ( int i = 0; i < size; i++ ) {
		for ( int k = 0; k < links[i].size(); k++ ) {
			int j = links[i][k];

			Eigen::Vector3f Mi[4], Mj[4];
			Mi[0] = x.block( i * 12 + 0 * 3, 0, 3, 1 );
			Mi[1] = x.block( i * 12 + 1 * 3, 0, 3, 1 );
			Mi[2] = x.block( i * 12 + 2 * 3, 0, 3, 1 );
			Mi[3] = x.block( i * 12 + 3 * 3, 0, 3, 1 );

			Mj[0] = x.block( j * 12 + 0 * 3, 0, 3, 1 );
			Mj[1] = x.block( j * 12 + 1 * 3, 0, 3, 1 );
			Mj[2] = x.block( j * 12 + 2 * 3, 0, 3, 1 );
			Mj[3] = x.block( j * 12 + 3 * 3, 0, 3, 1 );

			// Ri * ( gj - gi ) - ( gj - gi ) - ( tj - ti )

			Eigen::Vector3f dg = src[j] - src[i];

			Eigen::Vector3f dt = Mj[3] - Mi[3];

			Eigen::Vector3f r = ( dg[0] * Mi[0] + dg[1] * Mi[1] + dg[2] * Mi[2] ) - dg - dt;

			E += r.dot( r ) * w_smooth;

			Eigen::MatrixXf Ji = Eigen::MatrixXf::Zero( 3, 12 );
			Ji( 0, 0 + 3 * 0 ) = dg[0]; Ji( 0, 0 + 3 * 1 ) = dg[1]; Ji( 0, 0 + 3 * 2 ) = dg[2];
			Ji( 1, 1 + 3 * 0 ) = dg[0];	Ji( 1, 1 + 3 * 1 ) = dg[1];	Ji( 1, 1 + 3 * 2 ) = dg[2];
			Ji( 2, 2 + 3 * 0 ) = dg[0];	Ji( 2, 2 + 3 * 1 ) = dg[1];	Ji( 2, 2 + 3 * 2 ) = dg[2];

		//	std::cout << Ji <<std::endl;

			JtJ.block( i * 12, i * 12, 12, 12 ) += Ji.transpose() * Ji * w_smooth;
			Jtr.block( i * 12, 0, 12, 1 ) += Ji.transpose() * r * w_smooth;
		}}
	}
	printf( "E-smooth : %f\n", E );
	out_E += E;
}
bool read_text_file( const char *path, std::vector<std::vector<Eigen::Vector3f>> &poses ) {
	FILE *fp = fopen( path, "r" );
	if ( !fp ) {
		printf( "failed to read text file: %s\n", path );
		return false;
	}

	int frame = 0;
	while( fscanf( fp, "%d", &frame ) == 1 ) {
		frame -= 1;
		if ( frame != poses.size() ) {
			break;
		}
		printf( "read %d\n", frame );
		
		int subFrame = 0;
		fscanf( fp, "%d", &subFrame );

		poses.push_back( std::vector<Eigen::Vector3f>( 41 ) );
		for ( int i = 0; i < 41; i++ ) {
			float x, y, z;
			fscanf( fp, "%f", &x );
			fscanf( fp, "%f", &y );
			fscanf( fp, "%f", &z );
		//	printf( "%f %f %f\n", x, y, z);
			poses[frame][i][0] = x * 0.001f;
			poses[frame][i][1] = y * 0.001f;
			poses[frame][i][2] = z * 0.001f;
		}
	}

	fclose( fp );
	return true;
}

void test_gen_data( void ) {
	int size = 8;
	
	std::vector<Eigen::Vector3f> src( size ), dst( size );
	
	// gen transform
	Eigen::Vector3f euler;
	euler << 20, 50, -45;
	Eigen::Matrix3f rot = euler_to_mat( radians( euler ) );
	Eigen::Vector3f t;
	t << 30, 50, -10;
	std::cout << "Truth" << std::endl;
	std::cout << rot << std::endl;
	std::cout << t << std::endl;

	// gen data
	for ( int i = 0; i < size; i++ ) {
		Eigen::Vector3f euler;
		euler << 20, 50, -45;
		euler[1] += i * 5.0f;
		Eigen::Matrix3f rot = euler_to_mat( radians( euler ) );

		Eigen::Vector3f t;
		t << 30, 50, -10;

		src[i] = Eigen::Vector3f::Random();
		src[i][0] = fmod( src[i][0], 256.0f );
		src[i][1] = fmod( src[i][1], 256.0f );
		src[i][2] = fmod( src[i][2], 256.0f );
		dst[i] = rot * src[i] + t;
		//std::cout << "i << std::endl;
		//std::cout << src[i] << std::endl;
		//std::cout << dst[i] << std::endl;
	}
		
	// rigid transform
	Eigen::Matrix3f out_rot = Eigen::Matrix3f::Identity();
	Eigen::Vector3f out_t = Eigen::Vector3f::Zero();;
	{
		float rme = icp_point_to_point_svd( src, dst, out_rot, out_t );

		std::cout << "Result" << std::endl;
		std::cout << out_rot << std::endl;
		std::cout << out_t << std::endl;
		std::cout << "rme : " << rme << std::endl;
		std::cout << std::endl;
	}

	Eigen::MatrixXf JtJ = Eigen::MatrixXf::Identity( 12 * size, 12 * size ) * 1e-6f;
	Eigen::VectorXf Jtr = Eigen::VectorXf::Zero( 12 * size );
	Eigen::VectorXf x( 12 * size );
	float E = 0.0f;

	init_affine_transforms( size, x, out_rot, src, dst );

	for ( int i = 0; i < 5; i++ ) {
		JtJ = Eigen::MatrixXf::Identity( 12 * size, 12 * size ) * 1e-6f;
		Jtr = Eigen::VectorXf::Zero( 12 * size );
		E = 0.0f;
	
	//	EvalLinearSystem( size, JtJ, Jtr, x, src, dst, E );
		//std::cout << Jtr;
		x -= JtJ.fullPivHouseholderQr().solve( Jtr );
	}
}

class Timer {
public :
	clock_t begin, end;

	void Start( void ) {
		begin = clock();
	}

	void Pause( void ) {
		end = clock();
	}

	void Print( const char *id ) {
		double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
		printf( "time : %s : %f\n", id, time_spent );
	}
};

int _tmain(int argc, _TCHAR* argv[])
{
	std::vector<std::vector<Eigen::Vector3f>> poses;
		
	read_text_file( "New Session 0103.csv", poses );
	
	// rigid transform
	
	Timer timer;	
	
	Eigen::Matrix3f out_rot = Eigen::Matrix3f::Identity();
	Eigen::Vector3f out_t = Eigen::Vector3f::Zero();
	{
		timer.Start();
		float rme = icp_point_to_point_svd( poses[10], poses[600], out_rot, out_t );
		timer.Pause();
		timer.Print( "ICP" );

		std::cout << "Result" << std::endl;
		std::cout << out_rot << std::endl;
		std::cout << out_t << std::endl;
		std::cout << "rme : " << rme << std::endl;
		std::cout << std::endl;
	}

	int size = poses[0].size();

	Eigen::MatrixXf JtJ = Eigen::MatrixXf::Identity( 12 * size, 12 * size ) * 1e-6f;
	Eigen::VectorXf Jtr = Eigen::VectorXf::Zero( 12 * size );
	Eigen::VectorXf x( 12 * size );
	float E = 0.0f;

	init_affine_transforms( size, x, out_rot, poses[0], poses[600] );

	std::vector<std::vector<int>> links( poses[0].size() );
	for ( int i = 0; i < poses[0].size(); i++ ) {
		for ( int j = i + 1; j < poses[0].size(); j++ ) {
			if ( ( poses[0][i] - poses[0][j] ).norm() < 0.25f ) {
				links[i].push_back( j );
				links[j].push_back( i );
			}
		}
	}

	for ( int i = 0; i < links.size(); i++ ) {
		printf( "%d : %d\n", i, links[i].size() );
	}

	for ( int i = 0; i < 7; i++ ) {
		JtJ = Eigen::MatrixXf::Identity( 12 * size, 12 * size ) * 1e-3f;
		Jtr = Eigen::VectorXf::Zero( 12 * size );
		E = 0.0f;

		timer.Start();
		EvalRigidTerm( size, JtJ, Jtr, x, poses[0], poses[300], E );
		EvalSmoothTerm( size, JtJ, Jtr, x, poses[0], poses[300], E, links );
		timer.Pause();
		timer.Print( "Jacobian" );

		//std::cout << Jtr;
		timer.Start();
		SparseBlockSquareMatrix B;
		B.createFromDenseMatrix( JtJ, 12 );
	
		x -= pcg( B, Jtr, 100, 1e-6 );

		//x -= JtJ.fullPivHouseholderQr().solve( Jtr );
		timer.Pause();
		timer.Print( "Solve" );
	}

	return 0;
}

