#ifndef misc_h
#define misc_h

static void mad_mat_mult_v( int size, const float *B, const float *v, float *r ) {
	for ( int row = 0; row < size; row++ ) {
		float sum = 0.0f;
		for ( int col = 0; col < size; col++ ) {
			sum += B[col + row * size] * v[col];
		}
		r[row] += sum; // FIXME : +=
	}
}

static void mad_v_mult_mat( int size, const float *B, const float *v, float *r ) {
	for ( int row = 0; row < size; row++ ) {
		float sum = 0.0f;
		for ( int col = 0; col < size; col++ ) {
			sum += B[row + col * size] * v[col];
		}
		r[row] += sum; // FIXME : +=
	}
}

void random_positive_definite_matrix( int size, Eigen::MatrixXf &A ) {
	// A : positive definite matrix
	A = Eigen::MatrixXf::Random( size, size );
	A = A * A.transpose();
	A += Eigen::MatrixXf::Identity( size, size ) * size;
	//std::cout << A << std::endl;
}

float rme( const Eigen::VectorXf &a, const Eigen::VectorXf &b ) {
	return sqrtf( ( a - b ).dot( a - b ) / a.rows() );
}

#endif