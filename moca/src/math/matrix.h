#ifndef matrix_h
#define matrix_h

const float PI = 3.1415926535897932384626433832795;

template< class T >
T radians( const T &degree ) {
	return degree * ( PI / 180.0f );
}

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

// R(x)
Eigen::Matrix3f rot_x_mat( float rad ) {
	float c = cos( rad );
	float s = sin( rad );

	Eigen::Matrix3f m;
	m <<
		1, 0, 0,
		0, c, -s,
		0, s, c
		;

	return m;
}

// R(y)
Eigen::Matrix3f rot_y_mat( float rad ) {
	float c = cos( rad );
	float s = sin( rad );

	Eigen::Matrix3f m;
	m <<
		c, 0, s,
		0, 1, 0,
		-s, 0, c
		;

	return m;
}

// R(z)
Eigen::Matrix3f rot_z_mat( float rad ) {
	float c = cos( rad );
	float s = sin( rad );

	Eigen::Matrix3f m;
	m <<
		c, -s, 0,
		s, c, 0,
		0, 0, 1
		;

	return m;
}

// R(z) * R(y) * R(x)
Eigen::Matrix3f euler_to_mat( const Eigen::Vector3f &euler ) {
	return rot_z_mat( euler.z() ) * rot_y_mat( euler.y() ) * rot_x_mat( euler.x() );
}

Eigen::Matrix3f euler_to_skew( const Eigen::Vector3f &euler ) {
	float x = euler.x();
	float y = euler.y();
	float z = euler.z();

	Eigen::Matrix3f r;
	r <<
		1.0f, -z, +y,
		+z, 1.0f, -x,
		-y, +x, 1.0f;

	return r;
}

// skew( w ) * v = cross( w, v )
// Infinitesimal rotations = skew( w ) + I
Eigen::Matrix3f skew_mat( const Eigen::Vector3f &v ) {
	float x = v.x();
	float y = v.y();
	float z = v.z();
	
	Eigen::Matrix3f sk;
	sk <<
		 0, -z, +y,
		+z,  0, -x,
		-y, +x,  0
	;

	return sk;
}

// E(v)
Eigen::Vector3f mean( const std::vector< Eigen::Vector3f > &points ) {
	if ( points.empty() ) {
		return Eigen::Vector3f::Zero();
	}
	
	Eigen::Vector3f e;
	e.setZero();
	for ( int i = 0; i < points.size(); i++ ) {
		e += points[i];
	}
	e /= (float)( points.size() );

	return e;
}

// sqrt( E( | dst - r * src - t |^2 ) )
float rme( const std::vector< Eigen::Vector3f > &src, const std::vector< Eigen::Vector3f > &dst, const Eigen::Matrix3f &r, const Eigen::Vector3f &t ) {
	if ( src.empty() ) {
		return 0.0f;
	}
	
	float e = 0.0f;
	for ( int i = 0; i < src.size(); i++ ) {
		Eigen::Vector3f d = dst[i] - r * src[i] - t;
		e += d.dot( d );
	}
	e /= (float)( src.size() );

	return sqrt( e );
}


#endif