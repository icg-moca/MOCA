
void CalcWeights( const EPH_Vec3 &v, float w[8] ) {
	// x
	w[0] = w[2] = w[4] = w[6] = 1.0 - v.x;

	w[1] = w[3] = w[5] = w[7] = v.x;

	// y
	w[0] *= 1.0 - v.y;
	w[1] *= 1.0 - v.y;
	w[4] *= 1.0 - v.y;
	w[5] *= 1.0 - v.y;

	w[2] *= v.y;
	w[3] *= v.y;
	w[6] *= v.y;
	w[7] *= v.y;

	// z
	w[0] *= 1.0 - v.z;
	w[1] *= 1.0 - v.z;
	w[2] *= 1.0 - v.z;
	w[3] *= 1.0 - v.z;

	w[4] *= v.z;
	w[5] *= v.z;
	w[6] *= v.z;
	w[7] *= v.z;
}

void IdentityX( Eigen::VectorXf &X, int numJoints ) {
	X = Eigen::VectorXf::Zero( numJoints * 12 );
	for ( int i = 0; i < X.rows(); i += 12 ) {
		X[ i + ( 0 * 3 + 0 ) ] = 1;
		X[ i + ( 1 * 3 + 1 ) ] = 1;
		X[ i + ( 2 * 3 + 2 ) ] = 1;
	}
}

// E_smooth : Ri * ( gj - gi ) + ti + gi - ( gj + tj )
// Ri * ( gj - gi ) - ( gj - gi ) - ( tj - ti )
void CalcSmoothTerm(
	SparseBlock12 &JtJ, Eigen::VectorXf &Jtr, Eigen::VectorXf &X,
	const Eigen::Vector3f &vi, const Eigen::Vector3f &vj,
	int i, int j,
	float w
) {
	int col_i = i * 12;

	Eigen::Matrix3f Ri;
	Ri <<
		X[col_i + 0], X[col_i + 3], X[col_i + 6],
		X[col_i + 1], X[col_i + 4], X[col_i + 7],
		X[col_i + 2], X[col_i + 5], X[col_i + 8]
	;

	Eigen::Vector3f ti = X.block( col_i + 9, 0, 3, 1 );

	int col_j = j * 12;

	Eigen::Vector3f tj = X.block( col_j + 9, 0, 3, 1 );

	Eigen::Vector3f dg = vj - vi;
	Eigen::Vector3f dt = tj - ti;

	Eigen::Vector4f a( dg[0], dg[1], dg[2], +1.0f ); // i
	Eigen::Vector4f b( 0    , 0    , 0    , -1.0f ); // j

	// JtJ
	JtJ.AddMat( i, i, Mat12().Mat4( a * a.transpose() * w ) );
	JtJ.AddMat( j, j, Mat12().Mat4( b * b.transpose() * w ) );
	JtJ.AddMat( i, j, Mat12().Mat4( a * b.transpose() * w ) );
	JtJ.AddMat( j, i, Mat12().Mat4( b * a.transpose() * w ) );

	Eigen::Vector3f d = ( Ri * dg - dg - dt );

	// Jtr
	Jtr.block( col_i + 0 * 3, 0, 3, 1 ) += d * ( a[0] * w );
	Jtr.block( col_i + 1 * 3, 0, 3, 1 ) += d * ( a[1] * w );
	Jtr.block( col_i + 2 * 3, 0, 3, 1 ) += d * ( a[2] * w );
	Jtr.block( col_i + 3 * 3, 0, 3, 1 ) += d * ( a[3] * w );

	Jtr.block( col_j + 3 * 3, 0, 3, 1 ) += d * ( b[3] * w );
}

// dot( a, b ) = 0
void CalcRotTerm0(
	SparseBlock12 &JtJ, Eigen::VectorXf &Jtr,
	Eigen::Vector3f &a, Eigen::Vector3f &b,
	int k, int i, int j,
	float w
) {
	Eigen::VectorXf u = Eigen::VectorXf::Zero( 12 );

	u.block( i * 3, 0, 3, 1 ) = a;
	u.block( j * 3, 0, 3, 1 ) = b;

	JtJ.AddMat( k, k, ( u * u.transpose() * w ) );

	Jtr.block( k * 12, 0, 12, 1 ) += u * ( a.dot( b ) * w );
}

// dot( a, a ) = 1
void CalcRotTerm1(
	SparseBlock12 &JtJ, Eigen::VectorXf &Jtr,
	Eigen::Vector3f &a,
	int k, int i,
	float w
) {
	Eigen::VectorXf u = Eigen::VectorXf::Zero( 12 );

	u.block( i * 3, 0, 3, 1 ) = a + a;

	JtJ.AddMat( k, k, ( u * u.transpose() * w ) );

	Jtr.block( k * 12, 0, 12, 1 ) += u * ( ( a.dot( a ) - 1 ) * w );
}
