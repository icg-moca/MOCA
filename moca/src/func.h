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

EPH_Mat4 convert_6d_to_mat( const Eigen::VectorXf &x ) {
	Eigen::Vector3f euler;
	euler << x[0], x[1], x[2];
	Eigen::Matrix3f r = euler_to_mat( euler );

	// translation
	Eigen::Vector3f t;
	t << x[3], x[4], x[5];
		
	EPH_Mat4 T;
	T[0][0] = r(0, 0); T[1][0] = r(1, 0); T[2][0] = r(2, 0); T[3][0] = 0;
	T[0][1] = r(0, 1); T[1][1] = r(1, 1); T[2][1] = r(2, 1); T[3][1] = 0;
	T[0][2] = r(0, 2); T[1][2] = r(1, 2); T[2][2] = r(2, 2); T[3][2] = 0;
	T[0][3] = t(0)   ; T[1][3] = t(1)   ; T[2][3] = t(2);	 T[3][3] = 1;
	return T;
}



Eigen::Vector3f vec3_to_eigen( const EPH_Vec3 &v ) {
	return Eigen::Vector3f( v.x, v.y, v.z );
}
