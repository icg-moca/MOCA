//
//  cv_op.h
//  Moc
//
//  Created by Wei Li on 6/3/15.
//

#ifndef Moc_cv_op_h
#define Moc_cv_op_h

template< class T >
cv::Point3_<T> operator/( const cv::Point3_<T> &a, float s ) {
	s = 1.0f / s;
	return cv::Point3_<T>( a.x * s, a.y * s, a.z * s );
}

template< class T >
cv::Point3_<T> operator*( const cv::Mat &m, const cv::Point3_<T> &v ) {
	cv::Point3_<T> t;
	t.x = m.at<T>( 0, 0 ) * v.x + m.at<T>( 0, 1 ) * v.y + m.at<T>( 0, 2 ) * v.z;
	t.y = m.at<T>( 1, 0 ) * v.x + m.at<T>( 1, 1 ) * v.y + m.at<T>( 1, 2 ) * v.z;
	t.z = m.at<T>( 2, 0 ) * v.x + m.at<T>( 2, 1 ) * v.y + m.at<T>( 2, 2 ) * v.z;

	if ( m.size().width == 4 ) {
		t.x += m.at<T>( 0, 3 );
		t.y += m.at<T>( 1, 3 );
		t.z += m.at<T>( 2, 3 );
	}

	return t;
}

template< class T >
cv::Point3_<T> Sum( const vector< cv::Point3_<T> > &v ) {
	cv::Point3_<T> sum( 0, 0, 0 );
	for ( int i = 0; i < v.size(); i++ ) {
		sum += v[ i ];
	}
	return sum;
}

template< class T >
cv::Mat Outer( const cv::Point3_<T> &a, const cv::Point3_<T> &b ) {
	cv::Mat t = cv::Mat_<T>( 3, 3 );

	const T *aa = &a.x;
	const T *bb = &b.x;
	for ( int i = 0; i < 3; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			t.at<T>( i, j ) = aa[ i ] * bb[ j ];
		}
	}

	return t;
}

template< class T >
T Length( const cv::Point3_<T> &v ) {
	return sqrt( v.x * v.x + v.y * v.y + v.z * v.z );
}

// rotate firstly, translate next
template< class T >
cv::Mat TranslateRotate( const cv::Point3_<T> &t, const cv::Mat &r ) {
	cv::Mat m = cv::Mat_<T>( 4, 4 );

	for ( int i = 0; i < 3; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			m.at<T>( i, j ) = r.at<T>( i, j );
		}
	}

	m.at<T>( 0, 3 ) = t.x;
	m.at<T>( 1, 3 ) = t.y;
	m.at<T>( 2, 3 ) = t.z;

	m.at<T>( 3, 0 ) = 0;
	m.at<T>( 3, 1 ) = 0;
	m.at<T>( 3, 2 ) = 0;
	m.at<T>( 3, 3 ) = 1;

	return m;
}

template< class T >
cv::Mat QuatToMat( T x, T y, T z, T w ) {
	T x2 = x + x;
	T y2 = y + y;
	T z2 = z + z;

	T xx2 = x * x2;
	T yy2 = y * y2;
	T zz2 = z * z2;

	T xy2 = x * y2;
	T yz2 = y * z2;
	T zx2 = z * x2;

	T wx2 = w * x2;
	T wy2 = w * y2;
	T wz2 = w * z2;

	cv::Mat m = cv::Mat_< T >( 3, 3 );

	m.at<T>( 0, 0 ) = 1 - yy2 - zz2;
	m.at<T>( 0, 1 ) =     xy2 - wz2;
	m.at<T>( 0, 2 ) =     zx2 + wy2;

	m.at<T>( 1, 0 ) =     xy2 + wz2;
	m.at<T>( 1, 1 ) = 1 - zz2 - xx2;
	m.at<T>( 1, 2 ) =     yz2 - wx2;

	m.at<T>( 2, 0 ) =     zx2 - wy2;
	m.at<T>( 2, 1 ) =     yz2 + wx2;
	m.at<T>( 2, 2 ) = 1 - xx2 - yy2;

	return m;
}

#endif // Moc_cv_op_h
