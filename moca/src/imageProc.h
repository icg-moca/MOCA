template< class T >
class Image {
public :
	EPH_Vec2i m_size;
	EPH_List< T > m_pixels;

public :
	Image() : m_size( 0 ) {
	}
	
	Image( int width, int height ) : m_size( 0 ) {
		Create( width, height );
	}

	Image( const EPH_Vec2i &size ) : m_size( 0 ) {
		Create( size );
	}

	void Fill( const T &v ) {
		m_pixels.Fill( v );
	}

	void Free( void ) {
		m_size = 0;
		m_pixels.Free();
	}

	void Create( int width, int height ) {
		Create( EPH_Vec2i( width, height ) );
	}
	
	void Create( const EPH_Vec2i &size ) {
		Free();

		m_size = size;
		m_pixels.SetNum( size.Area() );
	}

	const T &operator[]( int index ) const {
		return m_pixels[index];
	}

	const T &operator[]( const EPH_Vec2i &pos ) const {
		return m_pixels[Index(pos)];
	}

	const T &operator()( int x, int y ) const {
		return m_pixels[Index(EPH_Vec2i( x, y ))];
	}
	
	const T &operator()( const EPH_Vec2i &pos ) const {
		return m_pixels[Index(pos)];
	}
	
	T &operator[]( int index ) {
		return m_pixels[index];
	}

	T &operator[]( const EPH_Vec2i &pos ) {
		return m_pixels[Index(pos)];
	}

	T &operator()( int x, int y ) {
		return m_pixels[Index(EPH_Vec2i( x, y ))];
	}

	T &operator()( const EPH_Vec2i &pos ) {
		return m_pixels[Index(pos)];
	}

	T operator()( const EPH_Vec2 &pos ) const {
		EPH_Vec2i p = pos.To<int>();
		EPH_Vec2 f = pos - p.To<float>();

		p = p.Clamp( 1, m_size - 2 );

		T c0 = m_pixels[Index(p + EPH_Vec2i( 0, 0 ))];
		T c1 = m_pixels[Index(p + EPH_Vec2i( 1, 0 ))];
		T c2 = m_pixels[Index(p + EPH_Vec2i( 0, 1 ))];
		T c3 = m_pixels[Index(p + EPH_Vec2i( 1, 1 ))];

		T y0 = ( c0 + ( c1 - c0 ) * f.x );
		T y1 = ( c2 + ( c3 - c2 ) * f.x );

		return y0 + ( y1 - y0 ) * f.y;
	}

	T operator()(const EPH_Vec2 &pos, float threshold) const {
		EPH_Vec2i p = pos.To<int>();
		EPH_Vec2 f = pos - p.To<float>();

		p = p.Clamp(1, m_size - 2);

		T c0 = m_pixels[Index(p + EPH_Vec2i(0, 0))];
		T c1 = m_pixels[Index(p + EPH_Vec2i(1, 0))];
		T c2 = m_pixels[Index(p + EPH_Vec2i(0, 1))];
		T c3 = m_pixels[Index(p + EPH_Vec2i(1, 1))];

		if (abs(c1 - c0) > threshold || abs(c2 - c0) > threshold || abs(c3 - c0) > threshold) {
			return c0;
		}

		T y0 = (c0 + (c1 - c0) * f.x);
		T y1 = (c2 + (c3 - c2) * f.x);

		return y0 + (y1 - y0) * f.y;
	}

	bool IsEmpty( void ) const {
		return m_pixels.IsEmpty();
	}

	int Index( const EPH_Vec2i &pos ) const {
		return pos.x + pos.y * m_size.x;
	}

	bool IsPosValid( const EPH_Vec2i &p ) const {
		return ( p.x >= 0 && p.x < m_size.x && p.y >= 0 && p.y < m_size.y );
	}

	int NumPixels( void ) const {
		return m_pixels.Num();
	}

	EPH_List< T > &Pixels( void ) {
		return m_pixels;
	}

	T *Data( void ) {
		return m_pixels.Ptr();
	}

	const T *Data(void) const {
		return m_pixels.Ptr();
	}

	void ZeroBuffer( void ) {
		m_pixels.ZeroBuffer();
	}

	int BufferSize( void ) const {
		return m_pixels.BufferSize();
	}

	EPH_Vec2i Size( void ) const {
		return m_size;
	}

	int Width() const {
		return m_size.x;
	}

	int Height() const {
		return m_size.y;
	}
};

EPH_Vec2 Distort( const EPH_Vec2 &p, const float k[] ) {
	float k1 = k[0];
	float k2 = k[1];
	float p1 = k[2];
	float p2 = k[3];
	float k3 = k[4];

	float r2 = p.x * p.x + p.y * p.y;
	float r4 = r2 * r2;
	float r6 = r4 * r2;

	float s = ( 1 + k1 * r2 + k2 * r4 + k3 * r6 );

	float x = p.x * s + 2 * p1 * p.x * p.y + p2 * ( r2 + 2 * p.x * p.x );
	float y = p.y * s + 2 * p2 * p.x * p.y + p1 * ( r2 + 2 * p.y * p.y );

	return EPH_Vec2( x, y );
}

EPH_Vec2 IntrInvTransform( const EPH_Vec4 &K, const EPH_Vec2 &v ) {
	return EPH_Vec2(v.x - K.z, v.y - K.w) / EPH_Vec2(K.x, K.y);
}

EPH_Vec2 IntrTransform(const EPH_Vec4 &K, const EPH_Vec2 &v) {
	return v * EPH_Vec2(K.x, K.y) + EPH_Vec2(K.z, K.w);
}
template< class T >
void UndistortImage( const Image<T> &src, Image<T> &dst, const EPH_Vec4 &K, float dist[] ) {
	dst.ZeroBuffer();
	
	for ( int y = 0; y < dst.m_size.y; y++ ) {
	for ( int x = 0; x < dst.m_size.x; x++ ) {
		EPH_Vec2 p = IntrInvTransform( K, EPH_Vec2( x, y ) );

		EPH_Vec2 q = IntrTransform( K, Distort( p, dist ) );

		if ( src.IsPosValid( q.To<int>() ) ) {
			dst(x, y) = src(q);
		}
	}}
}

template< class T >
void ReprojectRGBImage( const Image<T> &src, Image<T> &dst, const EPH_Vec4 &K_src, const EPH_Mat4 &M_src, const EPH_Vec4 &K_dst, const EPH_Mat4 &M_dst ) {
	dst.ZeroBuffer();
	
	EPH_Mat3 M = Block( M_src ).Transpose() * Block( M_dst );
	
	for ( int y = 0; y < dst.m_size.y; y++ ) {
	for ( int x = 0; x < dst.m_size.x; x++ ) {
		EPH_Vec2 q = ReprojectRay( EPH_Vec2( x, y ), K_dst, K_src, M ); // dst -> src
		if ( src.IsPosValid( q.To<int>() ) ) {
			dst( x, y ) = src( q );
		}
	}}
}

void Intensity( const Image<EPH_Vec4> &image, Image<float> &I ) {
	I.Create( image.m_size );
	for ( int i = 0; i < image.NumPixels(); i++ ) {
		if ( image[i][3] <= 0 ) {
			I[i] = 0;
		} else {
			// I[i] = image[i].r / ( 255.0f );
			I[i] = ( image[i][0] + image[i][1] + image[i][2] ) / ( 3.0f );
			if ( I[i] < 0 ) {
				image[i].Print();
			}
		}
	}
}

void Solbel3x3( const Image<float> &I, Image<float> &edge_h, Image<float> &edge_v ) {
	edge_h.Create( I.Size() );
	edge_v.Create( I.Size() );
	
	for ( int y = 1; y < I.Size().y - 1; y++ ) {
	for ( int x = 1; x < I.Size().x - 1; x++ ) {
		int index = I.Index( EPH_Vec2i( x, y ) );

		float v[3][3];

		for ( int py = -1; py <= +1; py++ ) {
			for ( int px = -1; px <= +1; px++ ) {
				v[py + 1][px + 1] = I[index + px + py * I.Size().x];
			}
		}

		edge_h[index] = ( v[0][2] - v[0][0] ) + ( v[1][2] - v[1][0] ) * 2 + ( v[2][2] - v[2][0] );
		edge_v[index] = ( v[2][0] - v[0][0] ) + ( v[2][1] - v[0][1] ) * 2 + ( v[2][2] - v[0][2] );
	}}
}

EPH_Vec2 MinMax( const Image<float> &I ) {
	float minI = +1000000;
	float maxI = -1000000;
	for ( int i = 0; i < I.NumPixels(); i++ ) {
		if ( I[i] < minI ) {
			minI = I[i];
		} else if ( I[i] > maxI ) {
			maxI = I[i];
		}
	}
	return EPH_Vec2( minI, maxI );
}

void Remap( const Image<float> &I, Image<unsigned char> &gray ) {
	gray.Create( I.Size() );
	
	EPH_Vec2 range = MinMax( I );
	range.Print();
	for ( int i = 0; i < I.NumPixels(); i++ ) {
		gray[i] = ( I[i] - range[0] ) / ( range[1] - range[0] ) * 255;
	}
}

void SavePGM( Image<float> &I, const EPH_String &path ) {
	path.Print();
	
	Image<unsigned char> gray;
	Remap( I, gray );
	
	std::ofstream file( path.Ptr(), std::ios::out | std::ios::binary);

	file << "P5\n" << I.m_size.x << " " << I.m_size.y << "\n" << UCHAR_MAX << "\n";
	file.write( (char *)gray.Data(), gray.BufferSize() );
}

void SavePGM( Image<unsigned short> &I, const EPH_String &path ) {
	Image<unsigned char> gray( I.Size() );
	for ( int i = 0; i < I.NumPixels(); i++ ) {
		gray[i] = I[i] / 4;
	}
	
	std::ofstream file( path.Ptr(), std::ios::out | std::ios::binary);

	file << "P5\n" << I.m_size.x << " " << I.m_size.y << "\n" << UCHAR_MAX << "\n";
	file.write( (char *)gray.Data(), gray.BufferSize() );
}