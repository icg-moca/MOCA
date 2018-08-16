class SparseBlockSquareMatrix {
public :
	typedef struct {
		int col;
		int blk;
	} offset_t;
public :
	int m_gridSize, m_blockSize;

	// buffers
	std::vector<int>				m_rowScan;		// row offset
	std::vector<offset_t>			m_blockInfos;	// column index and block offset for row elements
	std::vector<float>				m_blocks;		// m_blocks buffer

	// graph of sparse matrix for creating buffers
	std::vector<std::vector<int>>	m_colLinks; 

public :
	SparseBlockSquareMatrix() : m_gridSize( 0 ), m_blockSize( 0 ) {
	}

	void free( void ) {
		m_gridSize = 0;
		m_blockSize = 0;
		m_rowScan.clear();
		m_blockInfos.clear();
		m_colLinks.clear();
	}

	// m_gridSize = mat.size / m_blockSize
	void create( int size, int blockSize ) {
		free();

		m_blockSize = blockSize;
		m_gridSize = size / blockSize;

		m_colLinks.resize( m_gridSize );

		for ( int i = 0; i < m_gridSize; i++ ) {
			insertLink( i, i );
		}
	}
	
	bool isIndexValid( int row, int col ) const {
		return ( row >= 0 && row < m_gridSize && col >= 0 && col < m_gridSize );
	}
	
	int size( void ) const {
		return m_gridSize * m_blockSize;
	}

	int blockSize( void ) const {
		return m_blockSize;
	}

	int gridSize( void ) const {
		return m_gridSize;
	}

	int calcBlockIndex( int row, int col ) const {
		assert( isIndexValid( row, col ) );
		
		int index = findLink( row, col );
		if ( index < 0 ) {
			return -1;
		}

		return m_blockInfos[m_rowScan[row] + index].blk;
	}

	// FIXME: linear search, improve
	int findLink( int row, int col ) const {
		assert( isIndexValid( row, col ) );
		const std::vector<int> &links = m_colLinks[row];
		for ( int i = 0; i < links.size(); i++ ) {
			if ( links[i] == col ) {
				return i;
			}
		}
		return -1;
	}

	void insertLink( int row, int col ) {
		assert( isIndexValid( row, col ) );
		int index = findLink( row, col );
		if ( index < 0 ) {
			m_colLinks[row].push_back( col );
		}
	}

	// only alloc buffer for upper triangle elements
	void alloc( void ) {
		// row scan buffer
		m_rowScan.resize( m_gridSize + 1 ); // add additional one for total number
		m_rowScan[0] = 0;
		for ( int i = 1; i <= m_gridSize; i++ ) {
			m_rowScan[i] = m_rowScan[i-1] + m_colLinks[i-1].size();
		}

		// block info buffer
		int m_blockInfosIndex = 0;
		int blockIndex = 0;
		m_blockInfos.resize( m_rowScan[m_gridSize] );
		for ( int i = 0; i < m_colLinks.size(); i++ ) {
			const std::vector<int> &links = m_colLinks[i];
			for ( int j = 0; j < links.size(); j++ ) {
				int col = links[j];

				offset_t &index = m_blockInfos[m_blockInfosIndex];
	
				index.col = col;
				if ( col < i ) {
					index.blk = calcBlockIndex( col, i );
				} else {
					index.blk = blockIndex;
					++blockIndex;
				}

				++m_blockInfosIndex;
			}
		}

		// block buffer
		m_blocks.resize( blockIndex * m_blockSize *  m_blockSize, 0 );

		std::cout << m_blocks.size() << std::endl;
	}

	Eigen::VectorXf diagonal( void ) const {
		Eigen::VectorXf diag = Eigen::VectorXf::Zero( size() );

		for ( int i = 0; i < m_gridSize; i++ ) {
			const float *block = m_blocks.data() + calcBlockIndex( i, i ) * m_blockSize * m_blockSize;
			for ( int k = 0; k < m_blockSize; k++ ) {
				diag[k + i * m_blockSize] = block[k + k * m_blockSize];
			}
		}

		return diag;
	}

	Eigen::VectorXf operator*( const Eigen::VectorXf &v ) const {
		Eigen::VectorXf r = Eigen::VectorXf::Zero( size() );

		for ( int row = 0; row < m_gridSize; row++ ) { // for each row
			int start = m_rowScan[row];
			int end = m_rowScan[row+1]; // column m_blocks
			for ( int i = start; i < end; i++ ) {
				offset_t info = m_blockInfos[i];
				int col = info.col;
				int blk = info.blk;
				
				const float *ptr_B = m_blocks.data() + blk * m_blockSize * m_blockSize;
				const float *ptr_v = &v[col * m_blockSize];
				float *ptr_r = &r[row * m_blockSize];

				if ( col >= row ) {					
					mad_mat_mult_v( m_blockSize, ptr_B, ptr_v, ptr_r );
				} else {
					mad_v_mult_mat( m_blockSize, ptr_B, ptr_v, ptr_r );
				}
			}
		}

		return r;
	}

	void createFromDenseMatrix( const Eigen::MatrixXf &m, int blockSize ) {
		assert( m.rows() == m.cols() );
		create( m.rows(), blockSize );

		for ( int y = 0; y < m_gridSize; y++ ) {
		for ( int x = 0; x < m_gridSize; x++ ) {
			insertLink( y, x );
		}}
		alloc();

		for ( int y = 0; y < m_gridSize; y++ ) {
		for ( int x = y; x < m_gridSize; x++ ) {
			float *block = m_blocks.data() + calcBlockIndex( y, x ) * m_blockSize * m_blockSize;

			for ( int ly = 0; ly < m_blockSize; ly++ ) {
			for ( int lx = 0; lx < m_blockSize; lx++ ) {
				block[lx + ly * m_blockSize] = m(ly + y * m_blockSize, lx + x * m_blockSize);
			}}			
		}}
	}
};