template<int blockSize>
class SparseBlockSquareMatrix {
public :
	typedef struct {
		float e[blockSize * blockSize];
	} block_t;

	typedef struct {
		int col;
		int blk;
	} offset_t;
public :
	int gridSize;

	// buffers
	std::vector<int>				rowScan;		// row offset
	std::vector<offset_t>			blockInfos;		// column index and block offset for row elements
	std::vector<block_t>			blocks;			// blocks buffer

	// graph of sparse matrix for creating buffers
	std::vector<std::vector<int>>	colLinks; 

public :
	void free( void ) {
		gridSize = 0;
		rowScan.clear();
		blockInfos.clear();
		colLinks.clear();
	}

	// gridSize = mat.size / blockSize
	void create( int gridSize ) {
		free();

		this->gridSize = gridSize;

		colLinks.resize( gridSize );

		for ( int i = 0; i < gridSize; i++ ) {
			insertLink( i, i );
		}
	}
	
	bool isIndexValid( int row, int col ) const {
		return ( row >= 0 && row < gridSize && col >= 0 && col < gridSize );
	}
	
	int size( void ) const {
		return gridSize * blockSize;
	}

	int calcBlockIndex( int row, int col ) const {
		assert( isIndexValid( row, col ) );
		
		int index = findLink( row, col );
		if ( index < 0 ) {
			return -1;
		}

		return blockInfos[rowScan[row] + index].blk;
	}

	// FIXME: linear search, improve
	int findLink( int row, int col ) const {
		assert( isIndexValid( row, col ) );
		const std::vector<int> &links = colLinks[row];
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
			colLinks[row].push_back( col );
		}
	}

	// only alloc buffer for upper triangle elements
	void alloc( void ) {
		// row scan buffer
		rowScan.resize( gridSize + 1 ); // add additional one for total number
		rowScan[0] = 0;
		for ( int i = 1; i <= gridSize; i++ ) {
			rowScan[i] = rowScan[i-1] + colLinks[i-1].size();
		}

		// block info buffer
		int blockInfosIndex = 0;
		int blockIndex = 0;
		blockInfos.resize( rowScan[gridSize] );
		for ( int i = 0; i < colLinks.size(); i++ ) {
			const std::vector<int> &links = colLinks[i];
			for ( int j = 0; j < links.size(); j++ ) {
				int col = links[j];

				offset_t &index = blockInfos[blockInfosIndex];
	
				index.col = col;
				if ( col < i ) {
					index.blk = calcBlockIndex( col, i );
				} else {
					index.blk = blockIndex;
					++blockIndex;
				}

				++blockInfosIndex;
			}
		}

		// block buffer
		blocks.resize( blockIndex );
		memset( blocks.data(), 0, blocks.size() * sizeof( block_t ) );

		std::cout << blocks.size() << std::endl;
	}

	Eigen::VectorXf diagonal( void ) const {
		Eigen::VectorXf diag = Eigen::VectorXf::Zero( size() );

		for ( int i = 0; i < gridSize; i++ ) {
			const block_t &block = blocks[calcBlockIndex( i, i )];
			for ( int k = 0; k < blockSize; k++ ) {
				diag[k + i * blockSize] = block.e[k + k * blockSize];
			}
		}

		return diag;
	}



	Eigen::VectorXf operator*( const Eigen::VectorXf &v ) const {
		Eigen::VectorXf r = Eigen::VectorXf::Zero( size() );

		for ( int row = 0; row < gridSize; row++ ) { // for each row
			int start = rowScan[row];
			int end = rowScan[row+1]; // column blocks
			for ( int i = start; i < end; i++ ) {
				offset_t info = blockInfos[i];
				int col = info.col;
				int blk = info.blk;
				
				const float *ptr_B = blocks[blk].e;
				const float *ptr_v = &v[col * blockSize];
				float *ptr_r = &r[row * blockSize];

				if ( col >= row ) {					
					mad_mat_mult_v( blockSize, ptr_B, ptr_v, ptr_r );
				} else {
					mad_v_mult_mat( blockSize, ptr_B, ptr_v, ptr_r );
				}
			}
		}

		return r;
	}

	SparseBlockSquareMatrix<blockSize> &operator=( const Eigen::MatrixXf &m ) {
		assert( m.rows() == m.cols() );
		create( m.rows() / blockSize );

		for ( int y = 0; y < gridSize; y++ ) {
		for ( int x = 0; x < gridSize; x++ ) {
			insertLink( y, x );
		}}
		alloc();

		for ( int y = 0; y < gridSize; y++ ) {
		for ( int x = y; x < gridSize; x++ ) {
			block_t &block = blocks[calcBlockIndex( y, x )];

			for ( int ly = 0; ly < blockSize; ly++ ) {
			for ( int lx = 0; lx < blockSize; lx++ ) {
				block.e[lx + ly * blockSize] = m(ly + y * blockSize, lx + x * blockSize);
			}}			
		}}
		return *this;
	}
};