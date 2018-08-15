
class Mat12 {
public :
	Eigen::VectorXf	v[12];

public :
	Mat12() {
		for ( int i = 0; i < 12; i++ ) {
			v[i] = Eigen::VectorXf::Zero( 12 );
		}
	}

	Mat12 &operator+=( const Mat12 &mat ) {
		for ( int i = 0; i < 12; i++ ) {
			v[i] += mat.v[i];
		}

		return *this;
	}

	Eigen::VectorXf InverseDiagonal( void ) const {
		Eigen::VectorXf u( 12 );

		for ( int i = 0; i < 12; i++ ) {
			float t = v[i][i];
			if ( t < 1e-6f ) {
				t = 1.0f;
			} else {
				t = 1.0f / t;
			}
			u[i] = t;
		}

		return u;
	}

	void Identity( float s = 1.0f ) {
		Zero();
		for ( int row = 0; row < 12; row++ ) {
			v[row][row] = s;
		}
	}

	void Zero( void ) {
		for ( int i = 0; i < 12; i++ ) {
			v[i].setZero();
		}
	}

	Mat12 &VecProduct( const Eigen::VectorXf &a, const Eigen::VectorXf &b ) {
		for ( int i = 0; i < 12; i++ ) {
			v[i] = a * b[i];
		}

		return *this;
	}

	Mat12 &Mat4( const Eigen::Matrix4f &M ) {
		for ( int row = 0; row < 4; row++ ) {
			for ( int col = 0; col < 4; col++ ) {
				v[col * 3 + 0][row * 3 + 0] = M(row, col);
				v[col * 3 + 1][row * 3 + 1] = M(row, col);
				v[col * 3 + 2][row * 3 + 2] = M(row, col);
			}
		}

		return *this;
	}

	Mat12 &MatX( const Eigen::MatrixXf &M ) {
		for ( int row = 0; row < 12; row++ ) {
			for ( int col = 0; col < 12; col++ ) {
				v[col][row] = M(row, col);
				v[col][row] = M(row, col);
				v[col][row] = M(row, col);
			}
		}

		return *this;
	}


	Eigen::VectorXf operator*( const Eigen::VectorXf &a ) {
		Eigen::VectorXf u( 12 );
		u.setZero();

		for ( int i = 0; i < 12; i++ ) {
			u += v[i] * a[i];
		}

		return u;
	}

	bool IsZero( void ) const {
		for ( int i = 0; i < 12; i++ ) {
			for ( int j = 0; j < 12; j++ ) {
				if ( v[i][j] != 0 ) {
					return false;
				}
			}
		}
		return true;
	}
};

class SparseBlock12 {
public :
	typedef EPH_SharedPtr< Mat12 > mat_t;

	int size;
	int count;

	// FIXME : free
	EPH_List< mat_t > blocks;

public :
	void Create( int size ) {
		this->size = size;
		this->count = 0;

		blocks.Free();
		blocks.SetNum( size * size ); // FIXME : symmetry
	}

	void Identity( float s = 1.0f ) {
		for ( int i = 0; i < size; i++ ) {
			for ( int j = 0; j < size; j++ ) {
				mat_t &m = blocks[i * size + j];
				if ( i == j ) {
					if ( !m ) {
						m = new Mat12;
						count++;
					}
					m->Identity( s );
				} else {
					if ( m ) {
						m->Zero();
					}
				}
			}
		}
	}

	void AddMat( int row, int col, const Mat12 &mat ) {
		if ( row > col ) {
		//	return;
		}
		
		int index = row * size + col;

		if ( !blocks[index] ) {
			blocks[index] = new Mat12;
			*blocks[index] = mat;
			count++;
		} else {
			*blocks[index] += mat;
		}

		//std::cout << row << " " << col << std::endl << mat << std::endl;
	}

	void AddMat( int row, int col, const Eigen::MatrixXf &mat ) {
		if ( row > col ) {
		//	return;
		}
		
		int index = row * size + col;

		if ( !blocks[index] ) {
			blocks[index] = new Mat12;
			*blocks[index] = Mat12().MatX( mat );
			count++;
		} else {
			*blocks[index] += Mat12().MatX( mat );
		}

		//std::cout << row << " " << col << std::endl << mat << std::endl;
	}

	EPH_SharedPtr< Mat12 > &operator()( int row, int col ) {
		return blocks[row * size + col];
	}

	void InverseDiagonal( Eigen::VectorXf &D ) {
		D = Eigen::VectorXf( size * 12 );
		D.setZero();
		for ( int i = 0; i < size; i++ ) {
			mat_t &M = (*this)( i, i );
			if ( !M ) {
				continue;
			}

			D.block( i * 12, 0, 12, 1 ) = M->InverseDiagonal();
		}
	}

#if 0
	void MultVec( const Eigen::VectorXf &src, Eigen::VectorXf &dst ) {
		printf( "MultVec : %d : %d\n", size, count );
		dst = Eigen::VectorXf( size * 12 );
		dst.setZero();
		for ( int row = 0; row < size; row++ ) {
			Eigen::VectorXf sum( 12 );
			sum.setZero();
			for ( int col = 0; col < size; col++ ) {
				mat_t M = (*this)( row, col );
				if ( !M ) {
					continue;
				}
				sum += (*M) * src.block(col * 12, 0, 12, 1);
			}
			dst.block( row * 12, 0, 12, 1 ) = sum;
		}
	}
#endif

	void Flatten( EPH_List<float> &elems, EPH_List<int> &indices_row, EPH_List<int> &indices_col ) {
		elems.Empty();
		elems.EnsureSize( count * 12 * 12 );
		
		indices_row.Empty();
		indices_row.EnsureSize( size + 1 );

		indices_col.Empty();
		indices_col.EnsureSize( count );

		int maxNumCols = 0;
		int minNumCols = 256;

		for ( int row = 0; row < size; row++ ) {
			indices_row.Append( indices_col.Num() );
			int count = 0;
			for ( int col = 0; col < size; col++ ) {
				mat_t M = (*this)( row, col );
				if ( !M ) {
					continue;
				}

				indices_col.Append( col );
				count++;
				for ( int bcol = 0; bcol < 12; bcol++ ) {
					for ( int brow = 0; brow < 12; brow++ ) {
						elems.Append( M->v[bcol][brow] );
					}
				}
			}
			if ( count > maxNumCols ) {
				maxNumCols = count;
			}
			if ( count < minNumCols ) {
				minNumCols = count;
			}
		}

		indices_row.Append( indices_col.Num() );

		printf( "Flatten %d : %d : %d : %d\n", count, indices_row.Num() - 1, maxNumCols, minNumCols );
	}
};
