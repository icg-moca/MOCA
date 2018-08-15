	void GenColorMap( const Image< unsigned short > &depthMap, const Image< EPH_RGBA8 > &srcColorMap,  Image< EPH_RGBA8 > &dstColorMap, const Sensor &depthSensor, const Sensor &colorSensor ) {
		for ( int y = 0; y < depthMap.Height(); y++ ) {
			for ( int x = 0; x < depthMap.Width(); x++ ) {
				int index = x + y * depthMap.Width();

				EPH_Vec2 u = colorSensor.Project( inv_rigid_transform( colorSensor.extr ) * depthSensor.extr * depthSensor.Unproject( EPH_Vec2( x, y ) + 0.5f, depthMap[index] * 0.001f ).Cat(1) );

				if ( srcColorMap.IsPosValid( u.To<int>() ) ) {
					dstColorMap[index] = srcColorMap( srcColorMap.Size() - 1 - u.To<int>() );
					EPH_Math::Swap( dstColorMap[index].r, dstColorMap[index].b );
				} else {
					dstColorMap[index] = EPH_RGBA8( 0, 0, 0, 255 );
				}
			}
		}		
	}
	
	void GenEdgeMap( Image< unsigned short > &depthMap, Image< float > &dist ) {
		dist.Create( depthMap.Size() );
		dist.Fill( 0 );

		for ( int y = 1; y < 424 - 1; y++ ) {
			for ( int x = 1; x < 512 - 1; x++ ) {
				int index = x + y * 512;

				int d = depthMap[index];
				if ( !d ) {
					continue;
				}

#if 0
				EPH_Vec4 v( depthMap[index - 1], depthMap[index + 1], depthMap[index - 512] , depthMap[index + 512] );

				if ( v.MaxEntry() - v.MinEntry() > 50 ) {
					dist[index] = 1;
				}
#else				
				float ddx = abs( (int)depthMap[index + 1] + (int)depthMap[index - 1] - d - d ) / 2;
				float ddy =	abs( (int)depthMap[index + 512] + (int)depthMap[index - 512] - d - d ) / 2;
				
				if (
					ddx * ddx + ddy * ddy > 50 * 50
				) {
					dist[index] = 1;
				}
#endif
			}
		}
	}
	
	void GenDistanceMap( Image< unsigned short > &depthMap, Image< float > &dist ) {
		dist.Create( depthMap.Size() );
		dist.Fill( 512 * 512 + 512 * 512 );

		for ( int y = 1; y < 424 - 1; y++ ) {
			for ( int x = 1; x < 512 - 1; x++ ) {
				int index = x + y * 512;

				int d = depthMap[index];
				if ( !d ) {
					continue;
				}

#if 0
				EPH_Vec4 v( depthMap[index - 1], depthMap[index + 1], depthMap[index - 512] , depthMap[index + 512] );

				if ( v.MaxEntry() - v.MinEntry() > 50 ) {
					dist[index] = 0;
				}
#else				
				float ddx = abs( (int)depthMap[index + 1] + (int)depthMap[index - 1] - d - d ) / 2;
				float ddy =	abs( (int)depthMap[index + 512] + (int)depthMap[index - 512] - d - d ) / 2;
				
				if (
					ddx * ddx + ddy * ddy > 50 * 50
				) {
					dist[index] = 0;
				}
#endif
			}
		}

		EPH_List< float > temp( depthMap.NumPixels() );
			
		for ( int y = 0; y < 424; y++ ) {
			int index = y * 512;

			int a = 0, b = -1;

			while ( a < 512 ) {
				for ( ; a < 512; a++ ) {
					if ( dist[index + a] > 0 ) {
						break;
					}
				}

				if ( a >= 512 ) {
					break;
				}

				for ( b = a + 1; b < 512; b++ ) {
					if ( dist[index + b] <= 0 ) {
						break;
					}
				}

				// a -> b - 1
				for ( int i = a; i <= b - 1; i++ ) {
					int d = EPH_Math::Min( i - a + 1, b - i );
					temp[index + i] = d * d;
				//	printf( "%d %d %d\n", i, y, d );
				}

				a = b + 1;
			}
		}

		for ( int x = 0; x < 512; x++ ) {
			int index = x;

			int a = 0, b = -1;

			while ( a < 424 ) {
				for ( ; a < 424; a++ ) {
					if ( dist[index + a * 512] > 0 ) {
						break;
					}
				}

				if ( a >= 424 ) {
					break;
				}

				for ( b = a + 1; b < 424; b++ ) {
					if ( dist[index + b * 512] <= 0 ) {
						break;
					}
				}

				// a -> b - 1
				for ( int i = a; i <= b - 1; i++ ) {
					int d = EPH_Math::Min( i - a + 1, b - i );
					d *= d;

					for ( int k = a; k <= b - 1; k++ ) {
						float dd = ( k - i ) * ( k - i ) + temp[index + k * 512];
						if ( dd < d ) {
							d = dd;
						}
					}

					dist[index + i * 512] = d;
				}

				a = b + 1;
			}
		}

		for ( int i = 0; i < dist.NumPixels(); i++ ) {
			dist[i] = sqrt( dist[i] );
		}
	}

	void SaveDistMap( EPH_List< float > &dist, int id ) {
		EPH_List< EPH_RGBA8 > color( dist.Num() );

		for ( int i = 0; i < color.Num(); i++ ) {
			color[i].Set( pow( EPH_Math::Min( dist[i] / 50.0f, 1.0f ), 1.0f ) * 255.0f );
		}

		EPH_PNG png;
		png.SaveImage( 512, 424, EPH_PIXEL_FORMAT_RGBA8, color.Ptr(), EPH_String().SetWithFormat( "dist-%d.png", id ) );
	}