
int Ceil( int v, int mod ) {
	v += mod - 1;
	return v - v % mod;
}

int Ceil( int v, int mod, int &num ) {
	num = ( v + mod - 1 ) / mod;
	return mod * num;
}

class PCG_GPU {
public :
	CL::Context context;
	
	CL::Program program;
	CL::CommandQueue cq;

	// mem
	CL::Mem mem_M;
	CL::Mem mem_Mr;
	CL::Mem mem_rMr;

	CL::Mem mem_A, mem_A_rows, mem_A_cols;
	CL::Mem mem_Ap;
	CL::Mem mem_pAp;

	CL::Mem mem_x;
	CL::Mem mem_p;
	CL::Mem mem_r;

	// kernel
	CL::Kernel kn_zero_v;
	CL::Kernel kn_mul_v_v;
	CL::Kernel kn_mad_v_v_s;
	CL::Kernel kn_reduction;
	CL::Kernel kn_mul_M_v;

	int dim;

public :
	void Create( int newDim ) {
		dim = newDim;
		
		// context
		// platform
		EPH_List< CL::Platform > platforms;
		CL::System::EnumPlatforms( platforms );
		if ( platforms.IsEmpty() ) {
			return;
		}
		
		// context
		CL::System::CreateContext( platforms[1], context );

		cq.Create( context ); // , CL_QUEUE_PROFILING_ENABLE

		mem_M.CreateBuffer( context, CL_MEM_READ_WRITE, dim * sizeof( float ) );
		mem_x.CreateBuffer( context, CL_MEM_READ_WRITE, dim * sizeof( float ) );
		mem_p.CreateBuffer( context, CL_MEM_READ_WRITE, dim * sizeof( float ) );
		mem_r.CreateBuffer( context, CL_MEM_READ_WRITE, dim * sizeof( float ) );

		mem_Mr.CreateBuffer( context, CL_MEM_READ_WRITE, dim * sizeof( float ) );
		mem_Ap.CreateBuffer( context, CL_MEM_READ_WRITE, dim * sizeof( float ) );

		mem_rMr.CreateBuffer( context, CL_MEM_READ_WRITE, dim * sizeof( float ) );
		mem_pAp.CreateBuffer( context, CL_MEM_READ_WRITE, dim * sizeof( float ) );

		// kernel
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		EPH_TextFile source;
		source.Load( EPH_FileSystem::CurrentPath() + "/pcg.txt" );
		program.Create( context, source.Text(), flags );

		// kernel
		kn_zero_v		.Create( program, "zero_v" );
		kn_mul_v_v		.Create( program, "mul_v_v" );
		kn_mad_v_v_s	.Create( program, "mad_v_v_s" );
		kn_mul_M_v		.Create( program, "mul_M_v" );
		kn_reduction	.Create( program, "reduction" );
	}

	void Solve( SparseBlock12 &A, Eigen::VectorXf &b, Eigen::VectorXf &dx, int maxIters ) {
		dx.setZero();

		// x = 0
		cq.NDRangeKernel<1>( (kn_zero_v << dim, mem_x), Ceil( dim, 256 ), 256 );
	
		// p = 0
		cq.NDRangeKernel<1>( (kn_zero_v << dim, mem_p), Ceil( dim, 256 ), 256 );

		// M = 1 / diagonal( A ) : FIXME : Improve
		Eigen::VectorXf M;
		A.InverseDiagonal( M );
		cq.WriteBuffer( mem_M, CL_TRUE, 0, dim * sizeof( float ), M.data() ); 
	
		// r = b - A * x = b
		cq.WriteBuffer( mem_r, CL_TRUE, 0, dim * sizeof( float ), b.data() ); 

		// A
		EPH_List< float > A_elems;
		EPH_List< int > A_indices_row, A_indices_col;
		
		A.Flatten( A_elems, A_indices_row, A_indices_col );
	//	timer.Print( "Flatten" );

		mem_A		.CreateBuffer( context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, A_elems.BufferSize(), A_elems.Ptr() );
		mem_A_rows	.CreateBuffer( context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, A_indices_row.BufferSize(), A_indices_row.Ptr() );
		mem_A_cols	.CreateBuffer( context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, A_indices_col.BufferSize(), A_indices_col.Ptr() );

	//	cq.WriteBuffer( mem_A, CL_TRUE, A_elems );
	//	cq.WriteBuffer( mem_A_rows, CL_TRUE, A_indices_row );
	//	cq.WriteBuffer( mem_A_cols, CL_TRUE, A_indices_col );

		CL::Event ev;
		CL::Profile pf;

		EPH_Timer timer;
		timer.Restart();
		for ( int i = 0; i < maxIters; i++ ) {
			// M * r
			pf.Zero();
			cq.NDRangeKernel<1>( (kn_mul_v_v << dim, mem_Mr, mem_M, mem_r), Ceil( dim, 256 ), 256, &ev );
			// pf += ev; pf.Print( "M * r   " );

			// r dot Mr
			pf.Zero();
			cq.NDRangeKernel<1>( (kn_mul_v_v << dim, mem_rMr, mem_r, mem_Mr), Ceil( dim, 256 ), 256, &ev );
			// pf += ev; pf.Print( "r dot Mr" );

			pf.Zero();
			{
				// group_size * num_self_accum * num_groups >= size
				for( int size = dim; size > 1; ) {
					int group_size = 256;
					int num_self_accum = 4;
					int num_groups;
					int size_ceil = Ceil( size, group_size * num_self_accum, num_groups );
					int stride = group_size * num_groups;
					cq.NDRangeKernel<1>( (kn_reduction << size, stride, mem_rMr), size_ceil, 256, &ev );
					size = num_groups;
				}
			}
			// pf += ev; pf.Print( "r dot Mr" );

			float rMr;
			cq.ReadBuffer( mem_rMr, CL_TRUE, 0, sizeof( float ), &rMr );
			if ( rMr < 1e-5f ) {
				break;
			}

			// p += Mr / rMr
			pf.Zero();
			cq.NDRangeKernel<1>( (kn_mad_v_v_s << dim, mem_p, mem_Mr, 1.0f / rMr), Ceil( dim, 256 ), 256, &ev);
			// pf += ev; pf.Print( "p += Mr / rMr" );

			// Ap = A * p
			pf.Zero();
			cq.NDRangeKernel<1>( (kn_mul_M_v << (A_indices_row.Num() - 1), mem_Ap, mem_A, mem_A_rows, mem_A_cols, mem_p), Ceil(A_indices_row.Num() - 1, 256), 256, &ev );
			// pf += ev; pf.Print( "Ap = A * p" );

			// p dot Ap
			pf.Zero();
			cq.NDRangeKernel<1>( (kn_mul_v_v << dim, mem_pAp, mem_p, mem_Ap), Ceil( dim, 256 ), 256, &ev);
			// pf += ev; pf.Print( "p dot Ap" );
		
			pf.Zero();
			{
				for( int size = dim; size > 1; ) {
					int group_size = 256;
					int num_self_accum = 4;
					int num_groups;
					int size_ceil = Ceil( size, group_size * num_self_accum, num_groups );
					int stride = group_size * num_groups;
					cq.NDRangeKernel<1>( (kn_reduction << size, stride, mem_pAp), size_ceil, 256, &ev );
					size = num_groups;
				}
			}
			// pf += ev; pf.Print( "p dot Ap" );
		
			float pAp;
			cq.ReadBuffer( mem_pAp, CL_TRUE, 0, sizeof( float ), &pAp );

			// x += p / pAp
			pf.Zero();
			cq.NDRangeKernel<1>( (kn_mad_v_v_s << dim, mem_x, mem_p, 1.0f / pAp), Ceil( dim, 256 ), 256, &ev);
			// pf += ev; pf.Print( "x += p / pAp" );

			// r -= Ap / pAp
			pf.Zero();
			cq.NDRangeKernel<1>( (kn_mad_v_v_s << dim, mem_r, mem_Ap, -1.0f / pAp), Ceil( dim, 256 ), 256, &ev);
			// pf += ev; pf.Print( "r -= Ap / pAp" );

			// printf( "%d : rMr %f : pAp %f\n", i, rMr, pAp );
		}
		timer.Print( "pcg" );

		cq.ReadBuffer( mem_x, CL_TRUE, 0, sizeof( float ) * dim, dx.data() );
	}
};
