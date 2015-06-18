#define MAX_DEPTH	(100000.0f)

//============================================
// Image
//============================================
float get_depth( __global float *depth_map, int2 dim, float2 v ) {
	int2 p = clamp( convert_int2_rtn( v ), (int2)(0), dim - 1 );
	return depth_map[ p.x + p.y * dim.x ] * 0.001f;
}

float3 get_normal( __global float3 *normal_map, int2 dim, float2 v ) {
	int2 p = clamp( convert_int2_rtn( v ), (int2)(0), dim - 1 );
	return normal_map[ p.x + p.y * dim.x ];
}

__kernel void depth_to_point( __global float *depth, __global float3 *points, int2 dim, float4 IK ) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if ( x >= dim.x || y >= dim.y ) {
        return;
    }

    int index = x + y * dim.x;

    float d = depth[ index ] * 0.001f;

    float3 p = ( float3 )( x, y, d );
    p.xy -= IK.lo;
    p.xy /= IK.hi;
	p.xy *= d;
    
    points[ index ] = p;
}

__kernel void point_to_normal( __global float3 *points, __global float3 *normals, int2 dim ) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if ( x >= dim.x || y >= dim.y ) {
        return;
    }

    int index = x + y * dim.x;

    // FIXME : improve
    x = clamp( x, 1, dim.x - 2 );
    y = clamp( y, 1, dim.y - 2 );
    int k = x + y * dim.x;

    float3 s = points[ k + 1 ] - points[ k - 1 ];
    float3 t = points[ k + dim.x ] - points[ k - dim.x ];

    normals[ index ] = normalize( -cross( s, t ) );
}

__kernel void point_to_world( __global float3 *p, __global float3 *w, int2 dim, float16 M ) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    if ( x >= dim.x || y >= dim.y ) {
        return;
    }

    int index = x + y * dim.x;

    float3 v = p[ index ];
    
    w[ index ] = M.lo.lo.xyz * v.x + M.lo.hi.xyz * v.y + M.hi.lo.xyz * v.z + M.hi.hi.xyz;
}

//============================================
// TSDF
//============================================
float2 get_df( __global float2 *vol, int3 dim, float3 v ) {
	int3 p = clamp( convert_int3_rtn( v ), (int3)(0), dim - 1 );
	return vol[ p.x + p.y * dim.x + p.z * dim.x * dim.y ];
}

float get_df_c( __global float2 *vol, int3 dim, float3 v ) {
	v -= 0.5f;
	float3 i = floor(v);
	float3 f = v - i;

	float e000 = get_df( vol, dim, i + (float3)(0.0, 0.0, 0.0) ).x;
	float e100 = get_df( vol, dim, i + (float3)(1.0, 0.0, 0.0) ).x;
	float e010 = get_df( vol, dim, i + (float3)(0.0, 1.0, 0.0) ).x;
	float e110 = get_df( vol, dim, i + (float3)(1.0, 1.0, 0.0) ).x;

	float e001 = get_df( vol, dim, i + (float3)(0.0, 0.0, 1.0) ).x;
	float e101 = get_df( vol, dim, i + (float3)(1.0, 0.0, 1.0) ).x;
	float e011 = get_df( vol, dim, i + (float3)(0.0, 1.0, 1.0) ).x;
	float e111 = get_df( vol, dim, i + (float3)(1.0, 1.0, 1.0) ).x;

	float e00 = f.x * ( e100 - e000 ) + e000;
	float e10 = f.x * ( e110 - e010 ) + e010;
	float e01 = f.x * ( e101 - e001 ) + e001;
	float e11 = f.x * ( e111 - e011 ) + e011;

	float e0 = f.y * ( e10 - e00 ) + e00;
	float e1 = f.y * ( e11 - e01 ) + e01;

	float e = f.z * ( e1 - e0 ) + e0;

	return e;
}

float3 get_grad( __global float2 *vol, int3 dim, float3 v ) {
	float3 s = (float3)( 1.0, 0.0, 0.0 );

#if 1
	float3 n = (float3)(
						get_df_c( vol, dim, v + s.xyz ) - get_df_c( vol, dim, v - s.xyz ),
						get_df_c( vol, dim, v + s.zxy ) - get_df_c( vol, dim, v - s.zxy ),
						get_df_c( vol, dim, v + s.yzx ) - get_df_c( vol, dim, v - s.yzx )
						);
#else
	float3 n = (float3)(
						get_df( vol, dim, v + s.xyz ).x - get_df( vol, dim, v - s.xyz ).x,
						get_df( vol, dim, v + s.zxy ).x - get_df( vol, dim, v - s.zxy ).x,
						get_df( vol, dim, v + s.yzx ).x - get_df( vol, dim, v - s.yzx ).x
						);
#endif

	return normalize( n );
}

__kernel void df_clear( __global float2 *vol, int3 dim, float2 val ) {
	int x = get_global_id(0);
	int y = get_global_id(1);
	int z = get_global_id(2);

	if ( x >= dim.x || y >= dim.y || z >= dim.z ) {
		return;
	}

	int index = x + y * dim.x + z * dim.x * dim.y;

	vol[ index ] = val;
}

__kernel void df_sphere( __global float2 *vol, int3 dim, float r ) {
	int x = get_global_id(0);
	int y = get_global_id(1);
	int z = get_global_id(2);

	if ( x >= dim.x || y >= dim.y || z >= dim.z ) {
		return;
	}

	int index = x + y * dim.x + z * dim.x * dim.y;

	float3 v = (float3)( x, y, z ) + 0.5f;
	float3 c = (float3)( dim.x, dim.y, dim.z ) * 0.5f;

	float d = length( v - c ) - r;
	vol[ index ] = (float2)( d, 1.0 );
}

#define TRUNC		( 3.0f )
#define MAX_WEIGHT	( 2.0f )
__kernel void df_fuse(
			 __global float2 *vol, int3 vol_dim,
			 __global float *depth_map, __global float3 *normal_map, int2 map_dim,
			 float4 K, float16 M, float4 L
			 ) {
	int x = get_global_id(0);
	int y = get_global_id(1);
	int z = get_global_id(2);

	// volume space
	int3 pos = (int3)( x, y, z);
	int index = pos.x + pos.y * vol_dim.x + pos.z * vol_dim.x * vol_dim.y;
	if ( pos.x >= vol_dim.x || pos.y >= vol_dim.y || pos.z >= vol_dim.z ) {
        return;
    }

    // world space
    float3 world_v = ( convert_float3( pos ) + 0.5f ) * L.w + L.xyz;

    // view space
    float3 view_v = world_v - M.hi.hi.xyz;
    view_v = (float3)( dot( view_v, M.lo.lo.xyz ), dot( view_v, M.lo.hi.xyz ), dot( view_v, M.hi.lo.xyz ) );
    float3 dir = normalize( view_v );

    // proj space
    float  proj_z = view_v.z;
    float2 proj_v = view_v.xy / proj_z;
    proj_v *= K.hi;
    proj_v += K.lo;

    // proj range
    if ( proj_v.x < 0.0 || proj_v.y < 0.0 || proj_v.x >= map_dim.x || proj_v.y >= map_dim.y ) {
        return;
    }

//	vol[ index ] = (float2)(surf_z - proj_z );
//	return;

	  // normal
    float3 n = get_normal( normal_map, map_dim, proj_v );

    // sdf
    float dot_dir_n = -dot( dir, n );
    if ( dot_dir_n < 0.2 ) {
		return;
    }
	dot_dir_n /= dir.z; // dot_dir_z = dir.z

	float surf_z = get_depth( depth_map, map_dim, proj_v );
 
    float d = ( surf_z - proj_z ) * dot_dir_n / L.w;
	float w = dot_dir_n / proj_z * L.w;

	if ( d < -TRUNC || w <= 0.0 ) {
		return;
    }

    // weighted sum
    float2 df = vol[ index ];
    df.x = ( df.x * df.y + d * w ) / ( df.y + w );
    df.y += w;

	// clamp
	df.x = clamp( df.x, -TRUNC, TRUNC );
	df.y = min( df.y, MAX_WEIGHT );

	vol[ index ] = df;
}

//============================================
// Ray Cast
//============================================
float2 ray_vs_bound( float3 o, float3 inv, float3 bmin, float3 bmax, float2 e ) {
	float3 s = ( bmin - o ) * inv;
	float3 t = ( bmax - o ) * inv;
	float3 a = min( s, t );
	float3 b = max( s, t );
	return (float2)(
					max( max( e.x, a.x ), max( a.y, a.z ) ),
					min( min( e.y, b.x ), min( b.y, b.z ) )
					);
}

float ray_vs_df( __global float2 *vol, int3 dim, float3 o, float3 dir, float2 e ) {
	float t = e.x;
	float last_d = TRUNC;

	for ( int i = 0; i < 1500; i++ ){
		t += 1.0f;
		if ( t >= e.y ) {
			break;
		}

		float3 v = o + dir * t;
		float2 d = get_df( vol, dim, v );
		if ( d.x <= 0.0 && d.y > 0.0 ) {
			t -= ( d.x / ( d.x - last_d ) );
			break;
		}
		last_d = d.x;
	}

	return t;
}

float3 gen_ray( float4 K, float16 M, float4 L, float2 p, float3 *o, float3 *dir ) {
	p -= K.lo;
	p /= K.hi;

	*dir = normalize( p.x * M.lo.lo.xyz + p.y * M.lo.hi.xyz - M.hi.lo.xyz );
	*o   = ( M.hi.hi.xyz - L.xyz ) / L.w;
}

__kernel void ray_cast( __global float4 *col, int2 col_dim, __global float2 *vol, int3 vol_dim, float4 K, float16 M, float4 L ) {
	int x = get_global_id(0);
	int y = get_global_id(1);
	if ( x >= col_dim.x || y >= col_dim.y ) {
		return;
	}

	int index = x + y * col_dim.x;

	// gen ray
	float3 o, dir;
	gen_ray( K, M, L, (float2)(x, y) + 0.5f, &o, &dir );
	float3 inv = 1.0f / dir;

	// ray vs bound
	float2 e = ray_vs_bound( o, inv, (float3)( 0.0 ), convert_float3_rtn( vol_dim ), (float2)( 0.0, MAX_DEPTH ) );
	if ( e.x >= e.y ) {
		col[ index ] = ( float4 )( 0.0 );
		return;
	}

	// ray vs df
	float d = ray_vs_df( vol, vol_dim, o, dir, e );
	if ( d >= e.y )
	{
		col[ index ] = ( float4 )( 0.0, 0.0, 0.0, 0.0 );
		return;
	}

	float3 v = o + dir * d;
	float3 n = get_grad( vol, vol_dim, v );
	col[ index ] = ( float4 )( (float3)(dot( n, -dir )), 1.0 );
}
