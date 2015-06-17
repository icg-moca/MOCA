//
//  cl.h
//  Moc
//
//  Created by Wei Li on 5/14/15.
//  Copyright (c) 2015 EPH. All rights reserved.
//

#ifndef __EPH_CL__
#define __EPH_CL__

#include <stdio.h>
#include <vector>
using namespace std;

#ifdef __APPLE__
//#define CL_USE_DEPRECATED_OPENCL_1_0_APIS
//#define CL_USE_DEPRECATED_OPENCL_1_1_APIS
#include <opencl/cl.h>
#include <opencl/cl_platform.h>
#include <opencl/cl_gl.h>
#include <opencl/cl_ext.h>
#include <opencl/cl_gl_ext.h>
#endif

#ifdef _WIN32
//#define CL_USE_DEPRECATED_OPENCL_1_0_APIS
//#define CL_USE_DEPRECATED_OPENCL_1_1_APIS
#include <cl/cl.h>
#include <cl/cl_platform.h>
#include <cl/cl_gl.h>
#include <cl/cl_ext.h>
#include <cl/cl_gl_ext.h>
#endif

//================
// CL
//================
namespace cl {

	// error codes
	const int NUM_ERROR_CODES = 64;
	const char *error_code[ NUM_ERROR_CODES ] = {
		"CL_SUCCESS"									, // 0
		"CL_DEVICE_NOT_FOUND"							, // -1
		"CL_DEVICE_NOT_AVAILABLE"						, // -2
		"CL_COMPILER_NOT_AVAILABLE"						, // -3
		"CL_MEM_OBJECT_ALLOCATION_FAILURE"				, // -4
		"CL_OUT_OF_RESOURCES"							, // -5
		"CL_OUT_OF_HOST_MEMORY"							, // -6
		"CL_PROFILING_INFO_NOT_AVAILABLE"				, // -7
		"CL_MEM_COPY_OVERLAP"							, // -8
		"CL_IMAGE_FORMAT_MISMATCH"						, // -9
		"CL_IMAGE_FORMAT_NOT_SUPPORTED"					, // -10
		"CL_BUILD_PROGRAM_FAILURE"						, // -11
		"CL_MAP_FAILURE"								, // -12
		"CL_MISALIGNED_SUB_BUFFER_OFFSET"				, // -13
		"CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST"	, // -14
		""												, // -15
		""												, // -16
		""												, // -17
		""												, // -18
		""												, // -19
		""												, // -20
		""												, // -21
		""												, // -22
		""												, // -23
		""												, // -24
		""												, // -25
		""												, // -26
		""												, // -27
		""												, // -28
		""												, // -29
		"CL_INVALID_VALUE"								, // -30
		"CL_INVALID_DEVICE_TYPE"						, // -31
		"CL_INVALID_PLATFORM"							, // -32
		"CL_INVALID_DEVICE"								, // -33
		"CL_INVALID_CONTEXT"							, // -34
		"CL_INVALID_QUEUE_PROPERTIES"					, // -35
		"CL_INVALID_COMMAND_QUEUE"						, // -36
		"CL_INVALID_HOST_PTR"							, // -37
		"CL_INVALID_MEM_OBJECT"							, // -38
		"CL_INVALID_IMAGE_FORMAT_DESCRIPTOR"			, // -39
		"CL_INVALID_IMAGE_SIZE"							, // -40
		"CL_INVALID_SAMPLER"							, // -41
		"CL_INVALID_BINARY"								, // -42
		"CL_INVALID_BUILD_OPTIONS"						, // -43
		"CL_INVALID_PROGRAM"							, // -44
		"CL_INVALID_PROGRAM_EXECUTABLE"					, // -45
		"CL_INVALID_KERNEL_NAME"						, // -46
		"CL_INVALID_KERNEL_DEFINITION"					, // -47
		"CL_INVALID_KERNEL"								, // -48
		"CL_INVALID_ARG_INDEX"							, // -49
		"CL_INVALID_ARG_VALUE"							, // -50
		"CL_INVALID_ARG_SIZE"							, // -51
		"CL_INVALID_KERNEL_ARGS"						, // -52
		"CL_INVALID_WORK_DIMENSION"						, // -53
		"CL_INVALID_WORK_GROUP_SIZE"					, // -54
		"CL_INVALID_WORK_ITEM_SIZE"						, // -55
		"CL_INVALID_GLOBAL_OFFSET"						, // -56
		"CL_INVALID_EVENT_WAIT_LIST"					, // -57
		"CL_INVALID_EVENT"								, // -58
		"CL_INVALID_OPERATION"							, // -59
		"CL_INVALID_GL_OBJECT"							, // -60
		"CL_INVALID_BUFFER_SIZE"						, // -61
		"CL_INVALID_MIP_LEVEL"							, // -62
		"CL_INVALID_GLOBAL_WORK_SIZE"					  // -63
	};

	//================
	// Vec
	//================
	template< class T >
	class vec2 {
	public :
		T x, y;

	public :
		vec2() {
			x = y = 0;
		}

		vec2( const T r ) {
			x = y = r;
		}

		vec2( const T x, const T y ) {
			this->x = x;
			this->y = y;
		}

		T *ptr( void ) {
			return &x;
		}

		const T *ptr( void ) const {
			return &x;
		}

		T &operator[]( int index ) {
			assert( index >= 0 && index < 2 );
			return ptr()[ index ];
		}

		const T &operator[]( int index ) const {
			assert( index >= 0 && index < 2 );
			return ptr()[ index ];
		}

		T area( void ) const {
			return x * y;
		}
	};

	template< class T >
	class vec3 {
	public :
		T x, y, z;

	public :
		vec3() {
			x = y = z = 0;
		}

		vec3( const T r ) {
			x = y = z = r;
		}

		vec3( const T x, const T y, const T z ) {
			this->x = x;
			this->y = y;
			this->z = z;
		}

		T *ptr( void ) {
			return &x;
		}

		const T *ptr( void ) const {
			return &x;
		}

		T &operator[]( int index ) {
			assert( index >= 0 && index < 3 );
			return ptr()[ index ];
		}

		const T &operator[]( int index ) const {
			assert( index >= 0 && index < 3 );
			return ptr()[ index ];
		}

		T volume( void ) const {
			return x * y * z;
		}
	};

	template< class T >
	class vec4 {
		public :
		T x, y, z, w;

		public :
		vec4() {
			x = y = z = w = 0;
		}

		vec4( const T r ) {
			x = y = z = w = r;
		}

		vec4( const T x, const T y, const T z, const T w ) {
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		T *ptr( void ) {
			return &x;
		}

		const T *ptr( void ) const {
			return &x;
		}

		T &operator[]( int index ) {
			assert( index >= 0 && index < 4 );
			return ptr()[ index ];
		}

		const T &operator[]( int index ) const {
			assert( index >= 0 && index < 4 );
			return ptr()[ index ];
		}
	};

	typedef vec2<int> int2;
	typedef vec3<int> int3;
	typedef vec4<int> int4;

	typedef vec2<float> float2;
	typedef vec3<float> float3;
	typedef vec4<float> float4;

	typedef vec2<size_t> size2;
	typedef vec3<size_t> size3;
	typedef vec4<size_t> size4;

	//================
	// Error
	//================
	bool Error( cl_int error, const char *api ) {
		if ( !error ) {
			return true;
		}
		error = -error;

		if ( error <= 0 || error >= NUM_ERROR_CODES ) {
			printf( "[CL] ERROR : %s : %d\n", api, -error );
		} else {
			printf( "[CL] ERROR : %s : %s\n", api, error_code[ error ] );
		}
		return false;
	}

	//================
	// Log
	//================
	void Log( const char *str ) {
		printf( "[CL] : %s\n", str );
	}

	//================
	// Device
	//================
	class Device {
		public :
		cl_device_id _id;

		public :
		Device() : _id( 0 ) {
		}

		bool IsValid() const {
			return ( _id != 0 );
		}

		cl_device_id ID( void ) const {
			return _id;
		}
	};

	//================
	// Platform
	//================
	class Platform {
		public :
		cl_platform_id _id;

		public :
		Platform() : _id( 0 ) {
		}

		bool IsValid() const {
			return ( _id != 0 );
		}

		cl_platform_id ID( void ) const {
			return _id;
		}

		void EnumDevices( cl_device_type type, vector< Device > &devices ) {
			devices.clear();
			cl_uint num = 0;
			Error( clGetDeviceIDs( _id, type, 0, 0, &num ), "clGetDeviceIDs" );
			if ( num <= 0 ) {
				return;
			}
			devices.resize( num );
			Error( clGetDeviceIDs( _id, type, num, ( cl_device_id * )devices.data(), 0 ), "clGetDeviceIDs" ); // FIXME : convert
			printf( "[CL] Device : %d\n", num );
		}

		Device DefaultDevice( const char **ext = 0 ) {
			vector< Device > list;
			EnumDevices( CL_DEVICE_TYPE_GPU, list );
			printf( "[CL] : GPU Device : %lu\n", list.size() );
			if ( list.size() == 0 ) {
				EnumDevices( CL_DEVICE_TYPE_CPU, list );
				printf( "[CL] : CPU Device : %d\n", list.size() );
				if ( list.size() == 0 ) {
					Error( -1, "cl::DefaultDevice" );
					return Device();
				}
			}

			return list[0];

			Error( -1, "cl::DefaultDevice" );
			return Device();
		}

		void Version( void ) {
			size_t size;
			Error( clGetPlatformInfo( _id, CL_PLATFORM_VERSION, 0, 0, &size ), "clGetPlatformInfo : version" );

			vector< char > str( size );
			Error( clGetPlatformInfo( _id, CL_PLATFORM_VERSION, size, str.data(), 0 ), "clGetPlatformInfo : version" );

			printf( "[CL] Version\t: %s\n", str.data() );
		}

		void Vendor( void ) {
			size_t size;
			Error( clGetPlatformInfo( _id, CL_PLATFORM_VENDOR, 0, 0, &size ), "clGetPlatformInfo : vendor" );


			vector< char > str( size );
			Error( clGetPlatformInfo( _id, CL_PLATFORM_VENDOR, size, str.data(), 0 ), "clGetPlatformInfo : vendor" );

			printf( "[CL] Vendor\t: %s\n", str.data() );
		}
	};

	//================
	// Context
	//================
	class Context {
		public :
		cl_context		_context;
		cl_device_id	_device_id;

		public :
		Context() : _context( 0 ), _device_id( 0 ) {
		}

		// FIXME : init, release, create
		Context( Device device, const cl_context_properties *props = 0 ) : _context( 0 ), _device_id( 0 ) {
			Release();
			Create( device, props );
		}

		bool IsValid() const {
			return ( _context != 0 );
		}

		void Release( void ) {
			if ( _context ) {
				clReleaseContext( _context );
				_context = 0;
				_device_id = 0;
			}
		}

		void Create( Device device, const cl_context_properties *props = 0 ) {
			Release();
			_device_id = device.ID();
			cl_int err;
			_context = clCreateContext( props, 1, &_device_id, 0, 0, &err ); Error( err, "clCreateContext" );
		}

		~Context() {
			Release();
		}

		Context( const Context &context ) {
			*this = context;
		}

		Context &operator=( const Context &context ) {
			_context		= context._context;
			_device_id		= context._device_id;

			if ( _context ) {
				clRetainContext( _context );
			}

			return *this;
		}
	};

	enum {
		SUPPORT_CL_GL_INTEROP = 1
	};

	//================
	// System
	//================
	class System {
		public :
		static void EnumPlatforms( vector< Platform > &platforms ) {
			platforms.clear();
			cl_uint num = 0;
			Error( clGetPlatformIDs( 0, 0, &num ), "clGetPlatformIDs" );
			if ( num <= 0 ) {
				return;
			}
			platforms.resize( num );
			Error( clGetPlatformIDs( num, ( cl_platform_id * )platforms.data(), 0 ), "clGetPlatformIDs" ); // FIXME
			printf( "[CL] Platform\t: %d\n", num );
		}

		static Platform DefaultPlatform( void ) {
			vector< Platform > list;
			EnumPlatforms( list );
			if ( list.size() == 0 ) {
				return Platform();
			}
			list[ 0 ].Version();
			list[ 0 ].Vendor();
			return list[ 0 ];
		}

		static void CreateContext( Context &context, const int support = 0 ) {
			// platform
			Platform platform = DefaultPlatform();

			// extensions and properties
			vector< const char * > ext;
			vector< cl_context_properties > props;

			props.push_back( (cl_context_properties)CL_CONTEXT_PLATFORM ); props.push_back( (cl_context_properties)platform.ID() ); // render context

#if 0
			if ( support & SUPPORT_CL_GL_INTEROP ) {
#ifdef _WIN32
				ext.push_back( "cl_khr_gl_sharing" );
				props.push_back( (cl_context_properties)CL_WGL_HDC_KHR ); props.push_back( (cl_context_properties)wglGetCurrentDC() ); // device context
				props.push_back( (cl_context_properties)CL_GL_CONTEXT_KHR ); props.push_back( (cl_context_properties)wglGetCurrentContext() ); // render context
#endif

#ifdef __APPLE__
				ext.push_back( "cl_APPLE_gl_sharing" );
				CGLContextObj kCGLContext = CGLGetCurrentContext();
				CGLShareGroupObj kCGLShareGroup = CGLGetShareGroup(kCGLContext);
				props.push_back( (cl_context_properties)CL_CONTEXT_PROPERTY_USE_CGL_SHAREGROUP_APPLE); props.push_back( (cl_context_properties)kCGLShareGroup ); // device context#endif
#endif
			};
#endif
			ext.push_back( 0 ); // FIXME
			props.push_back( 0 ); // FIXME

			// device
			Device device = platform.DefaultDevice( ext.data() );

			// context
			context.Create( device, props.data() );
		}
	};

	//================
	// Program
	//================
	class Program {
		public :
		cl_program		_program;
		cl_context		_context;
		cl_device_id	_device_id;

		public :
		Program() : _program( 0 ), _context( 0 ), _device_id( 0 ) {
		}

		Program( Context &context, const char *source, const char *options = 0 ) : _program( 0 ), _context( 0 ), _device_id( 0 ) {
			Create( context, source );
		}

		bool IsValid() const {
			return ( _program != 0 );
		}

		void Release( void ) {
			if ( _program ) {
				clReleaseProgram( _program );
				_program = 0;
				_context = 0;
				_device_id = 0;
			}
		}

		void Create( Context &context, cl_uint count, const char **sources, const size_t *lengths = 0, const char *options = 0 ) {
			Release();
			_context = context._context;
			_device_id = context._device_id;

			cl_int err;
			_program = clCreateProgramWithSource( _context, count, sources, lengths, &err ); Error( err, "clCreateProgramWithSource" );
			Build( options ); // FIXME
		}

		void Create( Context &context, const char *source, const char *options = 0 ) {
			Create( context, 1, &source, 0, options );
		}

		void Build( const char *options = 0 ) {
			cl_int err = clBuildProgram( _program, 0, 0, options, 0, 0 ); Error( err, "clBuildProgram" );
			if ( err ) {
				BuildInfo();
			}
		}

		void BuildInfo( void ) {
			size_t size = 0;
			clGetProgramBuildInfo( _program, _device_id, CL_PROGRAM_BUILD_LOG, 0, 0, &size );

			vector< char > log( size );
			clGetProgramBuildInfo( _program, _device_id, CL_PROGRAM_BUILD_LOG, size, log.data(), 0 );

			printf( "[CL] Program :\n" );
			printf( "%s\n", log.data() );
		}

		public :
		~Program() {
			Release();
		}

		Program( const Program &prog ) {
			*this = prog;
		}

		Program &operator=( const Program &prog ) {
			if ( _program == prog._program ) {
				return *this;
			}

			Release();

			_program		= prog._program;
			_context		= prog._context;
			_device_id		= prog._device_id;
			if ( _program ) {
				clRetainProgram( _program );
			}

			return *this;
		}
	};

	//================
	// Kernel
	//================
	class Kernel {
		public :
		cl_kernel _kernel;

		public :
		Kernel() : _kernel( 0 ) {
		}

		Kernel( Program &program, const char *name ) : _kernel( 0 ) {
			Create( program, name );
		}

		bool IsValid() const {
			return ( _kernel != 0 );
		}

		void Release( void ) {
			if ( _kernel ) {
				clReleaseKernel( _kernel );
				_kernel = 0;
			}
		}

		void Create( Program &program, const char *name ) {
			Release();
			cl_int err;
			_kernel = clCreateKernel( program._program, name, &err ); Error( err, "clCreateKernel"  );
		}

		template< class T >
		Kernel &SetArg( int index, const T &arg ) {
			Error( clSetKernelArg( _kernel, index, sizeof( T ), ( const void* )&arg ), "clSetKernelArg" );
			return *this;
		}

		Kernel &SetArg( int index, size_t size, const void *arg = NULL ) {
			Error( clSetKernelArg( _kernel, index, size, arg ), "clSetKernelArg" );
			return *this;
		}

		template< class T >
		Kernel &SetArg( int index, const vector<T> &arg ) {
			Error( clSetKernelArg( _kernel, index, arg.size() * sizeof( T ), ( const void* )arg.data() ), "clSetKernelArg" );
			return *this;
		}

		public :
		int _arg_index;

		template< class T >
		Kernel &operator<<( const T &arg ) {
			_arg_index = 0;
			SetArg( _arg_index, arg );
			return *this;
		}

		template< class T >
		Kernel &operator,( const T &arg ) {
			_arg_index++;
			SetArg( _arg_index, arg );
			return *this;
		}

		public :
		~Kernel() {
			Release();
		}

		Kernel( const Kernel &kn ) {
			*this = kn;
		}

		Kernel &operator=( const Kernel &kn ) {
			if ( _kernel == kn._kernel ) {
				return *this;
			}

			Release();

			_kernel = kn._kernel;
			if ( _kernel ) {
				clRetainKernel( _kernel );
			}

			return *this;
		}
	};

	//================
	// Mem
	//================
	class Mem {
		public :
		cl_mem _mem;

		public :
		Mem() : _mem( 0 ) {
		}

		Mem( Context &context, cl_mem_flags flags, size_t size, void *ptr = 0 ) : _mem( 0 ) { // FIXME : _mem( 0 )
			CreateBuffer( context, flags, size, ptr );
		}

		template< class T >
		Mem( Context &context, cl_mem_flags flags, vector< T > &data ) : _mem( 0 ) {
			CreateBuffer( context, flags, data );
		}

		public :
		bool IsValid() const {
			return ( _mem != 0 );
		}

		void Release( void ) {
			if ( _mem ) {
				clReleaseMemObject( _mem );
				_mem = 0;
			}
		}

		void CreateBuffer( Context &context, cl_mem_flags flags, size_t size, void *ptr = 0 ) {
			Release();
			cl_int err;
			_mem = clCreateBuffer( context._context, flags, size, ptr, &err ); Error( err, "clCreateBuffer" );
		}

		template< class T >
		void CreateBuffer( Context &context, cl_mem_flags flags, vector< T > &data ) {
			Release();
			CreateBuffer( context, flags, data.size() * sizeof( T ), data.data() );
		}

		public :
		~Mem() {
			Release();
		}

		Mem( const Mem &mem ) {
			*this = mem;
		}

		Mem &operator=( const Mem &mem ) {
			if ( _mem == mem._mem ) {
				return *this;
			}

			Release();

			_mem = mem._mem;
			if ( _mem ) {
				clRetainMemObject( _mem );
			}

			return *this;
		}
	};

	//================
	// CommandQueue
	//================
	class CommandQueue {
		public :
		cl_command_queue _cmd_queue;

		public :
		CommandQueue() : _cmd_queue( 0 ) {
		}

		CommandQueue( Context &context, cl_command_queue_properties props = 0 ) : _cmd_queue( 0 ) {
			Create( context, props );
		}

		bool IsValid() const {
			return ( _cmd_queue != 0 );
		}

		void Release( void ) {
			if ( _cmd_queue ) {
				clReleaseCommandQueue( _cmd_queue );
				_cmd_queue = 0;
			}
		}

		void Create( Context &context, cl_command_queue_properties props = 0 ) {
			Release();
			cl_int err;
			_cmd_queue = clCreateCommandQueue( context._context, context._device_id, props, &err ); Error( err, "clCreateCommandQueue" );
		}

		void Finish( void ) {
			clFinish( _cmd_queue );
		}

		void NDRangeKernel1( Kernel &kernel, const size_t global_work_size, const size_t local_work_size) {
			// FIXME : offset must currently be a NULL value
			Error( clEnqueueNDRangeKernel( _cmd_queue, kernel._kernel, 1, 0, &global_work_size, &local_work_size, 0, 0, 0 ), "clEnqueueNDRangeKernel" );
		}

		void NDRangeKernel2( Kernel &kernel, const size2 &global_work_size, const size2 &local_work_size) {
			// FIXME : offset must currently be a NULL value
			Error( clEnqueueNDRangeKernel( _cmd_queue, kernel._kernel, 2, 0, global_work_size.ptr(), local_work_size.ptr(), 0, 0, 0 ), "clEnqueueNDRangeKernel" );
		}

		void NDRangeKernel3( Kernel &kernel, const size3 &global_work_size, const size3 &local_work_size ) {
			// FIXME : offset must currently be a NULL value
			Error( clEnqueueNDRangeKernel( _cmd_queue, kernel._kernel, 3, 0, global_work_size.ptr(), local_work_size.ptr(), 0, 0, 0 ), "clEnqueueNDRangeKernel" );
		}

		void ReadBuffer( Mem &mem, cl_bool block, size_t offset, size_t size, void *ptr) {
			Error( clEnqueueReadBuffer( _cmd_queue, mem._mem, block, offset, size, ptr, 0, 0, 0 ), "clEnqueueReadBuffer" );
		}

		void WriteBuffer( Mem &mem, cl_bool block, size_t offset, size_t size, const void *ptr ) {
			Error( clEnqueueWriteBuffer( _cmd_queue, mem._mem, block, offset, size, ptr, 0, 0, 0 ), "clEnqueueWriteBuffer" );
		}

		void CopyBuffer( Mem &src, Mem &dst, size_t src_offset, size_t dst_offset, size_t size ) {
			Error( clEnqueueCopyBuffer( _cmd_queue, src._mem, dst._mem, src_offset, dst_offset, size, 0, 0, 0 ), "clEnqueueCopyBuffer" );
		}

		void ReadImage( Mem &mem, cl_bool block, const size3 &offset, const size3 &size, size_t row_pitch, size_t slice_pitch, void *ptr ) {
			Error( clEnqueueReadImage( _cmd_queue, mem._mem, block, offset.ptr(), size.ptr(), row_pitch, slice_pitch, ptr, 0, 0, 0 ), "clEnqueueReadImage" );
		}

		void WriteImage( Mem &mem, cl_bool block, const size3 &offset, const size3 &size, size_t row_pitch, size_t slice_pitch, const void *ptr ) {
			Error( clEnqueueWriteImage( _cmd_queue, mem._mem, block, offset.ptr(), size.ptr(), row_pitch, slice_pitch, ptr, 0, 0, 0 ), "clEnqueueWriteImage" );
		}
		
		void AcquireGLObjects( Mem &mem ) {
			Error( clEnqueueAcquireGLObjects( _cmd_queue, 1, &mem._mem, 0, 0, 0 ), "clEnqueueAcquireGLObjects" );
		}
		
		void ReleaseGLObject( Mem &mem ) {
			Error( clEnqueueReleaseGLObjects( _cmd_queue, 1, &mem._mem, 0, 0, 0 ),"clEnqueueReleaseGLObjects" );
		}
		
		public :
		template< class T >
		void ReadBuffer( Mem &mem, cl_bool block, vector< T > &data ) {
			ReadBuffer( mem, block, 0, data.size() * sizeof( T ), data.data() );
		}
		
		template< class T >
		void WriteBuffer( Mem &mem, cl_bool block, const vector< T > &data ) {
			WriteBuffer( mem, block, 0, data.size() * sizeof( T ), data.data() );
		}
		
		public :
		~CommandQueue() {
			Release();
		}
		
		CommandQueue( const CommandQueue &cq ) {
			*this = cq;
		}
		
		CommandQueue &operator=( const CommandQueue &cq ) {
			if ( _cmd_queue == cq._cmd_queue ) {
				return *this;
			}
			
			Release();
			
			_cmd_queue = cq._cmd_queue;
			if ( _cmd_queue ) {
				clRetainCommandQueue( _cmd_queue );
			}
			
			return *this;
		}
	};
	
} // CL


#endif
