
class SensorMapProcess {
public:
	// program
	CL::Program program;

	// kernel
	CL::Kernel kn_d2p;
	CL::Kernel kn_p2n;

public:
	void CreateBuffer(CL::Context &context, SensorMap &sensorMap) {
		int total = 512 * 424;

		sensorMap.mem_d.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_ushort));
		sensorMap.mem_p.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float4));
		sensorMap.mem_n.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float4));
		sensorMap.mem_c.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float4));
	}

	void CompileProgram(CL::Context &context) {
		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		EPH_TextFile source;
		source.Load(EPH_FileSystem::CurrentPath() + "/shader/icp.txt");
		program.Create(context, source.Text(), flags);

		// kernel
		kn_d2p.Create(program, "depth_to_point");
		kn_p2n.Create(program, "point_to_normal");
	}

	void BackProjection(CL::CommandQueue &cq, Sensor &sensor, SensorMap &sensorMap) {
		cq.WriteBuffer(sensorMap.mem_d, CL_TRUE, 0, sensorMap.depth.BufferSize(), sensorMap.depth.Data());

		cq.NDRangeKernel<2>((kn_d2p << EPH_Vec2i(512, 424), sensorMap.mem_d, sensorMap.mem_p, sensor.intr), 512, 16);
		cq.NDRangeKernel<2>((kn_p2n << EPH_Vec2i(512, 424), sensorMap.mem_p, sensorMap.mem_n), 512, 16);

		cq.ReadBuffer(sensorMap.mem_p, CL_TRUE, sensorMap.points.Pixels());
		cq.ReadBuffer(sensorMap.mem_n, CL_TRUE, sensorMap.normals.Pixels());
	}

	void ColorProjection(CL::CommandQueue &cq, Sensor &sensor_ir, Sensor &sensor_rgb, SensorMap &sensorMap, Image<EPH_RGBA8> &image) {
		EPH_Mat4 M = inv_rigid_transform(sensor_rgb.extr) * sensor_ir.extr;
		for (int i = 0; i < sensorMap.points.NumPixels(); i++) {
			EPH_Vec4 v = M * sensorMap.points[i].Sub<3>().Cat(1);

			if (v.z <= 0) {
				continue;
			}

			v.x /= v.z;
			v.y /= v.z;

			v.x *= sensor_rgb.intr[0];
			v.y *= sensor_rgb.intr[1];

			v.x += sensor_rgb.intr[2];
			v.y += sensor_rgb.intr[3];

			if (v.x < 0 || v.x >= 1920 || v.y < 0 || v.y >= 1080) {
				sensorMap.colors[i].Set(0, 0, 0, 0);
				continue;
			}
			sensorMap.colors[i] = image(1920 - 1 - (int)v.x, (1080 - 1 - (int)(v.y))).ToVec4().Swap(0, 2);
		}

		cq.WriteBuffer(sensorMap.mem_c, CL_TRUE, sensorMap.colors.Pixels());
	}
};