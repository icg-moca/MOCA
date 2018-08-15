

class GlobalRegistrationICP {
public:
	// linear euqation solver
	EPH_List<cl_float8> buffer_j0, buffer_j1;
	CL::Mem mem_j0, mem_j1, mem_A;

	// program
	CL::Program program;
	CL::Kernel kn_jacobian, kn_reduction_j, kn_reduction_E;

public:
	void CreateBuffer(CL::Context &context) {
		// mem
		int total = 512 * 424;

		mem_j0.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float8));
		mem_j1.CreateBuffer(context, CL_MEM_READ_WRITE, total * sizeof(cl_float8));

		mem_A.CreateBuffer(context, CL_MEM_READ_WRITE, sizeof(cl_float8));
	}

	void CompileProgram(CL::Context &context) {
		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros";
		EPH_TextFile source;
		source.Load(EPH_FileSystem::CurrentPath() + "/shader/icp.txt");
		program.Create(context, source.Text(), flags);

		// kernel
		kn_jacobian.Create(program, "jacobian");
		kn_reduction_j.Create(program, "reduction_j");
		kn_reduction_E.Create(program, "reduction_E");
	}

	void EvalJr(
		CL::CommandQueue &cq,
		int k0, Sensor &sensor0, SensorMap &sensorMap0,
		int k1, Sensor &sensor1, SensorMap &sensorMap1,
		Eigen::MatrixXf &A, Eigen::VectorXf &b, float &E, float &N
		) {
		if (!sensorMap0.isValid || !sensorMap1.isValid) {
			return;
		}

		int total = 512 * 424;

		cq.NDRangeKernel<1>(
			(kn_jacobian <<
			mem_j0, mem_j1,
			sensorMap0.mem_p, sensorMap0.mem_n, sensorMap0.mem_c,
			sensorMap1.mem_p, sensorMap1.mem_n, sensorMap1.mem_c,
			sensor0.extr.Transpose(),
			(inv_rigid_transform(sensor1.extr) * sensor0.extr).Transpose(),
			sensor1.intr,
			total
			),
			total, 256
			);

		Eigen::MatrixXf J00(6, 6), J11(6, 6), J01(6, 6);
		Eigen::VectorXf J0r(6), J1r(6);

		J00 = Eigen::MatrixXf::Zero(6, 6);
		J11 = Eigen::MatrixXf::Zero(6, 6);
		J01 = Eigen::MatrixXf::Zero(6, 6);
		J0r = Eigen::VectorXf::Zero(6);
		J1r = Eigen::VectorXf::Zero(6);

		for (int i = 0; i < 6; i++) {
			cq.NDRangeKernel<1>((kn_reduction_j << mem_j0, mem_j0, i, mem_A, total), 256, 256);

			cl_float8 J;
			cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

			for (int k = 0; k < 6; k++) { J00(i, k) = J.s[k]; }
		}

		for (int i = 0; i < 6; i++) {
			cq.NDRangeKernel<1>((kn_reduction_j << mem_j1, mem_j1, i, mem_A, total), 256, 256);

			cl_float8 J;
			cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

			for (int k = 0; k < 6; k++) { J11(i, k) = J.s[k]; }
		}

		for (int i = 0; i < 6; i++) {
			cq.NDRangeKernel<1>((kn_reduction_j << mem_j0, mem_j1, i, mem_A, total), 256, 256);

			cl_float8 J;
			cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

			for (int k = 0; k < 6; k++) { J01(i, k) = J.s[k]; }
		}

		{
			cq.NDRangeKernel<1>((kn_reduction_j << mem_j1, mem_j0, 6, mem_A, total), 256, 256);

			cl_float8 J;
			cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

			for (int k = 0; k < 6; k++) { J0r(k) = J.s[k]; }
		}

		{
			cq.NDRangeKernel<1>((kn_reduction_j << mem_j0, mem_j1, 6, mem_A, total), 256, 256);

			cl_float8 J;
			cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

			for (int k = 0; k < 6; k++) { J1r(k) = J.s[k]; }
		}

		{
			cq.NDRangeKernel<1>((kn_reduction_E << mem_j0, mem_A, total), 256, 256);

			cl_float8 J;
			cq.ReadBuffer(mem_A, CL_TRUE, 0, sizeof(J), &J);

			E += J.s0;
			N += J.s1;
		}

		A.block(6 * k0, 6 * k0, 6, 6) += J00;
		A.block(6 * k1, 6 * k1, 6, 6) += J11;
		A.block(6 * k0, 6 * k1, 6, 6) += J01;
		A.block(6 * k1, 6 * k0, 6, 6) += J01.transpose();

		b.block(6 * k0, 0, 6, 1) += J0r;
		b.block(6 * k1, 0, 6, 1) += J1r;
	}

	void ICP(CL::CommandQueue &cq, EPH_List<Sensor> &sensors, EPH_List<SensorMap> &sensorMaps, int origin) {
		Eigen::MatrixXf A = Eigen::MatrixXf::Identity(6 * sensors.Num(), 6 * sensors.Num()) * 0.0001f; // FIXME
		Eigen::VectorXf b = Eigen::VectorXf::Zero(6 * sensors.Num());
		Eigen::VectorXf x = Eigen::VectorXf::Zero(6 * sensors.Num());
		float E = 0;
		float N = 0;

		for (int i = 0; i < sensors.Num(); i++) {
			for (int j = 0; j < sensors.Num(); j++) {
				if (i == j) {
					continue;
				}
				EvalJr(
					cq,
					i, sensors[i], sensorMaps[i],
					j, sensors[j], sensorMaps[j],
					A, b, E, N
					);
			}
		}

		x = A.fullPivHouseholderQr().solve(b);

		std::cout << "E : " << N << " : " << sqrt(E / (float)(N)) * 1000.0f << std::endl;

		EPH_Mat4 inv = sensors[origin].extr * convert_6d_to_mat(x.block(origin * 6, 0, 6, 1)) * inv_rigid_transform(sensors[origin].extr);

		for (int i = 0; i < sensors.Num(); i++) {
			if (i != origin) {
				sensors[i].extr = inv * sensors[i].extr * inv_rigid_transform(convert_6d_to_mat(x.block(i * 6, 0, 6, 1)));

				EPH_Mat3 m = sensors[i].extr.Block<3, 3>(0, 0);
				m.Normalize();

				sensors[i].extr = m.CatRow(EPH_Vec3(0)).CatCol(sensors[i].extr.Column(3));
			}
		}
	}
};