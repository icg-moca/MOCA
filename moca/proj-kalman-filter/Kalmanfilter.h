namespace Moca{
    //==============================================================================
	//				Kalman Filter for graph nodes(3D points)
	//==============================================================================
	class KalmanFilter {
	public:		
		Eigen::VectorXf m_x;				// state vector
		Eigen::MatrixXf m_P;				// state covariance matrix		
		Eigen::MatrixXf m_F;				// state transistion matrix		
		Eigen::MatrixXf m_Q;				// process covariance matrix		
		Eigen::MatrixXf m_H;				// measurement matrix		
		Eigen::MatrixXf m_R;				// measurement covariance matrix

	public:
		/**
		* @param x = Initial state
		* @param P = Initial state covariance
		* @param F = Transition matrix
		* @param H = Measurement matrix
		* @param R = Measurement covariance matrix
		* @param Q = Process covariance matrix
		*/
		KalmanFilter() {
			m_x = Eigen::VectorXf::Zero(6);
			m_P = Eigen::MatrixXf::Zero(6, 6);
			m_F = Eigen::MatrixXf::Identity(6, 6);

			m_Q = Eigen::MatrixXf::Zero(6, 6);

			m_H = Eigen::MatrixXf::Zero(3, 6);
			m_H <<
				1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f;
			m_R = Eigen::MatrixXf::Identity(3, 3);
			m_R *= 0.1f;
		}

		/**
		* set initial embedded graph node position + velocity (px, py, pz, 0, 0, 0)
		*/
		void SetInitX(const Eigen::VectorXf &x) {
			m_x = x;
		}

		/**
		* Prediction Predicts the state and the state covariance
		* using the process model
		* @param delta_T Time between t and t+1 in s
		*/
		void Predict(float &dt);

		/**
		 * Updates the state by using standard Kalman Filter equations
		 * @param z The measurement at t+1
		 */
		void Update(const Eigen::VectorXf &z);
	};

	inline void KalmanFilter::Predict(float &dt) {
		float noise_ax = 9.0f;
		float noise_ay = 9.0f;
		float noise_az = 9.0f;
		float dt_2 = dt * dt;
		float dt_3 = dt_2 * dt;
		float dt_4 = dt_3 * dt;
		// update Q
		m_Q <<
			dt_4 * 0.25f * noise_ax, 0.0f, 0.0f,	dt_3 *0.5f * noise_ax, 0.0f, 0.0f,
			0.0f, dt_4 * 0.25f * noise_ay, 0.0f,	0.0f, dt_3 *0.5f * noise_ay, 0.0f,
			0.0f, 0.0f, dt_4 * 0.25f * noise_az,	0.0f, 0.0f, dt_3 *0.5f * noise_az,
			dt_3 * 0.5f * noise_ax, 0.0f, 0.0f,		dt_2 * noise_ax, 0.0f, 0.0f,
			0.0f, dt_3 * 0.5f * noise_ay, 0.0f,		0.0f, dt_2 * noise_ay, 0.0f,
			0.0f, 0.0f, dt_3 * 0.5f * noise_az,		0.0f, 0.0f, dt_2 * noise_az;

		// update F
		m_F(0, 3) = dt;
		m_F(1, 4) = dt;
		m_F(2, 5) = dt;

		// predict
		m_x = m_F * m_x;
		MatrixXf Ft = m_F.transpose();
		m_P = m_F * m_P * Ft + m_Q;

		//std::cout << "[predict] Q: " << m_Q << std::endl;
		//std::cout << "[predict] F: " << m_F << std::endl;
	}

	// regular Kalman Filter update
	inline void KalmanFilter::Update(const Eigen::VectorXf & z) {
		Eigen::VectorXf z_pred = m_H * m_x;
		Eigen::VectorXf y = z - z_pred;		// here z = R(Ag + t) + T which is non-linear
		//std::cout << "y:    " << y << std::endl;

		// following is exact the same as in the function of KalmanFilter::Update()
		MatrixXf Ht = m_H.transpose();
		MatrixXf PHt = m_P * Ht;
		MatrixXf S = m_H * PHt + m_R;
		MatrixXf Si = S.inverse();
		MatrixXf K = PHt * Si;

		//std::cout << "[update] Ht: " << K << std::endl;
		//std::cout << "[update] PHt: " << K << std::endl;
		//std::cout << "[update] S: " << K << std::endl;
		//std::cout << "[update] Si: " << K << std::endl;
		//std::cout << "[update] K: " << K << std::endl;

		//new estimate
		m_x = m_x + (K * y);
		MatrixXf I = MatrixXf::Identity(6, 6);
		m_P = (I - K * m_H) * m_P;

		//std::cout << "[update] P: " << K << std::endl;
	}
}; 