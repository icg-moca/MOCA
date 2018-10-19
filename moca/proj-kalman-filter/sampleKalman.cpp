#include "Kalmanfitler.h"

void testKalman(void){
    if (m_gpuKernel == nullptr) {
		printf(">> GPU Kernel pointer is null\n");
		return;
	}

    // 0. set Kalman filter initial pos for each node  (call during initialization)
    Eigen::VectorXf x(6);
    x << m_canonicalModel.points[m_poiID].x(),m_canonicalModel.points[m_poiID].y(),m_canonicalModel.points[m_poiID].z(), 0.0, 0.0, 0.0;
    md_kalmanFilter.setInit(x);


	m_gpuKernel->BackProjectPoints(m_sensors[md_testSensorID].cali_ir.intr.IntrVec(), m_sensors[md_testSensorID].dep_to_gl,
		(unsigned short*)m_frames[md_testSensorID].dep_smooth.ptr(), md_partialPointCloud.points, md_partialPointCloud.normals);

	CreateGLmem(md_partialGLmem, md_partialPointCloud, true);
	printf("sensor id: %d, frame id: %d \n", md_testSensorID, m_stream.m_curFrame);


	// project canonical model point to sensor image plane (ONly for vis no relative to Kalman Filter)
	ProjCanonicalPointsToCam(md_partialPointCloud.points, md_partialPointCloud.normals);

	float dt = 0.33f;
	if (m_stream.m_curFrame > 0) dt = (m_stream.m_timeSeqs[md_testSensorID][m_stream.m_curFrame] - m_stream.m_timeSeqs[md_testSensorID][m_stream.m_curFrame - 1]) * 0.001f;
    std::cout << "DT: " << dt << std::endl;
	
    // 1. predict
	md_kalmanFilter.Predict(dt);
	
	// already in test sensor ID world space (but normal is unknown)
	Eigen::Vector4f predictP;
	predictP << md_kalmanFilter.m_x(0), md_kalmanFilter.m_x(1), md_kalmanFilter.m_x(2), 1.0f;
	Eigen::Matrix4f gl2cam;
	cv::cv2eigen(m_sensors[md_testSensorID].dep_to_gl.inv(), gl2cam);
	const float fx = (float)m_sensors[md_testSensorID].cali_ir.intr.cameraMatrix.at<double>(0, 0);
	const float fy = (float)m_sensors[md_testSensorID].cali_ir.intr.cameraMatrix.at<double>(1, 1);
	const float cx = (float)m_sensors[md_testSensorID].cali_ir.intr.cameraMatrix.at<double>(0, 2);
	const float cy = (float)m_sensors[md_testSensorID].cali_ir.intr.cameraMatrix.at<double>(1, 2);
	// search in world space
	SearchCorrespondence(m_predictDepthID, md_kalmanFilter, gl2cam, fx, fy, cx, cy,
		md_partialPointCloud.points, md_partialPointCloud.normals); 
    Eigen::VectorXf measureP(3);
	measureP << md_partialPointCloud.points[m_predictDepthID].x(), md_partialPointCloud.points[m_predictDepthID].y(), md_partialPointCloud.points[m_predictDepthID].z();

	// 2. update phase
	md_kalmanFilter.Update(measureP);
	m_updatedNodePos << md_kalmanFilter.m_x(0), md_kalmanFilter.m_x(1), md_kalmanFilter.m_x(2);		// "true" correspondence
	std::cout << " Predicted Point KF:  " << predictP.transpose() << std::endl;
	std::cout << " Updated Point KF:  "   << m_updatedNodePos.transpose() << std::endl;
}