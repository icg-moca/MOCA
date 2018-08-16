#pragma once
#ifndef _HMD_VIVE_H
#define _HMD_VIVE_H
#include <iostream>
#include <vector>

void ThreadSleep(unsigned long nMilliseconds)
{
#if defined(_WIN32)
	::Sleep(nMilliseconds);
#elif defined(POSIX)
	usleep(nMilliseconds * 1000);
#endif
}

void PrintGLMmat4(glm::mat4 &mat) {
	printf("%f, %f, %f, %f \n", mat[0][0], mat[0][1], mat[0][2], mat[0][3]);
	printf("%f, %f, %f, %f \n", mat[1][0], mat[1][1], mat[1][2], mat[1][3]);
	printf("%f, %f, %f, %f \n", mat[2][0], mat[2][1], mat[2][2], mat[2][3]);
	printf("%f, %f, %f, %f \n", mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
}
void PrintGLMmat4(glm::mat4 &mat, const char* str) {
	printf("%s: \n", str);
	printf("%f, %f, %f, %f \n", mat[0][0], mat[0][1], mat[0][2], mat[0][3]);
	printf("%f, %f, %f, %f \n", mat[1][0], mat[1][1], mat[1][2], mat[1][3]);
	printf("%f, %f, %f, %f \n", mat[2][0], mat[2][1], mat[2][2], mat[2][3]);
	printf("%f, %f, %f, %f \n", mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
}

class VIVE_HMD {
public:
	// HMD System
	vr::IVRSystem				*m_HMD;
	vr::IVRRenderModels			*m_RenderModels;
	vr::TrackedDevicePose_t		m_TrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
	std::string					m_strDriver;
	std::string					m_strDisplay;
	glm::mat4					m_mat4DevicePose[vr::k_unMaxTrackedDeviceCount];		// get pose of each device
	bool						m_ShowTrackedDevice[vr::k_unMaxTrackedDeviceCount];	// flag for controlling display device

	float hmd_NearClip;
	float hmd_FarClip;

	glm::mat4 m_mat4HMDPose;
	glm::mat4 m_mat4eyePosLeft;
	glm::mat4 m_mat4eyePosRight;

	//glm::mat4 m_mat4ProjectionCenter;
	glm::mat4 m_mat4ProjectionLeft;
	glm::mat4 m_mat4ProjectionRight;


	// HMD GLrender
	// for display
	GLfbo m_leftEye;
	GLfbo m_rightEye;
	uint32_t m_RenderWidth;
	uint32_t m_RenderHeight;

	// for rendering device
	GLuint GLHMDdeviceRenderModelProgram;
	GLuint GLcontrollerTransformProgram;
	GLuint renderModelMatrixLocation;
	GLuint controllerMatrixLocation;

	std::vector<GLVRobject*> vrRenderModels;								// m_vecRenderModels
	GLVRobject *trackedDeviceToRenderModel[vr::k_unMaxTrackedDeviceCount];  // m_rTrackedDeviceToRenderModel

																			// controller line
	GLobject controllerObj;				// what is the purpose of this instance? For drawing controller axis
	unsigned int controllerVertcount;


	// HMD Info
	int m_TrackedControllerCount;
	int m_TrackedControllerCount_Last;
	int m_ValidPoseCount;
	int m_ValidPoseCount_Last;
	std::string m_strPoseClasses;
	char m_DevClassChar[vr::k_unMaxTrackedDeviceCount];
public:
	VIVE_HMD() : controllerVertcount(0){

	}

	// Pipeline
	bool Init_HMD(void) {
		// HMD 
		hmd_NearClip = 0.1f;
		hmd_FarClip = 1000.0f;

		// Loading the SteamVR Runtime
		vr::EVRInitError eError = vr::VRInitError_None;
		m_HMD = vr::VR_Init(&eError, vr::VRApplication_Scene);
		if (eError != vr::VRInitError_None) {
			m_HMD = NULL;
			printf("Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
			return false;
		}

		m_RenderModels = (vr::IVRRenderModels*)vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &eError);
		if (!m_RenderModels) {
			m_HMD = NULL;
			vr::VR_Shutdown();
			printf("Unable to get render model interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
			return false;
		}

		m_strDriver = "No Driver";
		m_strDisplay = "No Display";
		m_strDriver = GetTrackedDeviceString(m_HMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
		m_strDisplay = GetTrackedDeviceString(m_HMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
		printf("HMD driver: %s\n", m_strDriver.c_str());
		printf("HMD display: %s\n", m_strDisplay.c_str());

		if (!InitVRCompositor()) {
			return false;
		}
		printf("HMD init success! \n");

		// Compile shader
		GLcontrollerTransformProgram = CompileGLShader("ControllerTransform", "Shaders/Controller.vs", "Shaders/Controller.fs");
		controllerMatrixLocation = glGetUniformLocation(GLcontrollerTransformProgram, "matrix");
		if (controllerMatrixLocation == -1)
		{
			printf("Unable to find matrix uniform in scene shader\n");
			return false;
		}
		GLHMDdeviceRenderModelProgram = CompileGLShader("RenderVIVEdevice", "Shaders/RenderVIVEdevice.vs", "Shaders/RenderVIVEdevice.fs");
		renderModelMatrixLocation = glGetUniformLocation(GLHMDdeviceRenderModelProgram, "matrix");
		if (renderModelMatrixLocation == -1)
		{
			printf("Unable to find matrix uniform in scene shader\n");
			return false;
		}
		return true;
	}

	// hhmd
	void Init_HMDrenderModel(int skipHMD) {
		SetupCameras();
		SetupStereoRenderTarget();
		SetupHMDdeviceRenderModels(skipHMD);
		controllerObj.Init("Controller");
	}

	// hhmd
	void VRshutdown() {
		// shutdown hmd
		if (m_HMD) {
			vr::VR_Shutdown();
			m_HMD = NULL;
		}
		// clean vrrendermodel
		//for (std::vector< GLVRobject * >::iterator i = vrRenderModels.begin(); i != vrRenderModels.end(); i++)
		{
			//	delete (*i);
		}
		//vrRenderModels.clear();
		// hmd fbo
		CleanFBO(m_leftEye);
		CleanFBO(m_rightEye);

		// irrelevant with vive
		if (GLcontrollerTransformProgram)
		{
			glDeleteProgram(GLcontrollerTransformProgram);
		}
		if (GLHMDdeviceRenderModelProgram)
		{
			glDeleteProgram(GLHMDdeviceRenderModelProgram);
		}

		controllerObj.Cleanup();
	}


	// hhmd
	void DrawToHMD(void(*renderFuc)(float*, GLuint)) {
		if (!m_HMD) {
			return;
		}
		glEnable(GL_MULTISAMPLE);
		//RenderControllerAxes();
		RenderVRStereoTargets(renderFuc);
		glDisable(GL_MULTISAMPLE);

		vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)m_leftEye.m_resolveTex, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
		vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);
		vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)m_rightEye.m_resolveTex, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
		vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);
	}

	// Utils
	glm::mat4 GetHMDMatrixProjectionEye( vr::Hmd_Eye eye ) {
		if (!m_HMD)
			return glm::mat4();

		vr::HmdMatrix44_t mat = m_HMD->GetProjectionMatrix(eye, hmd_NearClip, hmd_FarClip);

		return glm::mat4(
			mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
			mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
			mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
			mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
		);
	}
	// hhmd
	glm::mat4 GetHMDMatrixPoseEye( vr::Hmd_Eye eye )
	{
		if (!m_HMD)
			return glm::mat4();

		vr::HmdMatrix34_t matEyeRight = m_HMD->GetEyeToHeadTransform(eye);
		glm::mat4 matrixObj(
			matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0,
			matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
			matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
			matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f
		);

		return glm::inverse(matrixObj);
	}
	// hmd
	glm::mat4 GetCurrentViewProjectionMatrix(vr::Hmd_Eye eye)
	{
		glm::mat4 matMVP;
		if (eye == vr::Eye_Left)
		{
			matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
		}
		else if (eye == vr::Eye_Right)
		{
			matMVP = m_mat4ProjectionRight * m_mat4eyePosRight *  m_mat4HMDPose;
		}

		return matMVP;
	}

	// hhmd
	void UpdateHMDPose() {
		if (!m_HMD) {
			return;
		}

		vr::VRCompositor()->WaitGetPoses(m_TrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);
		m_ValidPoseCount = 0;
		m_strPoseClasses = "";
		for (int dev = 0; dev < vr::k_unMaxTrackedDeviceCount; dev++) {
			if (m_TrackedDevicePose[dev].bPoseIsValid) {
				m_ValidPoseCount++;
				m_mat4DevicePose[dev] = ConvertSteamVRMatrixToMat4(m_TrackedDevicePose[dev].mDeviceToAbsoluteTracking);
				if (m_DevClassChar[dev] == 0) {
					switch (m_HMD->GetTrackedDeviceClass(dev)) {
					case vr::TrackedDeviceClass_Controller:			m_DevClassChar[dev] = 'C'; break;
					case vr::TrackedDeviceClass_HMD:				m_DevClassChar[dev] = 'H'; break;
					case vr::TrackedDeviceClass_Invalid:			m_DevClassChar[dev] = 'I'; break;
					case vr::TrackedDeviceClass_GenericTracker:		m_DevClassChar[dev] = 'G'; break;
					case vr::TrackedDeviceClass_TrackingReference:	m_DevClassChar[dev] = 'T'; break;
					default:										m_DevClassChar[dev] = '?'; break;
					}
				}
				m_strPoseClasses += m_DevClassChar[dev];
			}
		}

		if (m_TrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
			m_mat4HMDPose = m_mat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
			m_mat4HMDPose = glm::inverse(m_mat4HMDPose);
		}

	}
	
	// hhmd event
	void VRhandleInput() {
		if (!m_HMD) {		// remove this will cause exception
			return;
		}

		// Process SteamVR events
		vr::VREvent_t event;
		while (m_HMD->PollNextEvent(&event, sizeof(event)))
		{
			ProcessVREvent(event);
		}

		// Process SteamVR controller state
		for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
		{
			vr::VRControllerState_t state;
			if (m_HMD->GetControllerState(unDevice, &state, sizeof(state)))
			{
				m_ShowTrackedDevice[unDevice] = state.ulButtonPressed == 0;
			}
		}


		// Spew out the controller and pose count whenever they change.
		if (m_TrackedControllerCount != m_TrackedControllerCount_Last || m_ValidPoseCount != m_ValidPoseCount_Last)
		{
			m_ValidPoseCount_Last = m_ValidPoseCount;
			m_TrackedControllerCount_Last = m_TrackedControllerCount;

			printf("PoseCount:%d(%s) Controllers:%d\n", m_ValidPoseCount, m_strPoseClasses.c_str(), m_TrackedControllerCount);
		}
	}

private:
	// hhmd
	void SetupCameras() {
		// not good, to be fixed
		m_mat4ProjectionLeft = GetHMDMatrixProjectionEye( vr::Eye_Left);
		m_mat4ProjectionRight = GetHMDMatrixProjectionEye( vr::Eye_Right);
		m_mat4eyePosLeft = GetHMDMatrixPoseEye( vr::Eye_Left);
		m_mat4eyePosRight = GetHMDMatrixPoseEye( vr::Eye_Right);

		PrintGLMmat4(m_mat4ProjectionLeft);
		PrintGLMmat4(m_mat4ProjectionRight);
		PrintGLMmat4(m_mat4eyePosLeft);
		PrintGLMmat4(m_mat4eyePosRight);
	}

	// hhmd render
	bool SetupStereoRenderTarget() {
		if (!m_HMD) {
			return false;
		}
		m_HMD->GetRecommendedRenderTargetSize(&m_RenderWidth, &m_RenderHeight);
		CreateFBO(m_leftEye, m_RenderWidth, m_RenderHeight);		// left eye fbo
		CreateFBO(m_rightEye, m_RenderWidth, m_RenderHeight);		// right eye fbo
		return true;
	}

	// hhmd render
	void RenderSceneToEye(vr::Hmd_Eye eye, GLuint fbo, void (*renderFuc)(float*, GLuint)) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		// draw scene for the eye
		{
			glm::mat4 vive_to_eye = GetCurrentViewProjectionMatrix(eye);
			renderFuc(glm::value_ptr(vive_to_eye), fbo);
		}

		bool isSteamVRDrawingControllers = m_HMD->IsSteamVRDrawingControllers();
		if (isSteamVRDrawingControllers) {
			// draw the controller axis lines
			glUseProgram(GLcontrollerTransformProgram);
			glUniformMatrix4fv(controllerMatrixLocation, 1, GL_FALSE, glm::value_ptr(GetCurrentViewProjectionMatrix(eye)));
			glBindVertexArray(controllerObj.m_vao);
			glDrawArrays(GL_LINES, 0, controllerVertcount);  // change this controllerVercount to obj member var?
			glBindVertexArray(0);
		}

		// ----- Render Model rendering ----- (this program should encapsulate int HMD)
		{
			glUseProgram(GLHMDdeviceRenderModelProgram);
			for (uint32_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++)
			{
				if (!trackedDeviceToRenderModel[unTrackedDevice] || !m_ShowTrackedDevice[unTrackedDevice])
					continue;

				const vr::TrackedDevicePose_t & pose = m_TrackedDevicePose[unTrackedDevice];
				if (!pose.bPoseIsValid)
					continue;

				if (!isSteamVRDrawingControllers && m_HMD->GetTrackedDeviceClass(unTrackedDevice) == vr::TrackedDeviceClass_Controller)
					continue;

				const glm::mat4 & matDeviceToTracking = m_mat4DevicePose[unTrackedDevice];
				glm::mat4 matMVP = GetCurrentViewProjectionMatrix(eye) * matDeviceToTracking;
				glUniformMatrix4fv(renderModelMatrixLocation, 1, GL_FALSE, glm::value_ptr(matMVP));

				trackedDeviceToRenderModel[unTrackedDevice]->Draw();
			}
			glUseProgram(0);
		}
	}

	// hhmd render
	void RenderVRStereoTargets(void(*renderFuc)(float*, GLuint)) {
		glClearColor(0.35f, 0.35f, 0.36f, 1.0f);
		//glClearColor(0.925f, 0.945f, 0.945f, 1.0f);
		glEnable(GL_MULTISAMPLE);
		glBindFramebuffer(GL_FRAMEBUFFER, m_leftEye.m_framebuffer);
		glViewport(0, 0, m_RenderWidth, m_RenderHeight);
		//glClearColor(1.f, 0.f, 0.f, 1.0f);
		RenderSceneToEye(vr::Eye_Left, m_leftEye.m_framebuffer, renderFuc);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glDisable(GL_MULTISAMPLE);

		glBindFramebuffer(GL_READ_FRAMEBUFFER, m_leftEye.m_framebuffer);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_leftEye.m_resolveFramebuffer);
		glBlitFramebuffer(0, 0, m_RenderWidth, m_RenderHeight, 0, 0, m_RenderWidth, m_RenderHeight,
			GL_COLOR_BUFFER_BIT, GL_LINEAR);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);


		// right eye
		glEnable(GL_MULTISAMPLE);
		glBindFramebuffer(GL_FRAMEBUFFER, m_rightEye.m_framebuffer);
		glViewport(0, 0, m_RenderWidth, m_RenderHeight);
		//glClearColor(0.f, 1.f, 0.f, 1.0f);
		RenderSceneToEye(vr::Eye_Right, m_rightEye.m_framebuffer, renderFuc);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glDisable(GL_MULTISAMPLE);

		glBindFramebuffer(GL_READ_FRAMEBUFFER, m_rightEye.m_framebuffer);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_rightEye.m_resolveFramebuffer);
		glBlitFramebuffer(0, 0, m_RenderWidth, m_RenderHeight, 0, 0, m_RenderWidth, m_RenderHeight,
			GL_COLOR_BUFFER_BIT, GL_LINEAR);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	}

	void RenderControllerAxes() {
		// don't draw controllers if somebody else has input focus
		if (!m_HMD->IsSteamVRDrawingControllers())
			return;

		std::vector<float> vertdataarray;
		controllerVertcount = 0;
		m_TrackedControllerCount = 0;
		for (vr::TrackedDeviceIndex_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; ++unTrackedDevice)
		{
			if (!m_HMD->IsTrackedDeviceConnected(unTrackedDevice))
				continue;

			if (m_HMD->GetTrackedDeviceClass(unTrackedDevice) != vr::TrackedDeviceClass_Controller)
				continue;

			m_TrackedControllerCount += 1;

			if (!m_TrackedDevicePose[unTrackedDevice].bPoseIsValid)
				continue;

			const glm::mat4 & mat = m_mat4DevicePose[unTrackedDevice];

			glm::vec4 center = mat * glm::vec4(0, 0, 0, 1);
			//printf("Controller center: %f, %f, %f\n", center.x, center.y, center.z);

			for (int i = 0; i < 3; ++i)
			{
				glm::vec3 color(0, 0, 0);
				glm::vec4 point(0, 0, 0, 1);
				point[i] += 0.05f;  // offset in X, Y, Z
				color[i] = 1.0;  // R, G, B
				point = mat * point;
				vertdataarray.push_back(center.x);
				vertdataarray.push_back(center.y);
				vertdataarray.push_back(center.z);

				vertdataarray.push_back(color.x);
				vertdataarray.push_back(color.y);
				vertdataarray.push_back(color.z);

				vertdataarray.push_back(point.x);
				vertdataarray.push_back(point.y);
				vertdataarray.push_back(point.z);

				vertdataarray.push_back(color.x);
				vertdataarray.push_back(color.y);
				vertdataarray.push_back(color.z);

				controllerVertcount += 2;
			}

			glm::vec4 start = mat * glm::vec4(0, 0, -0.02f, 1);
			glm::vec4 end = mat * glm::vec4(0, 0, -39.f, 1);
			glm::vec3 color(.92f, .92f, .71f);

			vertdataarray.push_back(start.x); vertdataarray.push_back(start.y); vertdataarray.push_back(start.z);
			vertdataarray.push_back(color.x); vertdataarray.push_back(color.y); vertdataarray.push_back(color.z);

			vertdataarray.push_back(end.x); vertdataarray.push_back(end.y); vertdataarray.push_back(end.z);
			vertdataarray.push_back(color.x); vertdataarray.push_back(color.y); vertdataarray.push_back(color.z);
			controllerVertcount += 2;
		}

		// Setup the VAO the first time through.
		if (controllerObj.m_vao == 0)
		{
			glGenVertexArrays(1, &controllerObj.m_vao);
			glBindVertexArray(controllerObj.m_vao);

			glGenBuffers(1, &controllerObj.m_vbo);
			glBindBuffer(GL_ARRAY_BUFFER, controllerObj.m_vbo);

			GLuint stride = 2 * 3 * sizeof(float);
			uintptr_t offset = 0;

			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

			offset += sizeof(glm::vec3);
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

			glBindVertexArray(0);
		}

		glBindBuffer(GL_ARRAY_BUFFER, controllerObj.m_vbo);

		// set vertex data if we have some
		if (vertdataarray.size() > 0)
		{
			//$ TODO: Use glBufferSubData for this...
			glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW);
		}
	}

	// hhmd render
	// Finds a render model we've already loaded or loads a new one
	GLVRobject *FindOrLoadRenderModel(const char *renderModelName) {
		GLVRobject *rendermodel = NULL;
		for (std::vector< GLVRobject * >::iterator i = vrRenderModels.begin(); i != vrRenderModels.end(); i++)
		{
			if (!_stricmp((*i)->GetName().c_str(), renderModelName))
			{
				rendermodel = *i;
				break;
			}
		}

		// load the model if we didn't find one
		if (!rendermodel)
		{
			vr::RenderModel_t *pModel;
			vr::EVRRenderModelError error;
			while (1)
			{
				error = vr::VRRenderModels()->LoadRenderModel_Async(renderModelName, &pModel);
				if (error != vr::VRRenderModelError_Loading)
					break;

				ThreadSleep(1);
			}

			if (error != vr::VRRenderModelError_None)
			{
				printf("Unable to load render model %s - %s\n", renderModelName, vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));
				return NULL; // move on to the next tracked device
			}

			vr::RenderModel_TextureMap_t *pTexture;
			while (1)
			{
				error = vr::VRRenderModels()->LoadTexture_Async(pModel->diffuseTextureId, &pTexture);
				if (error != vr::VRRenderModelError_Loading)
					break;

				ThreadSleep(1);
			}

			if (error != vr::VRRenderModelError_None)
			{
				printf("Unable to load render texture id:%d for render model %s\n", pModel->diffuseTextureId, renderModelName);
				vr::VRRenderModels()->FreeRenderModel(pModel);
				return NULL; // move on to the next tracked device
			}

			rendermodel = new GLVRobject(renderModelName);
			if (!rendermodel->InitBuffer(*pModel, *pTexture))
			{
				printf("Unable to create GL model from render model %s\n", renderModelName);
				delete rendermodel;
				rendermodel = NULL;
			}
			else
			{
				vrRenderModels.push_back(rendermodel);
			}
			vr::VRRenderModels()->FreeRenderModel(pModel);
			vr::VRRenderModels()->FreeTexture(pTexture);
		}
		return rendermodel;
	}

	// hhmd update helper
	glm::mat4 ConvertSteamVRMatrixToMat4(const vr::HmdMatrix34_t &matPose)
	{
		glm::mat4 matrixObj(
			matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
			matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
			matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
			matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
		);
		return matrixObj;
	}

	// hhmd event
	void ProcessVREvent(const vr::VREvent_t &event) {
		switch (event.eventType)
		{
		case vr::VREvent_TrackedDeviceActivated:
		{
			SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
			printf("Device %u attached. Setting up render model.\n", event.trackedDeviceIndex);
		}
		break;
		case vr::VREvent_TrackedDeviceDeactivated:
		{
			printf("Device %u detached.\n", event.trackedDeviceIndex);
		}
		break;
		case vr::VREvent_TrackedDeviceUpdated:
		{
			printf("Device %u updated.\n", event.trackedDeviceIndex);
		}
		break;
		// Controller
		case vr::VREvent_ButtonPress: // data is controller
		{

		}
		break;
		case vr::VREvent_ButtonUnpress: // data is controller
		{

		}
		break;
		case vr::VREvent_ButtonTouch: // data is controller
		{

		}
		break;
		case vr::VREvent_ButtonUntouch: // data is controller
		{

		}
		break;
		}
	}

	// hhmd render
	// Create/destroy GL a Render Model for a single tracked device
	void SetupRenderModelForTrackedDevice(vr::TrackedDeviceIndex_t devID) {
		if (devID >= vr::k_unMaxTrackedDeviceCount) {
			return;
		}

		std::string renderModelName = GetTrackedDeviceString(m_HMD, devID, vr::Prop_RenderModelName_String);
		printf("device name string: %s\n", renderModelName.c_str());
		GLVRobject *vrobj = FindOrLoadRenderModel(renderModelName.c_str());
		if (!vrobj) {
			std::string sTrackingSystemName = GetTrackedDeviceString(m_HMD, devID, vr::Prop_TrackingSystemName_String);
			printf("Unable to load render model for tracked device %d (%s.%s)", devID, sTrackingSystemName.c_str(), renderModelName.c_str());
		}
		else
		{
			trackedDeviceToRenderModel[devID] = vrobj;
			m_ShowTrackedDevice[devID] = true;
		}
	}
	// hhmd render
	// Create/destroy GL Render Models
	void SetupHMDdeviceRenderModels(int skipHMD) {
		memset(trackedDeviceToRenderModel, 0, sizeof(trackedDeviceToRenderModel));

		if (!m_HMD)
			return;
		// + 1 for skipping headset
		for (uint32_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + skipHMD; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++)
		{
			if (!m_HMD->IsTrackedDeviceConnected(unTrackedDevice))
				continue;

			SetupRenderModelForTrackedDevice(unTrackedDevice);
		}
	}

	bool InitVRCompositor() {
		vr::EVRInitError peError = vr::VRInitError_None;

		if (!vr::VRCompositor())
		{
			printf("Compositor initialization failed. See log file for details\n");
			return false;
		}

		return true;
	}

	// hhmd helper
	std::string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
	{
		uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
		if (unRequiredBufferLen == 0)
			return "";

		char *pchBuffer = new char[unRequiredBufferLen];
		unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
		std::string sResult = pchBuffer;
		delete[] pchBuffer;
		return sResult;
	}
};



#endif // !_HMD_VIVE_H