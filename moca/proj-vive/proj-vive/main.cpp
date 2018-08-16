#pragma once
#ifndef _VR_RENDER_H
#define _VR_RENDER_H
#include <stdio.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <iostream>

#include "glew.h"
#include "freeglut.h"

#include "openvr.h"

#ifndef _countof
#define _countof(x) (sizeof(x)/sizeof((x)[0]))
#endif

#include <glm\mat4x4.hpp>
#include <glm\vec4.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "plyloader.h"
#include "RenderUtils.h"
#include "GLobject.h"
#include "HMD.h"
using namespace std;

unsigned int screenWidth = 1280;
unsigned int screenHeight = 640;
bool reComplieShader = false;
GLuint GLMocaPointRenderProgram = -1;
GLuint GLdesktopWindowProgram = -1;

//=========================================================================
//		Global Scene
//=========================================================================
GLobject point_cloud_scene;

//=========================================================================
//		HMD object
//=========================================================================
VIVE_HMD vive;
// Conmpainon window 
struct CompWnd {
	GLobject	lefteye;
	GLobject	rigteye;
};
CompWnd companionWnd;

//=========================================================================
//		Draw methods
//=========================================================================
void RenderCompanionWindow(VIVE_HMD &vive) {

	if (!vive.m_HMD) {
		return;
	}
	glDisable(GL_DEPTH_TEST);
	glViewport(0, 0, screenWidth*0.5, screenHeight);

	glUseProgram(GLdesktopWindowProgram);
	glBindVertexArray(companionWnd.lefteye.m_vao);

	// render left eye (first half of index array )
	glBindTexture(GL_TEXTURE_2D, vive.m_leftEye.m_resolveTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glDrawElements(GL_TRIANGLES, companionWnd.lefteye.m_indiceCount, GL_UNSIGNED_SHORT, 0);


	glViewport(screenWidth*0.5, 0, screenWidth*0.5, screenHeight);
	glUseProgram(GLdesktopWindowProgram);
	glBindVertexArray(companionWnd.rigteye.m_vao);

	// render right eye (second half of index array )
	glBindTexture(GL_TEXTURE_2D, vive.m_rightEye.m_resolveTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glDrawElements(GL_TRIANGLES, companionWnd.rigteye.m_indiceCount, GL_UNSIGNED_SHORT, 0);

	glBindVertexArray(0);
	glUseProgram(0);
}

void DrawScene2D() {
	{
		//glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
		glDisable(GL_DEPTH_TEST);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		// Setup viewport, orthographic projection matrix
		glViewport(0, 0, (GLsizei)screenWidth, (GLsizei)screenHeight);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0.0f, screenWidth, 0.0f, screenHeight, -1.0f, +1.0f);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		// Draw a triangle at 0.5 z
		glBegin(GL_TRIANGLES);
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3f(50.5, 50.5, 0.5);
		glVertex3f(550.5, 50.5, 0.5);
		glVertex3f(550.0, 150.5, 0.5);
		glEnd();

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		//glPopAttrib();
	}
}

void DrawGroundScene3D(float *vive_to_eye) {
	{
		// default coord grids
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glMultMatrixf(vive_to_eye);
		{
			glLineWidth(2.0);
			int num = 2 * 7;
			int grad = 1;

			glBegin(GL_LINES);

			for (int z = -num; z <= num; z++) {
				if (z == 0) {
					//x axis
					glColor3f(1, 0, 0);
				}
				else {
					glColor3f(0.2, 0.2, 0.3);
				}
				glVertex3f(-num * grad, 0, z * grad);
				glVertex3f(+num * grad, 0, z * grad);
			}

			for (int x = -num; x <= num; x++) {
				if (x == 0) {
					//z axis
					glColor3f(0, 0, 1);
				}
				else {
					glColor3f(0.2, 0.2, 0.3);
				}
				glVertex3f(x * grad, 0, -num * grad);
				glVertex3f(x * grad, 0, +num * grad);
			}
			glEnd();
		}
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}
}

//=========================================================================
//		HMD draw methods
//=========================================================================
// render global scene
void RenderGlobalScene(float *vive_to_eye, GLuint fbo) {
	
	// draw 3D ground scene
	DrawGroundScene3D(vive_to_eye);

	// draw 3D object scene
	// point cloud data
	if (1) {
		glEnable(GL_PROGRAM_POINT_SIZE);
		glUseProgram(GLMocaPointRenderProgram);
		glm::mat4 model = glm::rotate(45.0f, glm::vec3(0.0, 1.0, 0.0)); // hard code
		model *= glm::translate(glm::vec3(-1.8, 0.0, -1.5));			// hard code (this mat can be replaced by vicon mat (m_to_w = mat_vicon_to_vive * mat_model_to_vicon)
		glUniformMatrix4fv(glGetUniformLocation(GLMocaPointRenderProgram, "model_to_vive"), 1, GL_FALSE, glm::value_ptr(model));
		glUniformMatrix4fv(glGetUniformLocation(GLMocaPointRenderProgram, "vive_to_eye"), 1, GL_FALSE, vive_to_eye);
		glUniform3fv(glGetUniformLocation(GLMocaPointRenderProgram, "color"), 1, glm::value_ptr(glm::vec3(0.0f, 1.0f, 0.0f)));
		glBindVertexArray(point_cloud_scene.m_vao);
		glDrawArrays(GL_POINTS, 0, point_cloud_scene.m_vertCount);
		glBindVertexArray(0);
		glDisable(GL_PROGRAM_POINT_SIZE);
	}
}

//=========================================================================
//		Render
//=========================================================================
void Render(void)
{
	// Get Back to the Modelview
	//glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
	//glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	//glEnable(GL_DEPTH_TEST);

	if (1) {
		vive.DrawToHMD(RenderGlobalScene);
	}
	if (1) {
		RenderCompanionWindow(vive);
	}
	if (0) {
		// draw 2D scene
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		DrawScene2D();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}

	glFlush();
	glFinish();
	glutSwapBuffers();
	glutPostRedisplay();
}
//=========================================================================
//		Reshape
//=========================================================================
void Reshape(int w, int h)
{
	// update screen width and height for imgui new frames
	screenWidth = w;
	screenHeight = h;

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;
	float ratio = 1.0f* w / h;

	glutPostRedisplay();
}
//=========================================================================
//		Update
//=========================================================================
// hhmd
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
void Update(void) {
	if (reComplieShader) {
		GLMocaPointRenderProgram = CompileGLShader("MOCA_PointCloud", "Shaders/PointRender.vs", "Shaders/PointRender.fs");
		vive.GLcontrollerTransformProgram = CompileGLShader("ControllerTransform", "Shaders/Controller.vs", "Shaders/Controller.fs");
		vive.GLHMDdeviceRenderModelProgram = CompileGLShader("RenderVIVEdevice", "Shaders/RenderVIVEdevice.vs", "Shaders/RenderVIVEdevice.fs");
		GLdesktopWindowProgram = CompileGLShader("DesktopWindow", "Shaders/DesktopWindow.vs", "Shaders/DesktopWindow.fs");
		reComplieShader = false;
	}

	vive.UpdateHMDPose();
	vive.VRhandleInput();
}


//=========================================================================
//		keyboard & mouse callback
//=========================================================================
bool keyboardEvent(unsigned char nChar, int nX, int nY)
{

	if (nChar == 27) { //Esc-key
		glutLeaveMainLoop();
		vive.VRshutdown();
		companionWnd.rigteye.Cleanup();
		companionWnd.lefteye.Cleanup();
	}
	if (nChar == 't' || nChar == 'T') {
		screenWidth = 1920;
		screenHeight = 1080;
	}
	if (nChar == 'c') {
		reComplieShader = !reComplieShader;
	}

	return true;
}

void KeyboardSpecial(int key, int x, int y)
{

}

void keyboardCallback(unsigned char nChar, int x, int y)
{
	if (keyboardEvent(nChar, x, y))
	{
		glutPostRedisplay();
	}
}

bool mouseEvent(int button, int state, int x, int y)
{
	//if (state == GLUT_DOWN && (button == GLUT_LEFT_BUTTON))
	//if (state == GLUT_DOWN && (button == GLUT_RIGHT_BUTTON))
	//if (state == GLUT_DOWN && (button == GLUT_MIDDLE_BUTTON))
	return true;
}

void MouseWheel(int button, int dir, int x, int y)
{

	if (dir > 0)
	{

	}
	else if (dir < 0)
	{

	}
}

void MouseCallback(int button, int state, int x, int y)
{
	if (mouseEvent(button, state, x, y))
	{

	}
}

void MouseDragCallback(int x, int y)
{
	glutPostRedisplay();
}

void MouseMoveCallback(int x, int y)
{
	glutPostRedisplay();
}

//=========================================================================
//		Initialize platforms
//=========================================================================
void Init_GLshader(void) {
	GLdesktopWindowProgram = CompileGLShader("DesktopWindow", "Shaders/DesktopWindow.vs", "Shaders/DesktopWindow.fs");
	GLMocaPointRenderProgram = CompileGLShader("MOCA_pointCloud", "Shaders/PointCloudRender.vs", "Shaders/PointCloudRender.fs");
}
// initialize ogl and imgui
void Init_OpenGL(int argc, char **argv, const char* title)
{
	glutInit(&argc, argv);
	glutInitContextVersion(3, 0);
	//glutInitContextFlags(GLUT_FORWARD_COMPATIBLE); // cause error
	glutInitContextProfile(GLUT_CORE_PROFILE);

	//glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);  // reduce speed...
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_MULTISAMPLE);

	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition(200, 200);
	glutCreateWindow(title);
	fprintf(stdout, "INFO: OpenGL Version: %s\n", glGetString(GL_VERSION));

	// glew
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		getchar();
		return;
	}
}

void Init_RenderScene(void) {
	// companion window
	companionWnd.lefteye.Init("LCompanionWindow");
	companionWnd.rigteye.Init("RCompanionWindow");

	// moca
	PLYModel moca_model("../../sample-data/textured_model.ply", 1, 1);
	point_cloud_scene.Init("PointCloud");
	point_cloud_scene.InitBuffer(moca_model.positions, moca_model.normals, moca_model.colors);
}

struct VertexDataWindow
{
	glm::vec2 position;
	glm::vec2 texCoord;

	VertexDataWindow(const glm::vec2 & pos, const glm::vec2 tex) : position(pos), texCoord(tex) {	}
};
void SetupDesktopWindow() {
	// vive check
	std::vector<VertexDataWindow> vVerts;

	// left eye verts
	vVerts.push_back(VertexDataWindow(glm::vec2(-1, -1), glm::vec2(0, 0)));
	vVerts.push_back(VertexDataWindow(glm::vec2(1, -1), glm::vec2(1, 0)));
	vVerts.push_back(VertexDataWindow(glm::vec2(-1, 1), glm::vec2(0, 1)));
	vVerts.push_back(VertexDataWindow(glm::vec2(1, 1), glm::vec2(1, 1)));

	GLushort vIndices[] = { 0, 1, 3, 0, 3, 2 };
	// set left eye 
	companionWnd.lefteye.m_indiceCount = _countof(vIndices);

	glGenVertexArrays(1, &companionWnd.lefteye.m_vao);
	glBindVertexArray(companionWnd.lefteye.m_vao);

	glGenBuffers(1, &companionWnd.lefteye.m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, companionWnd.lefteye.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, vVerts.size() * sizeof(VertexDataWindow), &vVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &companionWnd.lefteye.m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, companionWnd.lefteye.m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, companionWnd.lefteye.m_indiceCount * sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, position));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, texCoord));

	glBindVertexArray(0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	// set right eye
	companionWnd.rigteye.m_indiceCount = _countof(vIndices);

	glGenVertexArrays(1, &companionWnd.rigteye.m_vao);
	glBindVertexArray(companionWnd.rigteye.m_vao);

	glGenBuffers(1, &companionWnd.rigteye.m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, companionWnd.rigteye.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, vVerts.size() * sizeof(VertexDataWindow), &vVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &companionWnd.rigteye.m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, companionWnd.rigteye.m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, companionWnd.rigteye.m_indiceCount * sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, position));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, texCoord));

	glBindVertexArray(0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


void main(int argc, char **argv) {
	Init_OpenGL(argc, argv, "Moca Simple VR render");
	// shaders
	Init_GLshader();
	Init_RenderScene();
	if (vive.Init_HMD()) {
		int skipHMD = 1;
		vive.Init_HMDrenderModel(skipHMD);
		SetupDesktopWindow();
	}
	// callback
	glutDisplayFunc(Render);
	glutReshapeFunc(Reshape);
	glutIdleFunc(Update);
	glutKeyboardFunc(keyboardCallback);
	glutSpecialFunc(KeyboardSpecial);
	glutMouseFunc(MouseCallback);
	glutMouseWheelFunc(MouseWheel);
	glutMotionFunc(MouseDragCallback);
	glutPassiveMotionFunc(MouseMoveCallback);

	glutMainLoop();

	if (vive.m_HMD) {
		vr::VR_Shutdown();
	}
}
#endif // !_VR_RENDER_H


