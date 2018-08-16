#include <stdio.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <iostream>

// GL
#include "glew.h"
#include "freeglut.h"

// glm
#include <glm\mat4x4.hpp>
#include <glm\vec4.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// openvr
#include "openvr.h"

// model loader
#include "plyloader.h"
// local
#include "RenderUtils.h"
#include "GLobject.h"
#include "HMD.h"

// global variables
unsigned int screenWidth = 1280;
unsigned int screenHeight = 640;
bool reComplieShader = false;

VIVE_HMD vive;
// Conmpainon window 
struct CompWnd {
	GLobject	lefteye;
	GLobject	rigteye;
};
CompWnd companionWnd;

GLobject pointCloudScene;

GLuint GLMocaPointRenderProgram = -1;
GLuint GLdesktopWindowProgram = -1;
GLuint GLVRRenderProgram = -1;
#ifndef _countof
#define _countof(x) (sizeof(x)/sizeof((x)[0]))
#endif

/*
void SetupDesktopWindow() {
	// vive check
	std::vector<glm::vec2> verts;
	verts.push_back(glm::vec2(-1, -1));
	verts.push_back(glm::vec2(1, -1));
	verts.push_back(glm::vec2(-1, 1));
	verts.push_back(glm::vec2(1, 1));

	std::vector<glm::vec2> uvs;
	uvs.push_back(glm::vec2(0, 0));
	uvs.push_back(glm::vec2(1, 0));
	uvs.push_back(glm::vec2(0, 1));
	uvs.push_back(glm::vec2(1, 1));

	GLushort vIndices[] = { 0, 1, 3, 0, 3, 2 };
	// set left eye 
	companionWnd.lefteye.m_indiceCount = 6;

	glGenVertexArrays(1, &companionWnd.lefteye.m_vao);
	glBindVertexArray(companionWnd.lefteye.m_vao);

	glGenBuffers(1, &companionWnd.lefteye.m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, companionWnd.lefteye.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(glm::vec2), &verts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &companionWnd.lefteye.m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, companionWnd.lefteye.m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, companionWnd.lefteye.m_indiceCount * sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void *)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void *)0);

	glBindVertexArray(0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	// set right eye
	companionWnd.rigteye.m_indiceCount = 6;

	glGenVertexArrays(1, &companionWnd.rigteye.m_vao);
	glBindVertexArray(companionWnd.rigteye.m_vao);

	glGenBuffers(1, &companionWnd.rigteye.m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, companionWnd.rigteye.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(glm::vec2), &verts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &companionWnd.rigteye.m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, companionWnd.rigteye.m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, companionWnd.rigteye.m_indiceCount * sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void *)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void *)0);

	glBindVertexArray(0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
//*/
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

	// right eye verts
	vVerts.push_back(VertexDataWindow(glm::vec2(0, -1), glm::vec2(0, 0)));
	vVerts.push_back(VertexDataWindow(glm::vec2(1, -1), glm::vec2(1, 0)));
	vVerts.push_back(VertexDataWindow(glm::vec2(0, 1), glm::vec2(0, 1)));
	vVerts.push_back(VertexDataWindow(glm::vec2(1, 1), glm::vec2(1, 1)));

	GLushort vIndices[] = { 0, 1, 3, 0, 3, 2 , 4, 5, 7, 4, 7, 6};
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


void RenderCompanionWindow(VIVE_HMD &vive) {

	if (!vive.m_HMD) {
		return;
	}
	glDisable(GL_DEPTH_TEST);
	glViewport(0, 0, screenWidth*0.5, screenHeight);

	glUseProgram(GLdesktopWindowProgram);
	glBindVertexArray(companionWnd.lefteye.m_vao);

#if 1
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
#endif

	glBindVertexArray(0);
	glUseProgram(0);
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

void RenderScene(float *vive_to_eye, GLuint fbo) {
	DrawGroundScene3D(vive_to_eye);

	glEnable(GL_PROGRAM_POINT_SIZE);
	glUseProgram(GLMocaPointRenderProgram);
	glm::mat4 model = glm::rotate(45.0f, glm::vec3(0.0, 1.0, 0.0)); // hard code
	model *= glm::translate(glm::vec3(-1.8, 0.0, -1.5));			// hard code 
	glUniformMatrix4fv(glGetUniformLocation(GLMocaPointRenderProgram, "model_to_vive"), 1, GL_FALSE, glm::value_ptr(model));
	glUniformMatrix4fv(glGetUniformLocation(GLMocaPointRenderProgram, "vive_to_eye"), 1, GL_FALSE, vive_to_eye);
	glUniform3fv(glGetUniformLocation(GLMocaPointRenderProgram, "color"), 1, glm::value_ptr(glm::vec3(0.0f, 1.0f, 0.0f)));
	glBindVertexArray(pointCloudScene.m_vao);
	glDrawArrays(GL_POINTS, 0, pointCloudScene.m_vertCount);
	glBindVertexArray(0);
	glDisable(GL_PROGRAM_POINT_SIZE);
}

void Render() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	vive.DrawToHMD(RenderScene);
	
	RenderCompanionWindow(vive);

	glFlush();
	glFinish();
	glutSwapBuffers();
	glutPostRedisplay();
}

void Reshape(int w, int h) {

	screenWidth = w;
	screenHeight = h;
	if (h == 0)	h = 1;
	float ratio = 1.0f* w / h;

	glutPostRedisplay();
}

void Update(void) {
	if (reComplieShader) {
		GLVRRenderProgram = CompileGLShader("MOCA_pointCloud", "Shaders/PointRender.vs", "Shaders/PointRender.fs");
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
void keyboardCallback(unsigned char nChar, int x, int y)
{
	if (keyboardEvent(nChar, x, y))
	{
		glutPostRedisplay();
	}
}

int main(int argc, char** argv) {
	// glut 
	glutInit(&argc, argv);
	glutInitContextVersion(3, 0);
	glutInitContextProfile(GLUT_CORE_PROFILE);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_MULTISAMPLE);

	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition(200, 200);
	glutCreateWindow("Moca simple VR viewer");
	fprintf(stdout, "INFO: OpenGL Version: %s\n", glGetString(GL_VERSION));

	// glew
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		getchar();
		return -1;
	}
	GLdesktopWindowProgram = CompileGLShader("DesktopWindow", "Shaders/DesktopWindow.vs", "Shaders/DesktopWindow.fs");
	GLMocaPointRenderProgram = CompileGLShader("MOCA_pointCloud", "Shaders/PointCloudRender.vs", "Shaders/PointCloudRender.fs");
	

	// openvr
	if (vive.Init_HMD()) {
		int skipHMD = 1;
		vive.Init_HMDrenderModel(skipHMD);
		SetupDesktopWindow();
	}

	// prepare scene
	// companion window
	companionWnd.lefteye.Init("LCompanionWindow");
	companionWnd.rigteye.Init("RCompanionWindow");

	// moca

	PLYModel moca_model("../../sample-data/textured_model.ply", true, true);
	pointCloudScene.Init("PointCloud");
	pointCloudScene.InitBuffer(moca_model.positions, moca_model.normals, moca_model.colors);

	// callback
	glutDisplayFunc(Render);
	glutReshapeFunc(Reshape);
	glutIdleFunc(Update);
	glutKeyboardFunc(keyboardCallback);

	glutMainLoop();

	if (vive.m_HMD) {
		vr::VR_Shutdown();
	}

	return 0;
}