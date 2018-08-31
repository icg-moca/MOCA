#pragma once
// GL dependencies
#include <GL/glew.h>   
#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <Glut/glut.h>
#else
#include <GL\glut.h>
#endif
// Math
// Customized
#include "Include\Camera.h"


using namespace std;

// ----- Render -----
// constants
const float CAMERA_DISTANCE = 50.0f;
const float FOV = 90.0f;
const float NEAR_CLIPPING = 0.1f;
const float FAR_CLIPPING = 20000.0f;
const int G_SCREEN_WIDTH = 1280;
const int G_SCREEN_HEIGHT = 800;
const float RAY_MAXIMUN_DIST = 600.0f;
/// Camera global configuration ///
Camera camera;
bool mouseLeftDown; bool mouseRightDown;
int mouse_x = G_SCREEN_WIDTH / 2, mouse_y = G_SCREEN_HEIGHT / 2;
float offsetx = 0.0f, offsety = 0.0f;

bool showKinect = false;
bool showVicon = false;
bool showVelo = true;

//===============================================
// velodyne
//===============================================
Lidar::HDLOfflineCapture						capture;
std::vector<Lidar::Laser>						vLasers;
std::vector<glm::vec3>								vPoints;
float delta = 0.0f;
glm::mat4 vtrans = glm::mat4(1.0f);

void TranslateX(float delta) {
	glm::mat4 translate = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, delta, 0.0f));
	vtrans = translate * vtrans;
}
void TranslateZ(float delta) {
	glm::mat4 translate = glm::translate(glm::mat4(1.0f), glm::vec3(delta, 0.0f, 0.0f));
	vtrans = translate * vtrans;
}
void RotateAboutY(float rad) {
	glm::vec3 scale;
	glm::quat rotation;
	glm::vec3 translation;
	glm::vec3 skew;
	glm::vec4 perspective;
	glm::decompose(vtrans, scale, rotation, translation, skew, perspective);

	vtrans = glm::translate(glm::mat4(1.0f), translation) * glm::rotate(glm::mat4(1.0f), rad, glm::vec3(0.0f, 0.0f, 1.0f)) * glm::translate(glm::mat4(1.0f), -translation) * vtrans;
}
void SaveVeloMat(void) {
	FILE *fp = fopen("velo_cali.txt", "w");
	if (!fp) {
		return;
	}

	// kinect to camera
	for (int i = 0; i < 16; i++) {
		fprintf(fp, "%f ", (glm::value_ptr(vtrans)[i]));
	}
	fprintf(fp, "\n");

	fclose(fp);
	fp = NULL;
}

// Initialization
void InitVelo(void) {
	using namespace Lidar;
	const std::string califile = "xml/HDL-64E.xml";
	//const std::string filename = "HDLData/CapturedData/2015-08-27-second.pcap";
	const std::string filename = "HDLData/CapturedData/2017-12-19-14-15-23_Velodyne-HDL-64-Data.pcap";
	HDLOfflineCapture capture;
	capture.Init(califile);
	capture.OpenFile(filename);

	capture.ProcessPacket();

	//capture.RetrievePacket(vLasers);
	//while (vLasers[0].azimuth >= 1.0) {		// find first round with 360 degree
	//	capture.RetrievePacket(vLasers);
	//}

	capture.RetrieveFramePacket(vLasers, 10, false);

	ProcessLaserData(vLasers, capture.mCorrection, vPoints);
	for (int i = 0; i < vPoints.size(); i++) {
		vPoints[i] = vPoints[i] + glm::vec3(0.0, 0.0, 1.0);
	}
}


//====================================================================================================
// init
//====================================================================================================
void init(void) {

	// velodyne
	InitVelo();

	// kinect
	InitKinect();

	// vicon
	InitViconMarkers();

	Vec3f pos(0, 5, 5); Vec3f up(0, 1, 0); Vec3f target(0, 0, 0);
	camera.SetParam(pos, up, target);
}


//====================================================================================================
// Controls
//====================================================================================================
//============================================================================================
// keyboard input
//============================================================================================
void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case '27':
	{
		exit(0);
	}
	case'w':
	{
		camera.MoveForward(0.5);
		break;
	}
	case 's':
	{
		camera.MoveForward(-0.5);
		break;
	}
	case 'a':
	{
		camera.MoveLeftRight(0.1);
		break;
	}
	case 'd':
	{
		camera.MoveLeftRight(-0.1);
		break;
	}

	// manage calibration
	case 'f': // left
	{
		KTranslateX(delta);
		std::cout << glm::to_string(ktrans) << std::endl;
		break;
	}
	case 'h': // right
	{
		KTranslateX(-delta);
		std::cout << glm::to_string(ktrans) << std::endl;
		break;
	}
	case 't': // forwad
	{
		KTranslateZ(delta);
		std::cout << glm::to_string(ktrans) << std::endl;
		break;
	}
	case 'g': // backward
	{
		KTranslateZ(-delta);
		std::cout << glm::to_string(ktrans) << std::endl;
		break;
	}
	case 'r': // rotate counterclock
	{
		KRotateAboutY(HDL_Grabber_toRadians(1.0f));
		std::cout << glm::to_string(ktrans) << std::endl;
		break;
	}
	case 'y': // rotate clock
	{
		KRotateAboutY(HDL_Grabber_toRadians(-1.0f));
		std::cout << glm::to_string(ktrans) << std::endl;
		break;
	}
	// manage calibration
	case 'j': // left
	{
		TranslateX(delta);
		std::cout << glm::to_string(vtrans) << std::endl;
		break;
	}
	case 'l': // right
	{
		TranslateX(-delta);
		std::cout << glm::to_string(vtrans) << std::endl;
		break;
	}
	case 'i': // forwad
	{
		TranslateZ(delta);
		std::cout << glm::to_string(vtrans) << std::endl;
		break;
	}
	case 'k': // backward
	{
		TranslateZ(-delta);
		std::cout << glm::to_string(vtrans) << std::endl;
		break;
	}
	case 'u': // rotate counterclock
	{
		RotateAboutY(HDL_Grabber_toRadians(1.0f));
		std::cout << glm::to_string(vtrans) << std::endl;
		break;
	}
	case 'o': // rotate clock
	{
		RotateAboutY(HDL_Grabber_toRadians(-1.0f));
		std::cout << glm::to_string(vtrans) << std::endl;
		break;
	}
	case '9': // add cm
	{
		delta += 0.01f;
		printf("delta : %f\n", delta);
		break;
	}
	case '0': // minus cm
	{
		delta -= 0.01f;
		printf("delta : %f\n", delta);
		break;
	}
	case '-': // add mm
	{
		delta += 0.001f;
		printf("delta : %f\n", delta);
		break;
	}
	case '=': // minus mm
	{
		delta -= 0.001f;
		printf("delta : %f\n", delta);
		break;
	}
	case 'p': { // reset delta
		delta = 0.0f;
		printf("reset delta : %f\n", delta);
		break;
	}
	case '8': {
		SaveVeloMat();
		printf("save velo cali mat done... \n");
		break;
	}

	case '1': {
		showVelo = !showVelo;
		break;
	}
	}
	glutPostRedisplay();
}
void mouse(int button, int state, int x, int y)
{

	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			camera.m_nPrevX = x;
			camera.m_nPrevY = y;
			mouseLeftDown = true;
		}
		else if (state == GLUT_UP)
			mouseLeftDown = false;
	}

	else if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouseRightDown = true;
		}
		else if (state == GLUT_UP)
			mouseRightDown = false;
	}
}
void mouseMotion(int x, int y)
{
	if (mouseLeftDown)
	{
		camera.RotateMouse(x, y);
		//camera.Update();
	}
	if (mouseRightDown)
	{
		offsety = (mouse_y - y);
		mouse_y = y;
		GLfloat sensitivity = 0.015f;
		offsety *= sensitivity;
		camera.MoveForward(offsety);
	}
	glutPostRedisplay();
}

//====================================================================================================
// update
//====================================================================================================
void update(void) {


}

//====================================================================================================
// render
//====================================================================================================

void render(void) {
	// clear buffer
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	//glShadeModel(GL_SMOOTH);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(camera.m_position.x, camera.m_position.y, camera.m_position.z,			//eye(x,y,z)
		camera.m_target.x, camera.m_target.y, camera.m_target.z,						//focal(x,y,z)
		camera.m_up.x, camera.m_up.y, camera.m_up.z);									//up(0,1,0)

	// render point cloud
	// draw Velodyne Points
	glEnable(GL_POINT_SIZE);
	glPointSize(3.0);
	if (showVelo) {
		glBegin(GL_POINTS);
		glColor3f(1, 0, 0);
		glVertex3f(0, 0.01, 0);
		glColor3f(0.855, 0.439, 0.839);
		for (int i = 0; i < vPoints.size(); i++) {
			glm::vec3 p = glm::vec3(vtrans * glm::vec4(vPoints[i], 1.0f));
			//glVertex3f(-vPoints[i].y, vPoints[i].z, -vPoints[i].x);
			glVertex3f(-p.y, p.z, -p.x);
		}
		glEnd();
	}

	// draw grid
	float HDLheight = 2 * tanf(24.8f);
	glLineWidth(2);
	glBegin(GL_LINES);
	glColor3f(0.2, 0.2, 0.3);
	for (float i = -5; i <= +5; i += 0.1f) {
		glVertex3f(i, HDLheight, -5);
		glVertex3f(i, HDLheight, +5);

		glVertex3f(-5, HDLheight, i);
		glVertex3f(+5, HDLheight, i);
	}
	glEnd();

	// swap back and front buffers
	glutSwapBuffers();
}

//====================================================================================================
// reshape : update viewport and projection matrix when the window is resized
//====================================================================================================

void reshape(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;

	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (GLfloat)w / (GLfloat)h, NEAR_CLIPPING, FAR_CLIPPING); //FOV, ASPECT, NEAR CLIPPING, FAR CLIPPING

	glViewport(0, 0, w, h);
	printf("wh : %d %d\n", w, h);
	// Get Back to the Modelview
}


//====================================================================================================
// timer : triggered every 16ms ( about 60 frames per second )
//====================================================================================================

void timer(int value) {
	update();

	// render
	glutPostRedisplay();

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	glutTimerFunc(16, timer, 0);
}

//====================================================================================================
// main
//====================================================================================================

int main(int argc, char** argv) {
	// create opengL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(G_SCREEN_WIDTH, G_SCREEN_HEIGHT);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(argv[0]);

	// init
	init();

	// set callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);

	glutTimerFunc(16, timer, 0);

	// main loop
	glutMainLoop();

	return 0;
}
