#include "ViewManager.h"
#include <iostream>
#include <sstream>
#include <iomanip>

const double ViewManager::PI = 3.1415927;

void ViewManager::initialize()
{
	FsOpenWindow(16, 16, win_width, win_height, 1);

	//kind of janky but it works...need to initialize font after the FsOpenWindow command
	CourierNewFont textfont_temp;
	textfont = textfont_temp;

	theOrbiter.dist = 300;
	theOrbiter.p = -0.350;
	theOrbiter.h = 0.;

	theCamera.farZ = view_dist + theOrbiter.dist;

	/*theArm.buildArm_SCARA();*/
	/*theArm.buildArm_PUMA560();*/
	theArm.buildArm();
	controlArm();
}

void ViewManager::manage()
{
	FsGetWindowSize(win_width, win_height);

	//read keyboard/mouse inputs
	user_controls_read();

	//move arm if enough time has passed between calcs
	auto currentTime = std::chrono::system_clock::now();
	double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds> (currentTime - prevArmMoveTime).count();
	//cout << "elapsed time: " << elapsedTime << '\n';
	if (elapsedTime > moveTimeThresh && goalMoved) {

		controlArm();
		prevArmMoveTime = currentTime;
	}

	//start drawing
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, win_width, win_height);

	//do the 3D drawing
	draw_environment3D();
	draw_start();
	draw_goal();

	/*std::vector<double>armPosition{ PI/4, PI / 4, 10 };
	theArm.moveArm(armPosition);*/
	/*controlArm();*/
	theArm.draw();

	//do the 2D overlay drawing
	draw_overlay2D();

	FsSwapBuffers();
	FsSleep(10);
}

void ViewManager::user_controls_read()
{
	/*std::cout << "lastKey = " << lastKey << '\n';*/
	FsPollDevice();
	int key = FsInkey();
	/*std::cout << "key = " << key << '\n';*/

	//move camera around
	if (FsGetKeyState(FSKEY_RIGHT))
		theOrbiter.h += 0.005;
	if (FsGetKeyState(FSKEY_LEFT))
		theOrbiter.h -= 0.005;
	if (FsGetKeyState(FSKEY_DOWN))
		theOrbiter.p += 0.005;
	if (FsGetKeyState(FSKEY_UP))
		theOrbiter.p -= 0.005;

	//move camera in & out
	if (FsGetKeyState(FSKEY_F) && theOrbiter.dist > 0.5)
		theOrbiter.dist /= 1.02;
	if (FsGetKeyState(FSKEY_B) && theOrbiter.dist < theCamera.farZ * .8)
		theOrbiter.dist *= 1.02;

	//update camera views based on keyboard inputs above
	theOrbiter.setUpCamera(theCamera);
	theCamera.farZ = view_dist + theOrbiter.dist;

	//toggle moving start or goal position
	//TOGGLING DOESNT WORK, HAVE TO USE TWO SEPARATE KEYS
	//if (FsGetKeyState(FSKEY_T)) {
	//	switch (moveToggle) {
	//	case moveGoal:
	//		moveToggle = moveStart;
	//		break;
	//	case moveStart:
	//		moveToggle = moveGoal;
	//		break;
	//	}
	//}

	//change between moving start or goal position
	if (FsGetKeyState(FSKEY_T)) {
		moveToggle = moveStart;
		controlArm();
	}
	if (FsGetKeyState(FSKEY_G)) {
		moveToggle = moveGoal;
		controlArm();
	}
		
	//move the start/goal position
	goalMoved = startMoved = false;
	double xpos, ypos, zpos = 0.; //IN PROGRESS
	if (FsGetKeyState(FSKEY_D) && goal.x < mapsize) {
		goal.x += 0.5;
		goalMoved = true;
	}
	if (FsGetKeyState(FSKEY_A) && goal.x > -mapsize) {
		goal.x -= 0.5;
		goalMoved = true;
	}
	if (FsGetKeyState(FSKEY_W) && goal.y < mapsize) {
		goal.y += 0.5;
		goalMoved = true;
	}
	if (FsGetKeyState(FSKEY_S) && goal.y > -mapsize) {
		goal.y -= 0.5;
		goalMoved = true;
	}
	if (FsGetKeyState(FSKEY_E)) {
		goal.z += 0.5;
		goalMoved = true;
	}
	if (FsGetKeyState(FSKEY_C)) {
		goal.z -= 0.5;
		goalMoved = true;
	}

	// compute IK on press of space bar
	if (key == FSKEY_SPACE)
		controlArm();

	// compute test end point FKs on press of Z
	if (key == FSKEY_Z)
		theArm.testing_compute_test_FK_all(); // you need to put a stop on the cout line of this function and check the computed value by hovering mouse over that value
		//theArm.compute_test_FK_all(); // prints out values in the console


	//update last key pressed
	lastKey = key;
}

void ViewManager::draw_environment3D()
{
	//set up for 2D drawing (for background color)
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, (float)win_width - 1, (float)win_height - 1, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_DEPTH_TEST);

	//single color background square
	glColor3ub(180, 240, 255);	//light blue
	glBegin(GL_QUADS);
	glVertex2i(0, 0);
	glVertex2i(0, win_height);
	glVertex2i(win_width, win_height);
	glVertex2i(win_width, 0);
	glEnd();

	//set up for 3D drawing
	theCamera.setUpCameraProjection();
	theCamera.setUpCameraTransformation();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1, 1);

	//quad for the ground color
	double floor_offset = -0.5;
	glBegin(GL_QUADS);
	glColor3ub(180, 255, 180);	//light green
	glVertex3f(mapsize, floor_offset, -mapsize);
	glVertex3f(-mapsize, floor_offset, -mapsize);
	glVertex3f(-mapsize, floor_offset, mapsize);
	glVertex3f(mapsize, floor_offset, mapsize);
	glEnd();

	//draw grid on the floor
	glColor3ub(0, 230, 0);
	glLineWidth(1);
	glBegin(GL_LINES);

	double grid_size = 25;
	double mapcenter_x = 0;
	double mapcenter_z = 0;
	double grid_offset = 0;

	for (double x = mapcenter_x - mapsize; x <= mapcenter_x + mapsize; x += grid_size)
		for (double z = mapcenter_z - mapsize; z <= mapcenter_z + mapsize; z += grid_size)
		{
			{
				glVertex3f(x, grid_offset, x);
				glVertex3f(x, grid_offset, z);
				glVertex3f(x, grid_offset, z);
				glVertex3f(z, grid_offset, z);
			}
		}
	glEnd();

}

void ViewManager::draw_start()
{
	double start_size = 3; //size of start position indicator

	//set up for 3D drawing
	theCamera.setUpCameraProjection();
	theCamera.setUpCameraTransformation();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1, 1);

	//quad for the shadow on ground
	double shadow_offset = 0;
	glBegin(GL_QUADS);
	glColor3ub(100, 200, 100);	//dark green
	glVertex3f(start_size * 1.05 / 2 + start.x, shadow_offset, -start_size * 1.05 / 2 - start.y);
	glVertex3f(-start_size * 1.05 / 2 + start.x, shadow_offset, -start_size * 1.05 / 2 - start.y);
	glVertex3f(-start_size * 1.05 / 2 + start.x, shadow_offset, start_size * 1.05 / 2 - start.y);
	glVertex3f(start_size * 1.05 / 2 + start.x, shadow_offset, start_size * 1.05 / 2 - start.y);
	glEnd();

	//draw cube for showing start position in 3D
	glPushMatrix();
	glTranslatef(start.x, start.z, -start.y);	//order of x,y,z is reoriented with negative in front of y to account for OpenGL conventions

	glColor3ub(0, 255, 0);	//bright green
	DrawingUtilNG::drawCube(start_size);

	glPopMatrix();
}

void ViewManager::draw_goal()
{
	double goal_size = 3; //size of goal position indicator
	
	//set up for 3D drawing
	theCamera.setUpCameraProjection();
	theCamera.setUpCameraTransformation();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1, 1);

	//quad for the shadow on ground
	double shadow_offset = 0;
	glBegin(GL_QUADS);
	glColor3ub(100, 200, 100);	//dark green
	glVertex3f(goal_size * 1.05 / 2 + goal.x, shadow_offset, -goal_size * 1.05 / 2 - goal.y);
	glVertex3f(-goal_size * 1.05 / 2 + goal.x, shadow_offset, -goal_size * 1.05 / 2 - goal.y);
	glVertex3f(-goal_size * 1.05 / 2 + goal.x, shadow_offset, goal_size * 1.05 / 2 - goal.y);
	glVertex3f(goal_size * 1.05 / 2 + goal.x, shadow_offset, goal_size * 1.05 / 2 - goal.y);
	glEnd();

	//draw cube for showing goal position in 3D
	glPushMatrix();
	glTranslatef(goal.x, goal.z, -goal.y);	//order of x,y,z is reoriented with negative in front of y to account for OpenGL conventions

	glColor3ub(255, 0, 0);	//bright red
	DrawingUtilNG::drawCube(goal_size);

	glPopMatrix();
}

void ViewManager::draw_overlay2D()
{
	using namespace std;

	//set up for 2D drawing
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, (float)win_width - 1, (float)win_height - 1, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_DEPTH_TEST);

	//set up for writing text
	textfont.setColorRGB(0, 0, 0);
	stringstream datastring;
	string data;

	//controls - top left of screen
	textfont.drawText("controls:", 10, 30, .31);
	textfont.drawText("W, A, S, D, E, C - move start/goal position", 10, 50, .25);
	textfont.drawText("            T, G - toggle star(T)/(G)oal movement", 10, 65, .25);
	textfont.drawText("arrow keys, F, B - move camera", 10, 80, .25);
	textfont.drawText("           space - update arm position", 10, 95, .25);

	textfont.drawText("currently moving: ", 10, 130, .31);
	if (moveToggle == moveGoal) {
		textfont.setColorRGB(0.8, 0, 0);
		textfont.drawText("GOAL", 205, 130, .31);
	}
	if (moveToggle == moveStart) {
		textfont.setColorRGB(0, 0.7, 0);
		textfont.drawText("START", 205, 130, .31);
	}
	textfont.setColorRGB(0, 0, 0);

	//camera metrics - lower left of screen
	textfont.drawText("camera stats", 10, win_height - 145, .31);

	datastring << fixed << setprecision(1);
	datastring << "x:" << theCamera.x;
	data = datastring.str();
	textfont.drawText(data, 10, win_height - 120, .32);
	datastring.str("");

	datastring << "y:" << -theCamera.y; // negative sign to give the correct value in the coordinate system used by the robot arm
	data = datastring.str();
	textfont.drawText(data, 10, win_height - 100, .32);
	datastring.str("");

	datastring << "z:" << theCamera.z;
	data = datastring.str();
	textfont.drawText(data, 10, win_height - 80, .32);
	datastring.str("");

	datastring << fixed << setprecision(3);
	datastring << "h:" << theOrbiter.h;
	data = datastring.str();
	textfont.drawText(data, 10, win_height - 60, .32);
	datastring.str("");

	datastring << "p:" << theOrbiter.p;
	data = datastring.str();
	textfont.drawText(data, 10, win_height - 40, .32);
	datastring.str("");

	datastring << "d:" << theOrbiter.dist;
	data = datastring.str();
	textfont.drawText(data, 10, win_height - 20, .32);
	datastring.str("");

	//start & goal position - lower right of screen
	textfont.setColorRGB(0, 0.7, 0);
	textfont.drawText("start pos", win_width - 130, win_height - 175, .31);
	textfont.setColorRGB(0, 0, 0);

	datastring << fixed << setprecision(2);
	datastring << "x:" << start.x;
	data = datastring.str();
	textfont.drawText(data, win_width - 130, win_height - 150, .32);
	datastring.str("");

	datastring << "y:" << start.y;
	data = datastring.str();
	textfont.drawText(data, win_width - 130, win_height - 130, .32);
	datastring.str("");

	datastring << "z:" << start.z;
	data = datastring.str();
	textfont.drawText(data, win_width - 130, win_height - 110, .32);
	datastring.str("");

	textfont.setColorRGB(0.8, 0, 0);
	textfont.drawText("goal pos", win_width - 130, win_height - 85, .31);
	textfont.setColorRGB(0, 0, 0);

	datastring << fixed << setprecision(2);
	datastring << "x:" << goal.x;
	data = datastring.str();
	textfont.drawText(data, win_width - 130, win_height - 60, .32);
	datastring.str("");

	datastring << "y:" << goal.y;
	data = datastring.str();
	textfont.drawText(data, win_width - 130, win_height - 40, .32);
	datastring.str("");

	datastring << "z:" << goal.z;
	data = datastring.str();
	textfont.drawText(data, win_width - 130, win_height - 20, .32);
	datastring.str("");

}

bool ViewManager::controlArm()
{
	Vector3d newJointVariables;
	bool isInsideWorkspace;
	// compute IK from current joint variables
	double xpos, ypos, zpos;
	if (moveToggle == moveStart) {
		xpos = start.x;
		ypos = start.y;
		zpos = start.z;
	}
	if (moveToggle == moveGoal) {
		xpos = goal.x;
		ypos = goal.y;
		zpos = goal.z;
	}

	InverseKinematics theIK(xpos, ypos, zpos, &theArm);
	/*theIK.getIKAnalytical();*/
	isInsideWorkspace = theIK.getIK();
	theIK.getResult(newJointVariables);

	/*std::cout << "newJointVariables are " << newJointVariables << std::endl;*/

	// draw arm with updated joint variables
	std::vector<double>temp = InverseKinematics::vector3dToRegularVector(newJointVariables);
	theArm.moveArm(temp);
	if (isInsideWorkspace)
		std::cout << "Goal position is inside workspace" << std::endl;
	else
		std::cout << "Goal position is NOT inside workspace" << std::endl;
	return isInsideWorkspace;
}

//bool ViewManager::goalIsMoving()
//{
//	if (FsGetKeyState(FSKEY_D) /*&& goal.x < mapsize */||
//		FsGetKeyState(FSKEY_A) /*&& goal.x > -mapsize */||
//		FsGetKeyState(FSKEY_S) /*&& goal.y < mapsize */||
//		FsGetKeyState(FSKEY_W) ||
//		FsGetKeyState(FSKEY_E) ||
//		FsGetKeyState(FSKEY_C))
//		return true;
//	else
//		return false;
//}
