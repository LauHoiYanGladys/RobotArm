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
	theArm.buildArm();
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
	if (elapsedTime > moveTimeThresh && goalIsMoving()) {

		controlArm();
		prevArmMoveTime = currentTime;
	}

	//start drawing
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, win_width, win_height);

	//do the 3D drawing
	draw_environment3D();
	draw_goal();

	/*theArm.moveArm(PI/4, PI/4, 20.);*/
	/*controlArm();*/
	/*theArm.moveArm(PI / 4);*/
	theArm.draw();

	//do the 2D overlay drawing
	draw_overlay2D();

	FsSwapBuffers();
	FsSleep(5);
}

void ViewManager::user_controls_read()
{
	FsPollDevice();
	int key = FsInkey();

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
		
	if (FsGetKeyState(FSKEY_D) && goal.x < mapsize)
		goal.x += 0.5;
	if (FsGetKeyState(FSKEY_A) && goal.x > -mapsize)
		goal.x -= 0.5;
	if (FsGetKeyState(FSKEY_S) && goal.y < mapsize)
		goal.y += 0.5;
	if (FsGetKeyState(FSKEY_W) && goal.y > -mapsize)
		goal.y -= 0.5;

	if (FsGetKeyState(FSKEY_E))
		goal.z += 0.5;
	if (FsGetKeyState(FSKEY_C))
		goal.z -= 0.5;

	// compute IK on press of space bar
	if (key == FSKEY_SPACE)
		controlArm();

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
	glVertex3f(goal_size * 1.05 / 2 + goal.x, shadow_offset, -goal_size * 1.05 / 2 + goal.y);
	glVertex3f(-goal_size * 1.05 / 2 + goal.x, shadow_offset, -goal_size * 1.05 / 2 + goal.y);
	glVertex3f(-goal_size * 1.05 / 2 + goal.x, shadow_offset, goal_size * 1.05 / 2 + goal.y);
	glVertex3f(goal_size * 1.05 / 2 + goal.x, shadow_offset, goal_size * 1.05 / 2 + goal.y);
	glEnd();

	//draw cube for showing goal position in 3D
	glPushMatrix();
	glTranslatef(goal.x, goal.z, goal.y);	//order of x,y,z is reoriented to account for OpenGL conventions

	glColor3ub(0, 255, 0);	//bright green
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

	//metrics
	textfont.setColorRGB(0, 0, 0);
	stringstream datastring;
	string data;

	//camera metrics - lower left
	textfont.drawText("camera stats", 10, win_height - 145, .31);

	datastring << fixed << setprecision(1);
	datastring << "x:" << theCamera.x;
	data = datastring.str();
	textfont.drawText(data, 10, win_height - 120, .32);
	datastring.str("");

	datastring << "y:" << theCamera.y;
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

	//goal position - lower right
	textfont.drawText("goal pos", win_width - 130, win_height - 85, .31);

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

void ViewManager::controlArm()
{
	Vector3d newJointVariables;
	// compute IK from current joint variables
	InverseKinematics theIK(goal.x, goal.y, goal.z, &theArm);
	/*theIK.getIKAnalytical();*/
	theIK.getIK();
	theIK.getResult(newJointVariables);

	/*std::cout << "newJointVariables are " << newJointVariables << std::endl;*/

	// draw arm with updated joint variables
	// the negative sign needed before the first joint to prevent reflection about the horizontal axis, probably due to non-right-hand coordinate system of of openGL
	theArm.moveArm(-newJointVariables(0), newJointVariables(1), newJointVariables(2));

	// The test frames, though, need to store the correct joint variables for the next IK to start from the correct joint configuration
	// thus no negative sign in front of the first joint variable
	theArm.updateTestFrames(newJointVariables(0), newJointVariables(1), newJointVariables(2)); // to be extra safe that it's updated
	/*theArm.draw();*/
}

bool ViewManager::goalIsMoving()
{
	if (FsGetKeyState(FSKEY_D) && goal.x < mapsize ||
		FsGetKeyState(FSKEY_A) && goal.x > -mapsize ||
		FsGetKeyState(FSKEY_S) && goal.y < mapsize ||
		FsGetKeyState(FSKEY_E) ||
		FsGetKeyState(FSKEY_C))
		return true;
	else
		return false;
}
