#include "ViewManager.h"
#include <iostream>
#include <sstream>
#include <iomanip>

void ViewManager::initialize()
{
	FsOpenWindow(16, 16, win_width, win_height, 1);

	//kind of janky but it works...need to initialize font after the FsOpenWindow command
	CourierNewFont textfont_temp;
	textfont = textfont_temp;

	theOrbiter.dist = 300;
	theOrbiter.p = -0.135;
	theOrbiter.h = 0.6;

	theCamera.farZ = view_dist + theOrbiter.dist;
	theArm.buildArm();
}

void ViewManager::manage()
{
	FsGetWindowSize(win_width, win_height);

	//read keyboard/mouse inputs
	user_controls_read();

	//start drawing
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, win_width, win_height);

	//do the 3D drawing
	draw_environment3D();
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

	if (FsGetKeyState(FSKEY_RIGHT))
		theOrbiter.h += 0.005;
	if (FsGetKeyState(FSKEY_LEFT))
		theOrbiter.h -= 0.005;
	if (FsGetKeyState(FSKEY_DOWN))
		theOrbiter.p += 0.005;
	if (FsGetKeyState(FSKEY_UP))
		theOrbiter.p -= 0.005;
	if (FsGetKeyState(FSKEY_F) && theOrbiter.dist > 0.5)
		theOrbiter.dist /= 1.02;
	if (FsGetKeyState(FSKEY_B) && theOrbiter.dist < theCamera.farZ * .8)
		theOrbiter.dist *= 1.02;

	//update camera views based on keyboard inputs above
	theOrbiter.setUpCamera(theCamera);
	theCamera.farZ = view_dist + theOrbiter.dist;
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
	int mapsize = 250;
	int floor_offset = -1;
	glBegin(GL_QUADS);
	glColor3ub(180, 255, 180);	//light green
	glVertex3i(mapsize, floor_offset, -mapsize);
	glVertex3i(-mapsize, floor_offset, -mapsize);
	glVertex3i(-mapsize, floor_offset, mapsize);
	glVertex3i(mapsize, floor_offset, mapsize);
	glEnd();

	//draw grid on the floor
	glColor3ub(0, 230, 0);
	glLineWidth(1);
	glBegin(GL_LINES);

	int grid_size = 25;
	int mapcenter_x = 0;
	int mapcenter_z = 0;
	int grid_offset = 0;

	for (int x = mapcenter_x - mapsize; x <= mapcenter_x + mapsize; x += grid_size)
		for (int z = mapcenter_z - mapsize; z <= mapcenter_z + mapsize; z += grid_size)
		{
			{
				glVertex3i(x, grid_offset, x);
				glVertex3i(x, grid_offset, z);
				glVertex3i(x, grid_offset, z);
				glVertex3i(z, grid_offset, z);
			}
		}
	glEnd();

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

}
