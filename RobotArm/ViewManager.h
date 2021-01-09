#pragma once
#include "Arm.h"
#include "Camera3D.h"
#include "OrbitingViewer.h"
#include "GraphicFont.h"

class ViewManager
{
public:
	static const double PI;
	Arm theArm;
	Camera3D theCamera;
	OrbitingViewer theOrbiter;
	CourierNewFont textfont;

	int win_width = 1024;			//window width for fssimplewindow
	int win_height = 768;			//window height for fssimplewindow

	int view_dist = 2000;			//viewing distance for 3D environment

	int mapsize = 250;				//size of environment map

	DrawingUtilNG::vertexF goal = { 0,0,0 };	//goal position for end effector

	//constructor calls the initialize function
	ViewManager() {
		initialize();
	}

	//initial setup
	void initialize();

	//called on every loop of the main function
	//updates/draws everything based on user inputs and calculations
	void manage();

	//reads keyboard/mouse commands and updates appropriate properties
	void user_controls_read();

	//draws 3D environment that robot arm is in
	void draw_environment3D();

	//draws goal position for end effector
	void draw_goal();

	//draws 2D HUD overlay (text, etc)
	void draw_overlay2D();
};

