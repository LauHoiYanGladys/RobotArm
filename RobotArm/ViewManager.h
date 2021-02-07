#pragma once

#include <chrono>
#include "Arm.h"
#include "Camera3D.h"
#include "OrbitingViewer.h"
#include "GraphicFont.h"
#include "InverseKinematics.h"

class ViewManager
{
public:
	static const double PI;
	Arm theArm;
	//InverseKinematics theIK;
	Camera3D theCamera;
	OrbitingViewer theOrbiter;
	CourierNewFont textfont;
	int win_width = 1024;			//window width for fssimplewindow
	int win_height = 768;			//window height for fssimplewindow

	int view_dist = 2000;			//viewing distance for 3D environment

	int mapsize = 250;				//size of environment map

	DrawingUtilNG::vertexF start = { 60,20,50 };	//start position for end effector
	bool startMoved = false;						//boolean for whether the start position has changed

	DrawingUtilNG::vertexF goal = { 50,10,30 };		//goal position for end effector
	bool goalMoved = false;							//boolean for whether the goal position has changed

	//keep track of whether user is adjusting start or goal position
	enum moveToggleEnum { moveStart, moveGoal };
	moveToggleEnum moveToggle = moveGoal;

	//timestamp of the last time the arm calculated its position
	std::chrono::system_clock::time_point prevArmMoveTime;// = std::chrono::system_clock::now();
	double moveTimeThresh = 20;   //how much time (in ms) the arm will wait until next move calculation

	int lastKey = -1;	//value of the last key press

	//constructor calls the initialize function
	ViewManager()/*: theIK(0, 0, 0, &theArm)*/  {
		
		initialize();
		/*theIK = InverseKinematics(0, 0, 0, &theArm);*/
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

	//draws start position for end effector
	void draw_start();

	//draws goal position for end effector
	void draw_goal();

	//draws 2D HUD overlay (text, etc)
	void draw_overlay2D();

	// move arm according to IK results
	void controlArm();

	// check whether goal is currently moving
	//bool goalIsMoving();
};

