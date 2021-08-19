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
	std::vector<Arm*> theArm; // a vector to contain the single arm present; has to be in vector so that it can be deleted upon switching arm type
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

	DrawingUtilNG::vertexF arm_target = start;		//position that arm is targeting (start, goal, or maybe something in between?)
	bool targetMoved = false;						//boolean for whether the arm's target position has changed

	bool isArmReached = false;			//boolean for whether arm is able to reach desired end effector position
	bool isArmIntersecting = false;		//boolean for whether arm is intersectino obstacles

	bool isTracing = false; // boolean for whether arm is currently tracing from start to goal

	bool tracingPrelimsAreSet = false; // true if preliminaries are computed before tracing starts

	//keep track of whether user is adjusting start or goal position
	enum moveToggleEnum { moveStart, moveGoal };
	moveToggleEnum moveToggle = moveStart;

	// Type of arm to build
	enum armType {stanford, puma, scara};
	armType theArmType = scara;

	// vector containers for waypoints
	std::vector<DrawingUtilNG::vertexF> waypoints;
	
	// waypoint interval
	int waypointInterval = 2;

	// true if current arm control has been completed
	bool armControlCompleted = false;

	//timestamp of the last time the arm calculated its position
	std::chrono::system_clock::time_point prevArmMoveTime;// = std::chrono::system_clock::now();
	double moveTimeThresh = 20;   //how much time (in ms) the arm will wait until next move calculation

	//constructor calls the initialize function
	ViewManager() {initialize();}

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

	// move arm according to IK results, also outputs whether goal position is within the workspace (the threshold is set in Arm class upon building of arm)
	bool controlArm();

	// reposition end effector at current move target (start/goal)
	void repositionEndEffector();

	//checks whether arm is able to reach the end effector goal position (without moving arm)
	bool canArmReach(double xpos, double ypos, double zpos);

	// toggles between different arm types
	void toggleArmType();

	// deletes the old arm and builds a new arm
	void buildNewArm(armType theArmType);
};

