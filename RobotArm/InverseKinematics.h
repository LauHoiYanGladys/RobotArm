#pragma once
#include <Eigen/Dense>
#include <vector>
#include "DHframe.h"
#include "Arm.h"
#include "Link.h"

class InverseKinematics {
private:
	static const double PI;
	double xTarget;
	double yTarget;
	double zTarget;
	double xTargetNew;
	double yTargetNew;
	double zTargetNew;
	Vector3d goal;
	Vector3d goalNew;

	Vector3d testJointVariables;
	Vector3d new_testJointVariables;
	Matrix3d jacobian;
	std::vector<double> costGradient;
	Arm* theArm;
	enum class costType { currCost, newCost };
	double alpha; // learning rate for revolute joint
	double alphaPris; // learning rate for prismatic joint
	double costChangeStopThreshold;
	int maxIteration;
	double costChange;
	double stopThreshold;
	double delta;// for differentiation
	double theCurrCost;
	double theNewCost;
	double workspaceThreshold; // the distance between end-effector and goal after IK above which the goal is deemed outside workspace
public:
	// constructor
	InverseKinematics(double x, double y, double z, Arm* inputArm);
	
	// destructor
	~InverseKinematics() {};

	// compute cost gradient by (FK to end effector - goal position).norm();
	void computeCostGradient();
	
	// update all joint variable parameters in gradient descent
	void update();

	// performs gradient descent to find IK
	bool getIK();
	
	// checks whether a certain goal position is inside workspace given the cost from the IK
	bool isInsideWorkspace(double theCost);

	// gets analytical IK
	void getIKAnalytical();

	// evaluates the cost function at either current or updated variable values
	double computeCost(costType theCostType);

	// evaluates the cost function at the joint variables provided in a vector
	double computeCost(std::vector<double> jointVariables);

	// take partial derivative of the cost function of all joints
	std::vector<double> differentiateCost();

	// compute angle error
	double computeAngleDeviation();

	// sets the result to the function parameters that are called by reference for drawing
	void getResult(Vector3d& result);

	// gets distance between current end-effector position and goal position
	double getDistance();

	// turns vector3d into regular vector
	static std::vector<double> vector3dToRegularVector(Vector3d theVector);

	// keep prismatic joint variable >= 0
	void constrainPrismatic();
};
