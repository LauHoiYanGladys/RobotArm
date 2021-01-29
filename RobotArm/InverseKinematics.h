#pragma once
#include <Eigen/Dense>
#include <vector>
#include "DHframe.h"
#include "Arm.h"
#include "Link.h"

class InverseKinematics {
private:
	/*std::vector<Node*>fittingNodes;*/
	static const double PI;
	double xTarget;
	double yTarget;
	double zTarget;
	double xTargetNew;
	double yTargetNew;
	double zTargetNew;
	Vector3d goal;
	Vector3d goalNew;

	/*double testJointVariables;
	double new_testJointVariables;*/
	Vector3d testJointVariables;
	Vector3d new_testJointVariables;
	Matrix3d jacobian;
	/*Vector3d jacobian;
	double costGradient;*/
	/*Vector3d costGradient;*/
	std::vector<double> costGradient;
	Arm* theArm;
	/*enum jointNumber { first, second, third };*/
	enum class costType { currCost, newCost };
	double alpha; // learning rate for revolute joint
	double alphaPris; // learning rate for prismatic joint
	//double alpha0, alpha1, alpha2; // learning rate
	int maxIteration;
	double costChange;
	double stopThreshold;
	double delta;// for differentiation
	//double delta0, delta1, delta2; // for differentiation
	double theCurrCost;
	double theNewCost;

public:
	//// default constructor
	//InverseKinematics() {};

	// constructor
	InverseKinematics(double x, double y, double z, Arm* inputArm);
	
	// compute cost gradient by (FK to end effector - goal position).norm();
	void computeCostGradient();
	/*double computeCostGradient();*/

	// wrong method of finding partial derivative of cost function (kept here as a reminder)
	double differentiate(int jointVariableIndex);
	
	// update all joint variable parameters in gradient descent
	void update();

	// performs gradient descent to find IK
	void getIK();
	
	// gets analytical IK
	void getIKAnalytical();

	// evaluates the cost function at either current or updated variable values
	double computeCost(costType theCostType);
	
	// evaluates the cost function at the provided joint variables
	double computeCost(double jointVariable1, double jointVariable2, double jointVariable3);

	// evaluates the cost function at the joint variables provided in a vector
	double computeCost(std::vector<double> jointVariables);

	// take partial derivative of the cost function of a certain joint
	double differentiateCost(int jointVariableNum);

	// take partial derivative of the cost function of all joints
	std::vector<double> differentiateCost();

	// compute angle error
	double computeAngleDeviation();

	// computes Jacobian with the current joint variables
	void computeJacobian();

	// numerically computes a column of the Jacobian that corresponds to a certain joint
	// joint index is that joint's position in theJoints of theArm and runs from 0 to 2
	// superceded by computeJacobianColRev and computeJacobianColPris but kept here in case needed later
	Vector3d computeJacobianCol(int jointVariableIndex);

	// analytically compute jacobian column for revolute joints
	Vector3d computeJacobianColRev(int jointVariableIndex);

	// analytically compute jacobian column for prismatic joints
	Vector3d computeJacobianColPris(int jointVariableIndex);

	// sets the result to the function parameters that are called by reference for drawing
	void getResult(Vector3d& result);
	
	// get the correct prismatic joint variable after arm is correctly oriented (i.e. first two joint angles optimized)
	double getPrismaticJointVar();

	// gets distance between current end-effector position and goal position
	double getDistance();

	// turns vector3d into regular vector
	std::vector<double> vector3dToRegularVector(Vector3d theVector);

	// keep prismatic joint variable >= 0
	void constrainPrismatic();
};
