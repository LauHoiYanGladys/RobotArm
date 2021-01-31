#pragma once
#include <vector>
#include <unordered_map>
#include "DrawingUtilNG.h"
#include "Link.h"
#include "Joint.h"
#include "DHframe.h"

class Arm
{
public:
	static const double PI;
	std::vector<int>frameWithJointVariable; // The indices of frames with joint variables
	std::vector<int>frameWithEndPoints; // The indices of frames with end points (i.e. joint center or end-effector position)
	Arm() {};
	~Arm() {
		if (!theJoints.empty()) {
			for (auto& currJoint : theJoints)
				delete currJoint;
		}
		if (!theFrames.empty()) {
			for (auto& currFrame : theFrames)
				delete currFrame;
		}
		/*if (!theLinks.empty()) {
			for (auto& currLink : theLinks)
				delete currLink;
		}*/
	}

	std::vector<Link> theLinks; //vector of links that forms the arm. 0th link is the base
	std::vector<Joint*> theJoints; // vector of joints in the arm. 0th joint is the base joint.
	std::vector<DHframe*> theFrames; // vector of DH frames in the arm
	std::unordered_map<Joint*, DHframe*> jointFrameMap; // maps the joint to the corresponding DH frame
	double alpha; // learning rate for revolute joint
	double alphaPris; // learning rate for prismatic joint
	double costChangeStopThreshold; // stop IK when cost change is less than this threshold

	//draws the Arm in its current configuration
	void draw();

	// builds the Arm from DH parameters
	void buildArm();

	// builds the PUMA560 arm 
	void buildArm_PUMA560();

	// changes joint variables given in a vector (theFrames and theJoints), also updates test frames to ensure next test frame computation is based on actual joint variables 
	void moveArm(std::vector<double> jointVariables);

	// changes test joint variables given in a vector of all the frames 
	void updateTestFrames(std::vector<double> jointVariables);

	// returns all test joint variable as a Vector3d
	Vector3d getTestJointVariable();

	// computes and returns the forward kinematics to a certain frame identified by frame number
	Vector3d compute_test_FK(int frameIndex); // frameNumber runs from 0 to theFrames.size()-1

	// computes and returns the forward kinematics to a certain frame, identified by frame pointer
	Vector3d compute_test_FK(DHframe* theFrame); 

	// computes and returns the forward kinematics to each joint center and the end effector in a vector of DrawingUtilNG::vertexF
	std::vector<DrawingUtilNG::vertexF> compute_test_FK_all();

	// for debugging and testing compute_test_FK_all
	void testing_compute_test_FK_all();

	// gets DH parameters from user
	void getDHParameters();


};

