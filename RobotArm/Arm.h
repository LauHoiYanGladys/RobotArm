#pragma once
#include <vector>
#include <unordered_map>
#include "Link.h"
#include "Joint.h"
#include "DHframe.h"

class Arm
{
public:
	static const double PI;

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
	//draws the Arm in its current configuration
	void draw();

	// builds the Arm from DH parameters
	void buildArm();

	// changes joint variables (theFrames and theJoints)
	void moveArm(double newJointVariable1, double newJointVariable2, double newJointVariable3);

	// changes test joint variable of a frame identified by frameNumber (1, 2 or 4)
	void updateTestFrame(int frameNumber, double newJointVariable);

	// changes test joint variable of a frame identified by frame pointer
	void updateTestFrame(DHframe* theFrame, double newJointVariable);
	
	// changes test joint variables of all the frames
	void updateTestFrames(double newJointVariable1, double newJointVariable2, double newJointVariable3);

	// get test joint variable of a frame identified by frame pointer
	double getTestJointVariable(DHframe* theFrame);

	// returns all test joint variable as a Vector3d
	Vector3d/*double*/ getTestJointVariable();

	// returns all joint variable as a Vector3d
	Vector3d getJointVariable();

	// computes and returns the forward kinematics to a certain frame identified by frame number
	Vector3d compute_test_FK(int frameIndex); // frameNumber runs from 0 to theFrames.size()-1

	// computes and returns the forward kinematics to a certain frame, identified by frame pointer
	Vector3d compute_test_FK(DHframe* theFrame); 

	// gets DH parameters from user
	void getDHParameters();

};

