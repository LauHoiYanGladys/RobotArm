#pragma once
#include <vector>
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

	//draws the Arm in its current configuration
	void draw();

	// builds the Arm from DH parameters
	void buildArm();

	// changes joint variables
	void moveArm(double newJointVariable1, double newJointVariable2, double newJointVariable3);

	// gets DH parameters from user
	void getDHParameters();

};

