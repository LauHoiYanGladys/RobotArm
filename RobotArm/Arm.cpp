#include "Arm.h"
#include "DHframe.h"
#include "Joint.h"
#include "fssimplewindow.h"

const double Arm::PI = 3.1415927;

class Joint;

void Arm::draw()
{
	/*for (auto& currJoint : theJoints) {
		currJoint->draw();
	}*/
	theJoints[0]->draw();
	theJoints[1]->draw();
	theJoints[2]->draw();
}

void Arm::buildArm()
{
	// define the 0th DH frame
	DHframe* zerothFrame = new DHframe();

	// add DH frame to frame collection
	theFrames.push_back(zerothFrame);
	zerothFrame->assignParentDHframe(nullptr);

	// create the first (revolute) joint, corresponds to 0th DH frame
	Joint* firstJoint = new Joint(zerothFrame, 25);

	// add the joint to the joint collection
	theJoints.push_back(firstJoint);
	
	// define the first DH frame 
	DHframe* firstFrame = new DHframe(0., PI/2, 25., 0.);
	firstFrame->assignParentDHframe(zerothFrame);
	/*firstFrame->assignMovingJoint(firstJoint);*/

	// add DH frame to frame collection
	theFrames.push_back(firstFrame);

	// create the second (revolute) joint, corresponds to the 1st DH frame
	Joint* secondJoint = new Joint(firstFrame, 25.);
	/*secondJoint.assignParentJoint(&(theJoints.back()));*/
	/*secondJoint->assignParentJoint(firstJoint);*/
	secondJoint->assignLinkDirection(Link::alongX);

	// add the joint to the joint collection
	theJoints.push_back(secondJoint);

	// define the second DH frame
	DHframe* secondFrame = new DHframe(0., PI / 2, 0., PI / 2);
	secondFrame->assignParentDHframe(firstFrame);
	/*secondFrame->assignMovingJoint(secondJoint);*/

	// add DH frame to frame collection
	theFrames.push_back(secondFrame);

	// define the third DH frame, note that it has no associated moving joint 
	DHframe* thirdFrame = new DHframe(0., 0., 25., 0);
	thirdFrame->assignParentDHframe(secondFrame);

	// add DH frame to frame collection
	theFrames.push_back(thirdFrame);

	// create the third (prismatic) joint
	// note that it corresponds to the third DH frame instead of second DH frame
	Joint* thirdJoint = new Joint(thirdFrame, 25.);
	thirdJoint->setJointTypePrismatic();
	thirdJoint->updateJointVariable(10.);

	// add the joint to the joint collection
	theJoints.push_back(thirdJoint);

	// define the fourth DH frame 
	DHframe* fourthFrame = new DHframe(0., 0., 25., -PI/2);
	fourthFrame->assignParentDHframe(thirdFrame);
	/*fourthFrame->assignMovingJoint(thirdJoint);*/

	// add DH frame to frame collection
	theFrames.push_back(fourthFrame);
	
}

void Arm::moveArm(double newJointVariable1, double newJointVariable2, double newJointVariable3)
{
	// updates "theta" or "d" of relevant DH frames
	theFrames[1]->update_theta(newJointVariable1);
	theFrames[2]->update_theta(newJointVariable2);
	theFrames[4]->update_d(newJointVariable3);

	// updates the joint variables of relevant joints
	theJoints[0]->updateJointVariable(newJointVariable1);
	theJoints[1]->updateJointVariable(newJointVariable2);
	theJoints[2]->updateJointVariable(newJointVariable3);

}
