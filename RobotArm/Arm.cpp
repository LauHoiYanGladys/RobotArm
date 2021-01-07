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

	// add DH frame to frame collection
	theFrames.push_back(secondFrame);

	// define the third DH frame 
	DHframe* thirdFrame = new DHframe(0., 0., 25., 0);
	thirdFrame->assignParentDHframe(secondFrame);

	// add DH frame to frame collection
	theFrames.push_back(thirdFrame);

	// create the third (prismatic) joint
	// note that it corresponds to the third DH frame instead of second DH frame
	Joint* thirdJoint = new Joint(thirdFrame, 25.);
	thirdJoint->setJointTypePrismatic();

	// add the joint to the joint collection
	theJoints.push_back(thirdJoint);

	//// define the fourth DH frame 
	//DHframe* fourthFrame = new DHframe(0., 0., 25., 0);
	//fourthFrame->assignParentDHframe(thirdFrame);

	//// add DH frame to frame collection
	//theFrames.push_back(fourthFrame);
	
}
