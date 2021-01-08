#include "Arm.h"
#include "DHframe.h"
#include "Joint.h"
#include "Link.h"
#include "fssimplewindow.h"

const double Arm::PI = 3.1415927;

class Joint;

using namespace Eigen;

void Arm::draw()
{
		

	for (auto& frame : theFrames) {
		// store current matrix state
		glPushMatrix();

		// first, rotate frame 90 degrees counterclockwise about the x-axis
		// because openGL has y-axis up, while homogeneous transformation has z-axis up
		glRotatef(90, -1, 0, 0);

		// translate into the local frame
		Vector3d worldCenter = frame->getWorldCenter();
		/*std::cout << "World center of joint" << std::endl << worldCenter << std::endl;*/

		glTranslatef(worldCenter(0), worldCenter(1), worldCenter(2));

		// rotate into the local frame
		// get rotation matrix in world coordinates
		Matrix3d worldRotation = frame->getWorldRotationMat();
		/*std::cout << "World rotation of joint" << std::endl << worldRotation << std::endl;*/

		// get angle-axis representation from rotation matrix
		Eigen::AngleAxisd theAngleAxis(worldRotation);
		double angleRotate = theAngleAxis.angle();
		// convert from radian to degree
		angleRotate = angleRotate / PI * 180;
		Vector3d axisRotate = theAngleAxis.axis();
		glRotatef(angleRotate, axisRotate(0), axisRotate(1), axisRotate(2));

		//drawing of the joint
		frame->drawJoint();

		//drawing of the link
		frame->drawLink();

		// restore original matrix state
		glPopMatrix();

	}

}

void Arm::buildArm()
{
	// define the 0th DH frame
	DHframe* zerothFrame = new DHframe();

	// create first link and assign to 0th DH frame
	Link* firstLink = new Link(25);
	zerothFrame->assignLink(firstLink, Link::alongZ);

	// create the first (revolute) joint, assign to 0th DH frame
	Joint* firstJoint = new Joint();
	zerothFrame->assignJoint(firstJoint);

	// add the joint to the joint collection
	theJoints.push_back(firstJoint);

	// add DH frame to frame collection
	theFrames.push_back(zerothFrame);

	
	// define the first DH frame 
	DHframe* firstFrame = new DHframe(0., PI/2, 25., 0.);
	firstFrame->assignParentDHframe(zerothFrame);

	// create the second (revolute) joint, assign it to 1st DH frame
	Joint* secondJoint = new Joint();
	firstFrame->assignJoint(secondJoint);

	// add DH frame to frame collection
	theFrames.push_back(firstFrame);


	// add the joint to the joint collection
	theJoints.push_back(secondJoint);

	// define the second DH frame
	DHframe* secondFrame = new DHframe(0., PI / 2, 0., PI / 2);
	secondFrame->assignParentDHframe(firstFrame);


	// create the second link and assign it to the second DH frame
	Link* secondLink = new Link(25);
	secondFrame->assignLink(secondLink, Link::alongZ);

	// add DH frame to frame collection
	theFrames.push_back(secondFrame);

	// define the third DH frame, note that it has no associated joint 
	DHframe* thirdFrame = new DHframe(0., 0., 25., 0);
	thirdFrame->assignParentDHframe(secondFrame);

	// create the third link and assign it to the third DH frame
	Link* thirdLink = new Link(25);
	thirdFrame->assignLink(thirdLink, Link::alongZ);

	// create the third (prismatic) joint
	// note that it corresponds to the third DH frame instead of second DH frame
	Joint* thirdJoint = new Joint();
	thirdJoint->setJointTypePrismatic();
	thirdFrame->assignJoint(thirdJoint);

	// add DH frame to frame collection
	theFrames.push_back(thirdFrame);

	// add the joint to the joint collection
	theJoints.push_back(thirdJoint);

	// define the fourth DH frame 
	DHframe* fourthFrame = new DHframe(0., 0., 25., -PI/2);
	fourthFrame->assignParentDHframe(thirdFrame);

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
