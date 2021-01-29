#include <assert.h>     /* assert */
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
		Vector3d worldCenter = frame->getWorldCenter(false);
		/*std::cout << "World center of joint" << std::endl << worldCenter << std::endl;*/

		glTranslatef(worldCenter(0), worldCenter(1), worldCenter(2));

		// rotate into the local frame
		// get rotation matrix in world coordinates
		Matrix3d worldRotation = frame->getWorldRotationMat(false);
		/*std::cout << "World rotation of joint" << std::endl << worldRotation << std::endl;*/

		// get angle-axis representation from rotation matrix
		Eigen::AngleAxisd theAngleAxis(worldRotation);
		double angleRotate = theAngleAxis.angle();
		// convert from radian to degree
		angleRotate = angleRotate / PI * 180;
		Vector3d axisRotate = theAngleAxis.axis();
		glRotatef(angleRotate, axisRotate(0), axisRotate(1), axisRotate(2));

		//drawing of the frame
		frame->draw();

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
	/*firstJoint->setJointTypePrismatic();*/
	zerothFrame->assignJoint(firstJoint);
	jointFrameMap.insert({ firstJoint, zerothFrame });

	// add the joint to the joint collection
	theJoints.push_back(firstJoint);

	// add DH frame to frame collection
	theFrames.push_back(zerothFrame);

	
	// define the first DH frame 
	DHframe* firstFrame = new DHframe(0., PI/2, 25., 0.);
	firstFrame->assignParentDHframe(zerothFrame);

	// create the second (revolute) joint, assign it to 1st DH frame
	Joint* secondJoint = new Joint();
	/*secondJoint->setJointTypePrismatic();*/
	firstFrame->assignJoint(secondJoint);
	jointFrameMap.insert({ secondJoint, firstFrame });

	// record the frame index 1 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(1);

	// add DH frame to frame collection
	theFrames.push_back(firstFrame);

	// add the joint to the joint collection
	theJoints.push_back(secondJoint);

	// define the second DH frame
	DHframe* secondFrame = new DHframe(0., PI / 2, 0., PI / 2);
	secondFrame->assignParentDHframe(firstFrame);

	// record the frame index 2 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(2);

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
	jointFrameMap.insert({ thirdJoint, thirdFrame });

	// add DH frame to frame collection
	theFrames.push_back(thirdFrame);

	// add the joint to the joint collection
	theJoints.push_back(thirdJoint);

	// define the fourth DH frame 
	DHframe* fourthFrame = new DHframe(0., 0., 25., -PI/2);
	fourthFrame->assignParentDHframe(thirdFrame);

	// add DH frame to frame collection
	theFrames.push_back(fourthFrame);
	
	// record the frame index 4 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(4);

	// drawing an arm with single revolute joint for debugging
	//// define the 0th DH frame
	//DHframe* zerothFrame = new DHframe();

	//// create the first (revolute) joint, assign to 0th DH frame
	//Joint* firstJoint = new Joint();
	//zerothFrame->assignJoint(firstJoint);
	//jointFrameMap.insert({ firstJoint, zerothFrame });

	//// add the joint to the joint collection
	//theJoints.push_back(firstJoint);

	//// add DH frame to frame collection
	//theFrames.push_back(zerothFrame);

	//// define the first DH frame
	//DHframe* firstFrame = new DHframe(0., PI/2, 0., PI / 2);
	//firstFrame->assignParentDHframe(zerothFrame);

	//// create first link and assign to first DH frame
	//Link* firstLink = new Link(25);
	//firstFrame->assignLink(firstLink, Link::alongZ);

	//// add DH frame to frame collection
	//theFrames.push_back(firstFrame);

	//// define the second DH frame
	//DHframe* secondFrame = new DHframe(0., 0., 25., 0.);
	//secondFrame->assignParentDHframe(firstFrame);

	//// add DH frame to frame collection
	//theFrames.push_back(secondFrame);
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

void Arm::updateTestFrame(int frameNumber, double newJointVariable)
{
	if (frameNumber == 1)
		theFrames[1]->update_test_theta(newJointVariable);
	else if (frameNumber == 2)
		theFrames[2]->update_test_theta(newJointVariable);
	else if (frameNumber == 4)
		theFrames[4]->update_test_d(newJointVariable);
	else
		std::cout << "Invalid frame number, no frame updated" << std::endl;
}

void Arm::updateTestFrame(DHframe* theFrame, double newJointVariable)
{
	if (theFrame == theFrames[1])
		theFrames[1]->update_test_theta(newJointVariable);
	/*else if (theFrame == theFrames[2])
		theFrames[2]->update_test_theta(newJointVariable);
	else if (theFrame == theFrames[4])
		theFrames[4]->update_test_d(newJointVariable);*/
	else
		std::cout << "Invalid frame pointer, no frame updated" << std::endl;
}

void Arm::updateTestFrames(double newJointVariable1, double newJointVariable2, double newJointVariable3)
{
	// updates "theta" or "d" of relevant DH frames
	theFrames[1]->update_test_theta(newJointVariable1);
	theFrames[2]->update_test_theta(newJointVariable2);
	theFrames[4]->update_test_d(newJointVariable3);
	/*std::cout << "Test frames updated" << std::endl;*/
}

void Arm::updateTestFrames(std::vector<double> jointVariables)
{
	assert(jointVariables.size() == frameWithJointVariable.size());
	for (int i = 0; i < frameWithJointVariable.size(); i++) {
		int currFrameIndex = frameWithJointVariable[i];
		if (theJoints[i]->type == Joint::revolute)
			theFrames[currFrameIndex]->update_test_theta(jointVariables[i]);
		else if (theJoints[i]->type == Joint::prismatic)
			theFrames[currFrameIndex]->update_test_d(jointVariables[i]);
	}
}

double Arm::getTestJointVariable(DHframe* theFrame)
{
	if (theFrame != nullptr) {
		if (theFrame == theFrames[1])
			return theFrames[1]->test_theta;
		else if (theFrame == theFrames[2])
			return theFrames[2]->test_theta;
		else if (theFrame == theFrames[4])
			return theFrames[4]->test_d;
		else {
			std::cout << "Invalid frame pointer, null returned" << std::endl;
			return NULL;
		}
	}
	else {
		std::cout << "frame pointer is nullptr, null returned" << std::endl;
		return NULL;
	}
	
}

Vector3d/*double*/ Arm::getTestJointVariable()
{
	Vector3d testJointVariables;
	/*double testJointVariables;*/
	/*testJointVariables = theFrames[1]->test_theta;*/
	testJointVariables << theFrames[1]->test_theta, theFrames[2]->test_theta, theFrames[4]->test_d;
	return testJointVariables;
}

Vector3d Arm::getJointVariable()
{
	Vector3d jointVariables;
	/*jointVariables << theFrames[1]->theta;*/
	jointVariables << theFrames[1]->theta, theFrames[2]->theta, theFrames[4]->d;
	return jointVariables;
}

Vector3d Arm::compute_test_FK(int frameIndex)
{
	Vector3d theFK;
	DHframe* theFrame;
	if (!theFrames.empty() && frameIndex <= theFrames.size() - 1) {
		theFrame = theFrames[frameIndex];
		theFK = theFrame->getWorldCenter(true); 
	}

	return theFK;


}

Vector3d Arm::compute_test_FK(DHframe* theFrame)
{
	Vector3d theFK;
	theFK = theFrame->getWorldCenter(true); 
	return theFK;

	
}
