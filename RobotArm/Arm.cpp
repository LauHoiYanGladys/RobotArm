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
	alpha = 0.00001; // learning rate for revolute joint
	alphaPris = 0.5; // learning rate for prismatic joint
	costChangeStopThreshold = NULL; // slow response at certain goal positions if cost change is an IK stop criterion for this arm

	// define the 0th DH frame
	DHframe* zerothFrame = new DHframe();

	// create first link and assign to 0th DH frame
	Link* firstLink = new Link(25);
	zerothFrame->assignLink(firstLink, Link::linkDirection::alongZ);

	// create the first (revolute) joint, assign to 0th DH frame
	Joint* firstJoint = new Joint();
	zerothFrame->assignJoint(firstJoint);
	jointFrameMap.insert({ firstJoint, zerothFrame });
	frameWithEndPoints.push_back(0);

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
	jointFrameMap.insert({ secondJoint, firstFrame });
	frameWithEndPoints.push_back(1);

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
	secondFrame->assignLink(secondLink, Link::linkDirection::alongZ);

	// add DH frame to frame collection
	theFrames.push_back(secondFrame);

	// define the third DH frame, note that it has no associated joint 
	DHframe* thirdFrame = new DHframe(0., 0., 25., 0);
	thirdFrame->assignParentDHframe(secondFrame);

	// create the third link and assign it to the third DH frame
	Link* thirdLink = new Link(25);
	thirdFrame->assignLink(thirdLink, Link::linkDirection::alongZ);

	// create the third (prismatic) joint
	// note that it corresponds to the third DH frame instead of second DH frame
	Joint* thirdJoint = new Joint();
	thirdJoint->setJointTypePrismatic();
	thirdFrame->assignJoint(thirdJoint);
	jointFrameMap.insert({ thirdJoint, thirdFrame });
	frameWithEndPoints.push_back(3);

	// add DH frame to frame collection
	theFrames.push_back(thirdFrame);

	// add the joint to the joint collection
	theJoints.push_back(thirdJoint);

	// define the fourth DH frame (end-effector frame)
	DHframe* fourthFrame = new DHframe(0., 0., 25., -PI/2);
	fourthFrame->assignParentDHframe(thirdFrame);
	frameWithEndPoints.push_back(4);

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

void Arm::buildArm_PUMA560()
{
	alpha = 0.001; // learning rate
	costChangeStopThreshold = 0.02;

	// some link parameters
	double secondLinkOffset = 0.; // real PUMA560 has non-zero offset here, but I cannot get it to work with that, as the IK is somehow always computed as if this secondLinkOffset is zero; don't change now as it will affect the correctness of the frameWithEndPoints
	double firstLinkLength = 25.;
	double secondLinkLength = 25.;
	double thirdLinkLength = 15.;

	// define the 0th DH frame
	DHframe* zerothFrame = new DHframe();

	// create first link and assign to 0th DH frame
	Link* firstLink = new Link(firstLinkLength);
	zerothFrame->assignLink(firstLink, Link::linkDirection::alongZ);

	// create the first (revolute) joint, assign to 0th DH frame
	Joint* firstJoint = new Joint();
	zerothFrame->assignJoint(firstJoint);
	jointFrameMap.insert({ firstJoint, zerothFrame });
	frameWithEndPoints.push_back(0);

	// add the joint to the joint collection
	theJoints.push_back(firstJoint);

	// add DH frame to frame collection
	theFrames.push_back(zerothFrame);

	// define the first DH frame, note that it has no associated joint 
	DHframe* firstFrame = new DHframe(0., 0., firstLinkLength, 0.);
	firstFrame->assignParentDHframe(zerothFrame);

	// record the frame index 1 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(1);

	// add DH frame to frame collection
	theFrames.push_back(firstFrame);

	// define the second DH frame
	DHframe* secondFrame = new DHframe(0., -PI/2, 0., 0.);
	secondFrame->assignParentDHframe(firstFrame);

	//// record the frame index 2 in the collection of indices of frames containing jointVariables
	//frameWithJointVariable.push_back(2);

	// create the second (revolute) joint, assign it to 2nd DH frame
	Joint* secondJoint = new Joint();
	secondFrame->assignJoint(secondJoint);
	jointFrameMap.insert({ secondJoint, secondFrame });
	frameWithEndPoints.push_back(2);

	// add the second joint to the joint collection
	theJoints.push_back(secondJoint);

	//// create the second link with offset along Y and assign it to the second DH frame
	//Link* secondLink = new Link(secondLinkLength);
	//secondFrame->assignLink(secondLink, Link::linkDirection::alongX, Link::linkOffsetDirection::alongZOffset, secondLinkOffset);

	// add DH frame to frame collection
	theFrames.push_back(secondFrame);

	// define the third DH frame, note its offset from the second DH frame
	DHframe* thirdFrame = new DHframe(secondLinkLength, 0., secondLinkOffset, 0.);
	thirdFrame->assignParentDHframe(secondFrame);

	// create the second link and assign it to the third DH frame
	Link* secondLink = new Link(secondLinkLength);
	thirdFrame->assignLink(secondLink, Link::linkDirection::alongX_negative);

	// record the frame index 3 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(3);

	// create the third (revolute) joint
	// note that it corresponds to the third DH frame instead of second DH frame
	Joint* thirdJoint = new Joint();
	thirdFrame->assignJoint(thirdJoint);
	jointFrameMap.insert({ thirdJoint, thirdFrame });
	frameWithEndPoints.push_back(3);

	// add DH frame to frame collection
	theFrames.push_back(thirdFrame);

	// add the joint to the joint collection
	theJoints.push_back(thirdJoint);

	// define the fourth DH frame (no associated joint)
	DHframe* fourthFrame = new DHframe(0., -PI/2, 0., 0.);
	fourthFrame->assignParentDHframe(thirdFrame);

	// create the third link and assign it to the fourth DH frame (not sure why the third link always takes a moment to be drawn at the correct position - to be debugged)
	Link* thirdLink = new Link(thirdLinkLength);
	fourthFrame->assignLink(thirdLink, Link::linkDirection::alongZ);

	// add DH frame to frame collection
	theFrames.push_back(fourthFrame);

	// record the frame index 4 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(4);

	// define the fifth DH frame (end-effector frame)
	DHframe* fifthFrame = new DHframe(0., 0., thirdLinkLength, 0.);
	fifthFrame->assignParentDHframe(fourthFrame);
	frameWithEndPoints.push_back(5);

	// add DH frame to frame collection
	theFrames.push_back(fifthFrame);

}


void Arm::moveArm(std::vector<double> jointVariables)
{
	assert(jointVariables.size() == frameWithJointVariable.size());
	for (int i = 0; i < frameWithJointVariable.size(); i++) {
		int currFrameIndex = frameWithJointVariable[i];
		if (theJoints[i]->type == Joint::revolute) {
			theFrames[currFrameIndex]->update_theta(jointVariables[i]);	
		}
		else if (theJoints[i]->type == Joint::prismatic) {
			theFrames[currFrameIndex]->update_d(jointVariables[i]);
		}
		theJoints[i]->updateJointVariable(jointVariables[i]);	
	}
	// also updates test frames to ensure next test frame computation is based on actual joint variables 
	jointVariables[0] = -jointVariables[0]; // for some uncertain reason, test frames only come out right if the first joint variable fed into it is the negative of the first joint variable fed into moveArm
	updateTestFrames(jointVariables);

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


Vector3d/*double*/ Arm::getTestJointVariable()
{
	Vector3d testJointVariables;
	assert(testJointVariables.size() == frameWithJointVariable.size());
	for (int i = 0; i < frameWithJointVariable.size(); i++) {
		int currFrameIndex = frameWithJointVariable[i];
		if (theJoints[i]->type == Joint::revolute)
			testJointVariables(i) = theFrames[currFrameIndex]->test_theta;
		else if (theJoints[i]->type == Joint::prismatic)
			testJointVariables(i) = theFrames[currFrameIndex]->test_d;
	}
	/*double testJointVariables;*/
	/*testJointVariables = theFrames[1]->test_theta;*/
	/*testJointVariables << theFrames[1]->test_theta, theFrames[2]->test_theta, theFrames[4]->test_d;*/
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

std::vector<DrawingUtilNG::vertexF> Arm::compute_test_FK_all()
{
	std::vector<DrawingUtilNG::vertexF>res;

	for (int i = 0; i < frameWithEndPoints.size(); i++) {
		int currFrameIndex = frameWithEndPoints[i];
		Vector3d tempVector3d = compute_test_FK(currFrameIndex);
		std::cout << "tempVector3d is " << tempVector3d(0) << ", " << tempVector3d(1) << ", " << tempVector3d(2) << std::endl;
		DrawingUtilNG::vertexF tempVertexF = { (float)tempVector3d(0), (float)tempVector3d(1),(float)tempVector3d(2) };
		std::cout << "tempVertexF is " << (float)tempVector3d(0) << ", " << (float)tempVector3d(1) << ", " << (float)tempVector3d(2) << std::endl;
		res.push_back(tempVertexF);
	}
	return res;
}

void Arm::testing_compute_test_FK_all()
{
	std::vector<DrawingUtilNG::vertexF>res = compute_test_FK_all();
	std::cout << "Waiting" << std::endl; //set a stop on this line and check value of res by hovering mouse over the "res" on previous line
}


