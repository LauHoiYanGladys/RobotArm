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

		//Note that the coordinate system is made right-handed in fssimplewindow, with z-axis pointing out of the screen

		// rotate frame 90 degrees counterclockwise about the x-axis
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
	double secondLinkOffset = 5.; 
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

	// create the second (revolute) joint, assign it to 2nd DH frame
	Joint* secondJoint = new Joint();
	secondFrame->assignJoint(secondJoint);
	jointFrameMap.insert({ secondJoint, secondFrame });
	frameWithEndPoints.push_back(2);

	// add the second joint to the joint collection
	theJoints.push_back(secondJoint);

	// add DH frame to frame collection
	theFrames.push_back(secondFrame);

	// define the third DH frame (defined to make it easy finding the positions of end points when secondLinkOffset is not zero) 
	DHframe* thirdFrame = new DHframe(0., 0., secondLinkOffset, 0.);
	thirdFrame->assignParentDHframe(secondFrame);
	frameWithEndPoints.push_back(3);
	// record the frame index 3 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(3);

	// create the second link and assign it to the third DH frame
	Link* secondLink = new Link(secondLinkLength);
	thirdFrame->assignLink(secondLink, Link::linkDirection::alongX);

	// add DH frame to frame collection
	theFrames.push_back(thirdFrame);

	// define the fourth DH frame
	DHframe* fourthFrame = new DHframe(secondLinkLength, 0., 0., 0.);
	fourthFrame->assignParentDHframe(thirdFrame);

	// create the third (revolute) joint
	// note that it corresponds to the fourth DH frame 
	Joint* thirdJoint = new Joint();
	fourthFrame->assignJoint(thirdJoint);
	jointFrameMap.insert({ thirdJoint, fourthFrame });
	frameWithEndPoints.push_back(4);

	// add the joint to the joint collection
	theJoints.push_back(thirdJoint);

	// add DH frame to frame collection
	theFrames.push_back(fourthFrame);

	// define the fifth DH frame (no associated joint, just defined to conform to the DH convention)
	DHframe* fifthFrame = new DHframe(0., -PI/2, 0., 0.);
	fifthFrame->assignParentDHframe(fourthFrame);

	// create the third link and assign it to the fifth DH frame 
	Link* thirdLink = new Link(thirdLinkLength);
	fifthFrame->assignLink(thirdLink, Link::linkDirection::alongZ);

	// add DH frame to frame collection
	theFrames.push_back(fifthFrame);

	// record the frame index 5 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(5);

	// define the sixth DH frame (end-effector frame)
	DHframe* sixthFrame = new DHframe(0., 0., thirdLinkLength, 0.);
	sixthFrame->assignParentDHframe(fifthFrame);
	frameWithEndPoints.push_back(6);

	// add DH frame to frame collection
	theFrames.push_back(sixthFrame);

}

void Arm::buildArm_SCARA()
{
	alpha = 0.001; // learning rate for revolute joint
	alphaPris = 0.6; // learning rate for prismatic joint
	costChangeStopThreshold = 0.02;


	double zerothLinkLength = 25.;
	double firstLinkOffset = 5.;
	double firstLinkLength = 15.;
	double secondLinkLength = 15.;

	// define the 0th DH frame, note it is BEFORE the first joint (0th frame defined like this to make it easy to make a zeroth link to "elevate" the SCARA arm above ground); 
	DHframe* zerothFrame = new DHframe();

	// create zeroth link and assign to 0th DH frame
	Link* zerothLink = new Link(zerothLinkLength);
	zerothFrame->assignLink(zerothLink, Link::linkDirection::alongZ);
	frameWithEndPoints.push_back(0);

	// add zeroth DH frame to frame collection
	theFrames.push_back(zerothFrame);

	// define the first DH frame
	DHframe* firstFrame = new DHframe(0., 0., zerothLinkLength, 0.);
	firstFrame->assignParentDHframe(zerothFrame);
	frameWithEndPoints.push_back(1); // although this end point is not necessary for the skeleton, it's defined so that each joint consistently coincide with endpoints 

	// create the first (revolute) joint, assign to 1st DH frame
	Joint* firstJoint = new Joint();
	firstFrame->assignJoint(firstJoint);
	jointFrameMap.insert({ firstJoint, firstFrame });
	
	// add the joint to the joint collection
	theJoints.push_back(firstJoint);

	// add first frame to frame collection
	theFrames.push_back(firstFrame);

	// define the second DH frame (defined to make it easy finding the positions of end points when firstLinkOffset is not zero) 
	DHframe* secondFrame = new DHframe(0., 0., firstLinkOffset, 0.);
	secondFrame->assignParentDHframe(firstFrame);
	frameWithEndPoints.push_back(2);
	// record the frame index 2 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(2);

	// create the first link and assign it to the second DH frame
	Link* firstLink = new Link(firstLinkLength);
	secondFrame->assignLink(firstLink, Link::linkDirection::alongX);

	// add DH frame to frame collection
	theFrames.push_back(secondFrame);

	// define the third DH frame
	DHframe* thirdFrame = new DHframe(firstLinkLength, 0., 0., 0.);
	thirdFrame->assignParentDHframe(secondFrame);
	frameWithEndPoints.push_back(3);

	// create the second (revolute) joint
	// note that it corresponds to the third DH frame 
	Joint* secondJoint = new Joint();
	thirdFrame->assignJoint(secondJoint);
	jointFrameMap.insert({ secondJoint, thirdFrame });
	
	// add the joint to the joint collection
	theJoints.push_back(secondJoint);

	// add third DH frame to frame collection
	theFrames.push_back(thirdFrame);

	// define the fourth DH frame
	DHframe* fourthFrame = new DHframe(0., 0., 0., 0.);
	fourthFrame->assignParentDHframe(thirdFrame);
	// record the frame index 4 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(4);

	// create second link and assign to fourth DH frame
	Link* secondLink = new Link(secondLinkLength);
	fourthFrame->assignLink(secondLink, Link::linkDirection::alongX);

	// add fourth DH frame to frame collection
	theFrames.push_back(fourthFrame);

	// define the fifth DH frame
	DHframe* fifthFrame = new DHframe(secondLinkLength, PI, 0., 0.);
	fifthFrame->assignParentDHframe(fourthFrame);
	frameWithEndPoints.push_back(5);
	
	// note that "third link length" is not defined and effectively set to zero
	// create the third (prismatic) joint
	// note that it corresponds to the fifth DH frame 
	Joint* thirdJoint = new Joint();
	thirdJoint->setJointTypePrismatic();
	fifthFrame->assignJoint(thirdJoint);
	jointFrameMap.insert({ thirdJoint, fifthFrame });

	// add the joint to the joint collection
	theJoints.push_back(thirdJoint);

	// add fifth DH frame to frame collection
	theFrames.push_back(fifthFrame);

	// define the sixth DH frame (end-effector frame), note that the last revolute joint in a typical SCARA arm is skipped as we currently only model end-effector's position but not orientation
	DHframe* sixthFrame = new DHframe(0., 0., 0., 0.); 
	sixthFrame->assignParentDHframe(fifthFrame);
	frameWithEndPoints.push_back(6);
	// record the frame index 6 in the collection of indices of frames containing jointVariables
	frameWithJointVariable.push_back(6);

	// add DH frame to frame collection
	theFrames.push_back(sixthFrame);

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


Vector3d Arm::getTestJointVariable()
{
	Vector3d testJointVariables;
	assert(testJointVariables.size() == frameWithJointVariable.size());
	assert(frameWithJointVariable.back() < theFrames.size());
	for (int i = 0; i < frameWithJointVariable.size(); i++) {
		int currFrameIndex = frameWithJointVariable[i];
		if (theJoints[i]->type == Joint::revolute)
			testJointVariables(i) = theFrames[currFrameIndex]->test_theta;
		else if (theJoints[i]->type == Joint::prismatic)
			testJointVariables(i) = theFrames[currFrameIndex]->test_d;
	}

	return testJointVariables;
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
	std::cout << "Testing completed" << std::endl; //set a stop on this line and check value of res by hovering mouse over the "res" on previous line
}


