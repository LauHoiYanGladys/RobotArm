#pragma once
#include <math.h>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

//class Joint;

class DHframe {
public:
	Matrix4d transformMatrix;
	Matrix4d worldTransformMatrix;
	double theta; // variable part of jointAngle, may be varied if relevant joint is moving
	double d; // variable part of linkOffset, may be varied if relevant joint is moving
	double theta_fixed; // fixed part of jointAngle
	double d_fixed; // fixed part of linkOffset

	// DH parameters
	double linkLength, linkTwist;
	double linkOffset; // d_fixed + d
	double jointAngle; // theta_fixed + theta

	DHframe* parent; // pointer to previous DH frame
	//Joint* movingJoint; // pointer to the joint that actually possess that joint variable
public:
	// constructor for the 0th DH frame
	DHframe() {
		theta = 0.;
		d = 0.;
		theta_fixed = 0.;
		d_fixed = 0.;
		linkLength = 0.;
		linkTwist = 0.;
		linkOffset = 0.;
		jointAngle = 0.;
		transformMatrix << 1, 0, 0, 0,
							0, 1, 0, 0,
							0, 0, 1, 0,
							0, 0, 0, 1;
		parent = nullptr;
		//movingJoint = nullptr;
		// the method used before implementing rotation about x-axis at drawing
		//transformMatrix << 1, 0, 0, 0,
		//					0, 0, 1, 0,
		//					0, -1, 0, 0,
		//					0, 0, 0, 1;
	};

	// constructor for all DH frames except the 0th
	DHframe(double theLinkLength, double theLinkTwist, double theLinkOffset, double theJointAngle) {
		theta = 0.;
		d = 0.;
		theta_fixed = theJointAngle;
		d_fixed = theLinkOffset;
		linkLength = theLinkLength;
		linkTwist = theLinkTwist;
		linkOffset = d_fixed + d;
		jointAngle = theta_fixed + theta;
		initialize();
		parent = nullptr;
		//movingJoint = nullptr;

	}; 

	// initialize (or update) transform matrix from the DH parameters
	void initialize();

	// returns z-axis of the frame (second last column without the element at the end)
	Vector3d getZAxis();
	
	// assigns parent DH frame (i.e. the previous DH frame)
	void assignParentDHframe(DHframe* theParent);

	// assigns moving joint (i.e. the joint that moves when the d or theta is changed)
	/*void assignMovingJoint(Joint* theMovingJoint);*/

	// gets world center
	Vector3d getWorldCenter();

	// gets world rotation matrix
	Matrix3d getWorldRotationMat();

	// gets world transform matrix
	Matrix4d getWorldTransformMatrix();

	// gets translation part of a homogeneous transform matrix (last column without the element at the end)
	Vector3d getTranslation(Matrix4d theMatrix);

	// gets rotation part of a homogeneous transform matrix (left upper 3x3 block)
	Matrix3d getRotation(Matrix4d theMatrix);

	// updates the transformation matrix as joint variables change (note that link length and link twist do not change)
	/*void updateTransformMatrix();*/

	// update link length
	void updateLinkLength(double newLinkLength);

	// update link twist
	void updateLinkTwist(double newLinkTwist);

	// update fixed part of link offset
	void update_d_fixed(double new_d_fixed);

	// update variable part of link offset
	void update_d(double new_d);

	// recompute link offset and update it
	void updateLinkOffset();

	// update fixed part of joint angle
	void update_theta_fixed(double new_theta_fixed);

	// update variable part of joint angle
	void update_theta(double new_theta);

	// recompute joint angle and update it
	void updateJointAngle();

	// update joint variable of moving joint
	//void updateMovingJoint();

	// print transform matrix (for debugging)
	void printTransformMatrix();
};