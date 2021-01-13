#pragma once
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include "Link.h"
#include "Joint.h"
#include "fssimplewindow.h"
#include "DrawingUtilNG.h"

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

	// used for inverse kinematics computation, but not actual drawing
	Matrix4d test_transformMatrix;
	Matrix4d test_worldTransformMatrix;
	double test_theta; // variable part of jointAngle, may be varied if relevant joint is moving
	double test_d; // variable part of linkOffset, may be varied if relevant joint is moving
	double test_linkOffset; // d_fixed + test_d
	double test_jointAngle; // theta_fixed + test_theta

	DHframe* parent; // pointer to previous DH frame
	Link* link;
	Joint* joint;
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

		test_theta = 0.;
		test_d = 0.;
		test_linkOffset = 0.;
		test_jointAngle = 0.;
		test_transformMatrix << 1, 0, 0, 0,
								0, 1, 0, 0,
								0, 0, 1, 0,
								0, 0, 0, 1;
		parent = nullptr;
		link = nullptr;
		joint = nullptr;
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

		test_theta = 0.;
		test_d = 0.;
		test_linkOffset = d_fixed + test_d;
		test_jointAngle = theta_fixed + test_theta;
		test_initialize();
		
		parent = nullptr;
		link = nullptr;
		joint = nullptr;
		//movingJoint = nullptr;

	}; 

	// initialize (or update) transform matrix from the DH parameters
	void initialize();

	// initialize (or update) test transform matrix from the DH parameters
	void test_initialize();

	// returns z-axis of the frame (second last column without the element at the end)
	Vector3d getZAxis();

	// returns z-axis of the frame in world coordinates (second last column without the element at the end)
	Vector3d getZAxisWorldTest();
	
	// assigns parent DH frame (i.e. the previous DH frame)
	void assignParentDHframe(DHframe* theParent);

	// assigns joint
	void assignJoint(Joint* theJoint);

	// assign link that follows one of its axis
	void assignLink(Link* theLink, Link::linkDirection _direction);

	// assign link direction
	void assignLinkDirection(Link::linkDirection _direction) { link->direction = _direction; }

	// gets world center (the parameter indicates whether it is the test_worldTransformMatrix or not)
	Vector3d getWorldCenter(bool isTest = false);

	// gets world rotation matrix (the parameter indicates whether it is the test_worldTransformMatrix or not)
	Matrix3d getWorldRotationMat(bool isTest = false);

	// gets world transform matrix (the parameter indicates whether it is the test_worldTransformMatrix or not)
	Matrix4d getWorldTransformMatrix(bool isTest = false);

	// gets translation part of a homogeneous transform matrix (last column without the element at the end)
	static Vector3d getTranslation(Matrix4d theMatrix);

	// gets rotation part of a homogeneous transform matrix (left upper 3x3 block)
	static Matrix3d getRotation(Matrix4d theMatrix);

	// update link length
	void updateLinkLength(double newLinkLength);

	// update link twist
	void updateLinkTwist(double newLinkTwist);

	// update fixed part of link offset
	void update_d_fixed(double new_d_fixed);

	// update variable part of link offset
	void update_d(double new_d);

	// update variable part of test link offset
	void update_test_d(double new_test_d);

	// recompute link offset and update it
	void updateLinkOffset(bool isTest = false);

	// update fixed part of joint angle
	void update_theta_fixed(double new_theta_fixed);

	// update variable part of joint angle
	void update_theta(double new_theta);

	// update variable part of test joint angle
	void update_test_theta(double new_test_theta);

	// recompute joint angle and update it
	void updateJointAngle(bool isTest = false);

	// updates both worldTransformMatrix and test_worldTransformMatrix
	void updateWorldTransformMatrices();

	// print transform matrix (for debugging)
	void printTransformMatrix();

	// draws link & joint associated with frame
	void draw() {
		drawLink();
		drawJoint();
	}

	// draws link
	void drawLink();

	// draws joint
	void drawJoint();
};