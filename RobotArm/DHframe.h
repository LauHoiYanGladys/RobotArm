#pragma once
#include <math.h>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class DHframe {
public:
	Matrix4d transformMatrix;
	double linkLength, linkTwist, linkOffset, jointAngle;
public:
	// constructor for the 0th DH frame
	DHframe() {
		linkLength = 0.;
		linkTwist = 0.;
		linkOffset = 0.;
		jointAngle = 0.;
		transformMatrix << 1, 0, 0, 0,
							0, 1, 0, 0,
							0, 0, 1, 0,
							0, 0, 0, 1;
		
		// the method used before implementing rotation about x-axis at drawing
		//transformMatrix << 1, 0, 0, 0,
		//					0, 0, 1, 0,
		//					0, -1, 0, 0,
		//					0, 0, 0, 1;
	};

	// constructor for all DH frames except the 0th
	DHframe(double theLinkLength, double theLinkTwist, double theLinkOffset, double theJointAngle) {
		linkLength = theLinkLength;
		linkTwist = theLinkTwist;
		linkOffset = theLinkOffset;
		jointAngle = theJointAngle;
		initialize();
	}; 

	// initialize transform matrix from the DH parameters
	void initialize();

	// returns center of the frame (last column without the element at the end)
	Vector3d getCenter();

	// returns z-axis of the frame (second last column without the element at the end)
	Vector3d getZAxis();

	// returns the rotation matrix
	Matrix3d getRotationMat();
	
	// updates the transformation matrix as joint variables change (note that link length and link twist do not change)
	void updateTransformMatrix(double linkOffset, double jointAngle);

	// update link length
	void updateLinkLength(double newLinkLength);

	// update link twist
	void updateLinkTwist(double newLinkTwist);

	// update link offset
	void updateLinkOffset(double newLinkOffset);

	// update joint angle
	void updateJointAngle(double newJointAngle);

	// print transform matrix (for debugging)
	void printTransformMatrix();
};