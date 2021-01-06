#pragma once

#include "DHframe.h"
#include "Link.h"

class Joint {
private:
	Vector3d center;
	Vector3d zAxis;
	Matrix3d rotationMat;
	enum jointType { revolute, prismatic };
	//Link* child;
	Joint* parent; // nullptr for the first joint
public:
	static const double PI;
	// constructor taking DHframe as parameter
	Joint(DHframe theFrame) {
		center = theFrame.getCenter();
		zAxis = theFrame.getZAxis();
		rotationMat = theFrame.getRotationMat();
		/*child = nullptr;*/
		parent = nullptr;
	};

	// draws joint at the correct position and orientation
	void draw();

	// assigns parent joint
	void assignParentJoint(Joint* theParent);

	// gets center
	Vector3d getCenter() { return center; };

	// gets rotation matrix
	Matrix3d getRotationMat() { return rotationMat; };

	// gets world center
	Vector3d getWorldCenter();

	// gets world rotation matrix
	Matrix3d getWorldRotationMat();
};