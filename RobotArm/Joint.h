#pragma once

#include "DHframe.h"
#include "Link.h"

class Joint {
public:

	static const double PI;

	Vector3d center;
	Vector3d zAxis;
	Matrix3d rotationMat;
	enum jointType { revolute, prismatic };

	DHframe* frame;
	Link* link;
	Joint* parent; // nullptr for the first joint

	// constructor taking DHframe as parameter
	Joint(DHframe theFrame, double link_length) {
		center = theFrame.getCenter();
		zAxis = theFrame.getZAxis();
		rotationMat = theFrame.getRotationMat();
		/*child = nullptr;*/
		parent = nullptr;
		link = new Link(link_length, Link::alongZ);
	};

	// draws joint at the correct position and orientation
	void draw();

	// assigns parent joint
	void assignParentJoint(Joint* theParent);

	//// gets center
	//Vector3d getCenter() { return center; };

	//// gets rotation matrix
	//Matrix3d getRotationMat() { return rotationMat; };

	// gets world center
	Vector3d getWorldCenter();

	// gets world rotation matrix
	Matrix3d getWorldRotationMat();

	//assigns a length to the link that belongs to the joint
	void assignLinkLength(double _length) { link->length = _length; }

	//assigns a direction to the link that belongs to the joint
	void assignLinkDirection(Link::linkDirection _direction) { link->direction = _direction; }
};