#pragma once


//#include "DHframe.h"
#include "Link.h"

class Joint {
public:

	static const double PI;

	/*Vector3d zAxis;
	Matrix3d rotationMat;*/
	enum jointType { revolute, prismatic };

	// so that each joint knows its own joint variable, a quantity generally stored in the next DH frame instead of its own
	// espcially useful in drawing prismatic joints (which changes length)
	double jointVariable; 

	/*DHframe* frame;*/
	Link* link;
	//Joint* parent; // nullptr for the first joint
	jointType type;

	// constructor taking DHframe as parameter
	Joint(/*DHframe* theFrame, *//*double link_length*/) {
		//frame = theFrame;
		//zAxis = theFrame->getZAxis();
		/*parent = nullptr;*/
		/*link = new Link(link_length, Link::alongZ);*/
		type = revolute;
		jointVariable = 0.;
	};

	// draws joint at the correct position and orientation
	/*void draw();*/

	// assigns parent joint
	/*void assignParentJoint(Joint* theParent);*/

	//assigns a length to the link that belongs to the joint
	void assignLinkLength(double _length) { link->length = _length; }

	//assigns a direction to the link that belongs to the joint
	/*void assignLinkDirection(Link::linkDirection _direction) { link->direction = _direction; }*/

	// set joint type as prismatic
	void setJointTypePrismatic();

	// set joint type as revolute (may be useful when editing an existing arm)
	void setJointTypeRevolute();

	// updates joint variable
	void updateJointVariable(double newJointVariable);

};