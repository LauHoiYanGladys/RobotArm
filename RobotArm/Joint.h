#pragma once

#include "Link.h"

class Joint {
public:

	static const double PI;

	enum jointType { revolute, prismatic };

	// so that each joint knows its own joint variable, a quantity generally stored in the next DH frame instead of its own
	// espcially useful in drawing prismatic joints (which changes length)
	double jointVariable; 

	Link* link;
	jointType type;

	// constructor
	Joint() {
		type = revolute;
		jointVariable = 0.;
	};

	//assigns a length to the link that belongs to the joint
	void assignLinkLength(double _length) { link->length = _length; }

	// set joint type as prismatic
	void setJointTypePrismatic();

	// set joint type as revolute (may be useful when editing an existing arm)
	void setJointTypeRevolute();

	// updates joint variable
	void updateJointVariable(double newJointVariable);
};