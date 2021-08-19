#include "Joint.h"
#include "DrawingUtilNG.h"
#include "fssimplewindow.h"


const double Joint::PI = 3.1415927;

void Joint::setJointTypePrismatic()
{
	type = prismatic;
}

void Joint::setJointTypeRevolute()
{
	type = revolute;
}

void Joint::updateJointVariable(double newJointVariable)
{
	jointVariable = newJointVariable;
}


