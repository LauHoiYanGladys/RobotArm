#include <stdexcept>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>    // std::max
#include "Joint.h"
#include "DrawingUtilNG.h"
#include "fssimplewindow.h"


const double Joint::PI = 3.1415927;

//void Joint::draw()
//{
//	// store current matrix state
//	glPushMatrix();
//
//	// first, rotate frame 90 degrees counterclockwise about the x-axis
//	// because openGL has y-axis up, while homogeneous transformation has z-axis up
//	glRotatef(90, -1, 0, 0);
//
//	// translate into the local frame
//	Vector3d worldCenter = frame->getWorldCenter();
//	/*std::cout << "World center of joint" << std::endl << worldCenter << std::endl;*/
//
//	glTranslatef(worldCenter(0), worldCenter(1), worldCenter(2));
//
//	// rotate into the local frame
//	// get rotation matrix in world coordinates
//	Matrix3d worldRotation = frame->getWorldRotationMat();
//	/*std::cout << "World rotation of joint" << std::endl << worldRotation << std::endl;*/
//
//	// get angle-axis representation from rotation matrix
//	Eigen::AngleAxisd theAngleAxis(worldRotation);
//	double angleRotate = theAngleAxis.angle();
//	// convert from radian to degree
//	angleRotate = angleRotate / PI * 180;
//	Vector3d axisRotate = theAngleAxis.axis();
//	glRotatef(angleRotate, axisRotate(0), axisRotate(1), axisRotate(2));
//
//	//drawing of the joint
//	glColor3ub(0, 0, 255);	//blue
//	if (theJointType == revolute)
//		DrawingUtilNG::drawCylinderZ(2., 2., 10., 0, 0, 0);
//	else if (theJointType == prismatic) {
//		// prismatic joint itself without extension
//		DrawingUtilNG::drawPrismZ(4., 4., 10., 0, 0, 0);
//		glColor3ub(0, 191, 255);	//deep sky blue
//		// extension of prismatic joint ("center" shifted half the joint's original length in the z-direction)
//		DrawingUtilNG::drawPrismZOffset(4., 4., jointVariable, 0, 0, 5.);
//	}
//		
//
//	//drawing of the link
//	//glColor3ub(255, 0, 0);	//red
//	//if (theJointType == revolute)
//	//	link->draw_simple();
//	//else if (theJointType == prismatic)
//	//	link->draw_offset_prismatic(jointVariable);
//
//	// restore original matrix state
//	glPopMatrix();
//}

//void Joint::assignParentJoint(Joint* theParent)
//{
//	parent = theParent;
//}

void Joint::setJointTypePrismatic()
{
	theJointType = prismatic;
}

void Joint::setJointTypeRevolute()
{
	theJointType = revolute;
}

void Joint::updateJointVariable(double newJointVariable)
{
	jointVariable = newJointVariable;
}


