#include <vector>
#include "DHframe.h"

void DHframe::initialize()
{

	transformMatrix(0, 0) = cos(jointAngle);
	transformMatrix(0, 1) = -sin(jointAngle) * cos(linkTwist);
	transformMatrix(0, 2) = sin(jointAngle) * sin(linkTwist);
	transformMatrix(0, 3) = linkLength * cos(jointAngle);
	transformMatrix(1, 0) = sin(jointAngle);
	transformMatrix(1, 1) = cos(jointAngle) * cos(linkTwist);
	transformMatrix(1, 2) = -cos(jointAngle) * sin(linkTwist);
	transformMatrix(1, 3) = linkLength * sin(jointAngle);
	transformMatrix(2, 0) = 0;
	transformMatrix(2, 1) = sin(linkTwist);
	transformMatrix(2, 2) = cos(linkTwist);
	transformMatrix(2, 3) = linkOffset;
	transformMatrix(3, 0) = 0;
	transformMatrix(3, 1) = 0;
	transformMatrix(3, 2) = 0;
	transformMatrix(3, 3) = 1;
}

Vector3d DHframe::getCenter()
{
	Vector3d center = transformMatrix.block<3, 1>(0, 3);
	return center;
	
}

Vector3d DHframe::getZAxis()
{
	Vector3d zAxis = transformMatrix.block<3, 1>(0, 2);
	return zAxis;
}

Matrix3d DHframe::getRotationMat()
{
	Matrix3d rotationMat = transformMatrix.block<3, 3>(0, 0);
	return rotationMat;
}

void DHframe::updateTransformMatrix(double newLinkOffset, double newJointAngle)
{
	
	updateLinkOffset(newLinkOffset);
	updateJointAngle(newJointAngle);
	initialize();
	
}

void DHframe::updateLinkLength(double newLinkLength)
{
	linkLength = newLinkLength;
}

void DHframe::updateLinkTwist(double newLinkTwist)
{
	linkTwist = newLinkTwist;
}

void DHframe::updateLinkOffset(double newLinkOffset)
{
	linkOffset = newLinkOffset;
}

void DHframe::updateJointAngle(double newJointAngle)
{
	jointAngle = newJointAngle;
}

void DHframe::printTransformMatrix()
{
	std::cout << transformMatrix << std::endl;
}
