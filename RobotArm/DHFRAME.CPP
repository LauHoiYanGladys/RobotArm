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

Vector3d DHframe::getZAxis()
{
	Vector3d zAxis = transformMatrix.block<3, 1>(0, 2);
	return zAxis;
}


void DHframe::assignParentDHframe(DHframe* theParent)
{
	parent = theParent;
	worldTransformMatrix = getWorldTransformMatrix(); // can be assigned once parent DH frame is known
	/*std::cout << "World transform matrix: " << std::endl << worldTransformMatrix << std::endl;*/
}

Vector3d DHframe::getWorldCenter()
{
	return getTranslation(worldTransformMatrix);
}

Matrix3d DHframe::getWorldRotationMat()
{
	return getRotation(worldTransformMatrix);
}

Matrix4d DHframe::getWorldTransformMatrix()
{
	if (parent == nullptr) // base case
		return transformMatrix;
	else {
		/*std::cout << "parent transform matrix" << std::endl << parent->worldTransformMatrix << std::endl;*/
		return  (parent->worldTransformMatrix) * transformMatrix;
	}
		
}

Vector3d DHframe::getTranslation(Matrix4d theMatrix)
{
	Vector3d translation = theMatrix.block<3, 1>(0, 3);
	return translation;
}

Matrix3d DHframe::getRotation(Matrix4d theMatrix)
{
	Matrix3d rotation = theMatrix.block<3, 3>(0, 0);
	return rotation;

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
