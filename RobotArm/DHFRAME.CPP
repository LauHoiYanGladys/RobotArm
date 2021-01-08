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

	/*std::cout << "Transform matrix updated to:" << std::endl << transformMatrix << theta << std::endl;*/
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

//void DHframe::assignMovingJoint(Joint* theMovingJoint)
//{
//	movingJoint = theMovingJoint;
//}

Vector3d DHframe::getWorldCenter()
{
	worldTransformMatrix = getWorldTransformMatrix(); // to be 100% sure it's updated
	return getTranslation(worldTransformMatrix);
}

Matrix3d DHframe::getWorldRotationMat()
{
	worldTransformMatrix = getWorldTransformMatrix(); // to be 100% sure it's updated
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

//void DHframe::updateTransformMatrix()
//{
//	
//	updateLinkOffset(newLinkOffset);
//	updateJointAngle(newJointAngle);
//	initialize();
//	
//}

void DHframe::updateLinkLength(double newLinkLength)
{
	linkLength = newLinkLength;
	initialize();
	worldTransformMatrix = getWorldTransformMatrix();
}

void DHframe::updateLinkTwist(double newLinkTwist)
{
	linkTwist = newLinkTwist;
	initialize();
	worldTransformMatrix = getWorldTransformMatrix();
}

void DHframe::update_d_fixed(double new_d_fixed)
{
	d_fixed = new_d_fixed;
	updateLinkOffset();

}

void DHframe::update_d(double new_d)
{
	d = new_d;
	updateLinkOffset();
}

void DHframe::updateLinkOffset()
{
	linkOffset = d + d_fixed;
	initialize();
	worldTransformMatrix = getWorldTransformMatrix();
}

void DHframe::update_theta_fixed(double new_theta_fixed)
{
	theta_fixed = new_theta_fixed;
	updateJointAngle();
}

void DHframe::update_theta(double new_theta)
{
	theta = new_theta;
	updateJointAngle();
	/*std::cout << "Joint angle updated to " << theta << std::endl;*/

}

void DHframe::updateJointAngle()
{
	jointAngle = theta + theta_fixed;
	initialize();
	worldTransformMatrix = getWorldTransformMatrix();
	
}

//void DHframe::updateMovingJoint()
//{
//	movingJoint->updateJointVariable();
//}

void DHframe::printTransformMatrix()
{
	std::cout << transformMatrix << std::endl;
}
