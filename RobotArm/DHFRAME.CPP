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

void DHframe::test_initialize()
{
	test_transformMatrix(0, 0) = cos(test_jointAngle);
	test_transformMatrix(0, 1) = -sin(test_jointAngle) * cos(linkTwist);
	test_transformMatrix(0, 2) = sin(test_jointAngle) * sin(linkTwist);
	test_transformMatrix(0, 3) = linkLength * cos(test_jointAngle);
	test_transformMatrix(1, 0) = sin(test_jointAngle);
	test_transformMatrix(1, 1) = cos(test_jointAngle) * cos(linkTwist);
	test_transformMatrix(1, 2) = -cos(test_jointAngle) * sin(linkTwist);
	test_transformMatrix(1, 3) = linkLength * sin(test_jointAngle);
	test_transformMatrix(2, 0) = 0;
	test_transformMatrix(2, 1) = sin(linkTwist);
	test_transformMatrix(2, 2) = cos(linkTwist);
	test_transformMatrix(2, 3) = test_linkOffset;
	test_transformMatrix(3, 0) = 0;
	test_transformMatrix(3, 1) = 0;
	test_transformMatrix(3, 2) = 0;
	test_transformMatrix(3, 3) = 1;
	/*std::cout << "Test transform matrix updated to:" << std::endl << test_transformMatrix << theta << std::endl;*/
}

Vector3d DHframe::getZAxis()
{
	Vector3d zAxis = transformMatrix.block<3, 1>(0, 2);
	return zAxis;
}

Vector3d DHframe::getZAxisWorldTest()
{
	Vector3d zAxisWorld = test_worldTransformMatrix.block<3, 1>(0, 2);
	return zAxisWorld;
}

void DHframe::assignParentDHframe(DHframe* theParent)
{
	parent = theParent;
	///*std::cout << "World transform matrix: " << std::endl << worldTransformMatrix << std::endl;*/
	updateWorldTransformMatrices(); // world and test_worldTransformMatrix can be assigned once parent DH frame is known
}

void DHframe::assignJoint(Joint* theJoint)
{
	joint = theJoint;
}

void DHframe::assignLink(Link* theLink, Link::linkDirection _direction)
{
	link = theLink;
	theLink->assignLinkDirection(_direction);
}

Vector3d DHframe::getWorldCenter(bool isTest)
{
	if (!isTest) {
		worldTransformMatrix = getWorldTransformMatrix(false); // to be 100% sure it's updated
		return getTranslation(worldTransformMatrix);
	}
	else {
		test_worldTransformMatrix = getWorldTransformMatrix(true); // to be 100% sure it's updated
		return getTranslation(test_worldTransformMatrix);
	}
}

Matrix3d DHframe::getWorldRotationMat(bool isTest)
{
	if (!isTest) {
		worldTransformMatrix = getWorldTransformMatrix(false); // to be 100% sure it's updated
		return getRotation(worldTransformMatrix);
	}
	else {
		test_worldTransformMatrix = getWorldTransformMatrix(true); // to be 100% sure it's updated
		return getRotation(test_worldTransformMatrix);
	}
}

Matrix4d DHframe::getWorldTransformMatrix(bool isTest)
{
	if (!isTest) {
		if (parent == nullptr) // base case
			return transformMatrix;
		else {
			/*std::cout << "parent transform matrix" << std::endl << parent->worldTransformMatrix << std::endl;*/
			return  (parent->getWorldTransformMatrix(isTest)) * transformMatrix;
		}
	}
	else {
		if (parent == nullptr) // base case
			return test_transformMatrix;
		else {
			/*std::cout << "parent test world transform matrix" << std::endl << parent->test_worldTransformMatrix << std::endl;*/
			return  (parent->getWorldTransformMatrix(isTest)) * test_transformMatrix;
			
		}
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

void DHframe::updateLinkLength(double newLinkLength)
{
	linkLength = newLinkLength;
	initialize();
	test_initialize();
	updateWorldTransformMatrices();
}

void DHframe::updateLinkTwist(double newLinkTwist)
{
	linkTwist = newLinkTwist;
	initialize();
	test_initialize();
	updateWorldTransformMatrices();
}

void DHframe::update_d_fixed(double new_d_fixed)
{
	d_fixed = new_d_fixed;
	updateLinkOffset(false);
	updateLinkOffset(true);
}

void DHframe::update_d(double new_d)
{
	d = new_d;
	updateLinkOffset(false);
}

void DHframe::update_test_d(double new_test_d)
{
	test_d = new_test_d;
	updateLinkOffset(true);

}

void DHframe::updateLinkOffset(bool isTest)
{
	if (!isTest) {
		linkOffset = d + d_fixed;
		initialize();
		worldTransformMatrix = getWorldTransformMatrix(false);
	}
	else {
		test_linkOffset = test_d + d_fixed;
		test_initialize();
		test_worldTransformMatrix = getWorldTransformMatrix(true);
	}
}

void DHframe::update_theta_fixed(double new_theta_fixed)
{
	theta_fixed = new_theta_fixed;
	// need to update both test_jointAngle and jointAngle
	updateJointAngle(false);
	updateJointAngle(true);
}

void DHframe::update_theta(double new_theta)
{
	theta = new_theta;
	updateJointAngle(false); // update jointAngle only, not test_jointAngle
	/*std::cout << "Joint angle updated to " << theta << std::endl;*/

}

void DHframe::update_test_theta(double new_test_theta)
{
	test_theta = new_test_theta;
	updateJointAngle(true);
}

void DHframe::updateJointAngle(bool isTest)
{
	if (!isTest) {
		jointAngle = theta + theta_fixed;
		initialize();
		worldTransformMatrix = getWorldTransformMatrix(false);
	}
	else {
		test_jointAngle = test_theta + theta_fixed;
		/*std::cout << "test_jointAngle updated to " << test_jointAngle << std::endl;*/
		test_initialize();
		test_worldTransformMatrix = getWorldTransformMatrix(true);
	}
	
}

void DHframe::updateWorldTransformMatrices()
{
	worldTransformMatrix = getWorldTransformMatrix(false);
	test_worldTransformMatrix = getWorldTransformMatrix(true);
	std::cout << "world transformation matrices initialized" << std::endl;
}


void DHframe::printTransformMatrix()
{
	std::cout << transformMatrix << std::endl;
}

void DHframe::drawLink()
{
	if (link != nullptr) {
		glColor3ub(255, 0, 0);	//red

		//if (joint != nullptr) {
		//	if (joint->type == Joint::revolute)
		//		link->draw_simple();
		//	else if (joint->type == Joint::prismatic)
		//		link->draw_offset_prismatic(joint->jointVariable);
		//}
		//else
		//	link->draw_simple();
		
		if (joint != nullptr && joint->type == Joint::prismatic)
			link->draw(joint->jointVariable);
		else
			link->draw(0);

	}
}

void DHframe::drawJoint()
{
	if (joint != nullptr) {
		//drawing of the joint
		glColor3ub(0, 0, 255);	//blue
		if (joint->type == Joint::revolute)
			DrawingUtilNG::drawCylinderZ(2., 2., 10., 0, 0, 0);
		else if (joint->type == Joint::prismatic) {
			// prismatic joint itself without extension
			DrawingUtilNG::drawPrismZ(4., 4., 10., 0, 0, 0);
			glColor3ub(0, 191, 255);	//deep sky blue
			// extension of prismatic joint ("center" shifted half the joint's original length in the z-direction)
			DrawingUtilNG::drawPrismZOffset(4., 4., joint->jointVariable, 0, 0, 5.);
		}
	}
}
