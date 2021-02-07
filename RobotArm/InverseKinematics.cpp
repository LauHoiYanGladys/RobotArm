#include "InverseKinematics.h"
#include<iostream>
#include <cmath> 
#include <algorithm>    // std::max

const double InverseKinematics::PI = 3.1415927;
InverseKinematics::InverseKinematics(double x, double y, double z, Arm* inputArm)
{
	xTarget = x;
	yTarget = y;
	zTarget = z;
	goal << x, y, z;
	theArm = inputArm;
	testJointVariables = theArm->getTestJointVariable();
	/*testJointVariables << PI/4, PI/4, 10;*/
	/*testJointVariables = theArm->getJointVariable();*/
	jacobian << 0, 0, 0,
				0, 0, 0,
				0, 0, 0;
	/*jacobian << 0, 0, 0;*/ // (single-revolute joint arm(for debugging))

	alpha = inputArm->alpha; // revolute joint learning rate
	alphaPris = inputArm->alphaPris; // prismatic joint need larger learning rate because its values are inherently larger than those of revolute
	costChangeStopThreshold = inputArm->costChangeStopThreshold;
	maxIteration = 700;
	stopThreshold = 1;
	delta = 0.001;
	workspaceThreshold = inputArm->workspaceThreshold;
}

void/*Vector3d*//*double*/ InverseKinematics::computeCostGradient()
{
	std::vector<double>testJointVariables_regularVec = vector3dToRegularVector(testJointVariables);
	theArm->updateTestFrames(testJointVariables_regularVec);

	// take partial derivative of the cost function
	costGradient = differentiateCost();

}

//double InverseKinematics::differentiate(int jointVariableIndex)
//{
//	double res;
//	/*Vector3d res;*/
//	double cost1 = 0.;
//	double cost2 = 0.;
//	/*Vector3d cost1;
//	Vector3d cost2;*/
//	Vector3d FK_plus;
//	Vector3d FK_minus;
//	if (jointVariableIndex == 0) {
//
//		theArm->updateTestFrames(testJointVariables(0)+delta, testJointVariables(1), testJointVariables(2));
//		FK_plus = theArm->compute_test_FK(4);
//		theArm->updateTestFrames(testJointVariables(0)-delta, testJointVariables(1), testJointVariables(2));
//		FK_minus = theArm->compute_test_FK(4);
//
//		cost1 = 0.5 * (FK_plus - goal).dot(FK_plus - goal);
//		cost2 = 0.5 * (FK_minus - goal).dot(FK_minus - goal);
//		/*cost1 = 0.5 * ((FK_plus - goal).transpose()) * ((FK_plus - goal));
//		cost2 = 0.5 * ((FK_minus - goal).transpose()) * ((FK_minus - goal));*/
//		/*cost1 = pow(FK_plus(0) - goal(0), 2) + pow(FK_plus(1) - goal(1), 2) + pow(FK_plus(2) - goal(2), 2);
//		cost2 = pow(FK_minus(0) - goal(0), 2) + pow(FK_minus(1) - goal(1), 2) + pow(FK_minus(2) - goal(2), 2);*/
//
//		res = (cost1 - cost2)/ (2 * delta);
//	}
//	else if (jointVariableIndex == 1) {
//
//		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1) + delta, testJointVariables(2));
//		FK_plus = theArm->compute_test_FK(4);
//		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1) - delta, testJointVariables(2));
//		FK_minus = theArm->compute_test_FK(4);
//
//		cost1 = 0.5 * (FK_plus - goal).dot(FK_plus - goal);
//		cost2 = 0.5 * (FK_minus - goal).dot(FK_minus - goal);
//		/*cost1 = 0.5 * ((FK_plus - goal).transpose()) * ((FK_plus - goal));
//		cost2 = 0.5 * ((FK_minus - goal).transpose()) * ((FK_minus - goal));*/
//		/*cost1 = pow(FK_plus(0) - goal(0), 2) + pow(FK_plus(1) - goal(1), 2) + pow(FK_plus(2) - goal(2), 2);
//		cost2 = pow(FK_minus(0) - goal(0), 2) + pow(FK_minus(1) - goal(1), 2) + pow(FK_minus(2) - goal(2), 2);*/
//
//		res = (cost1 - cost2)/ (2 * delta);
//	}
//	else if (jointVariableIndex == 2) {
//
//		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2) + delta);
//		FK_plus = theArm->compute_test_FK(4);
//		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2) - delta);
//		FK_minus = theArm->compute_test_FK(4);
//
//		cost1 = 0.5 * (FK_plus - goal).dot(FK_plus - goal);
//		cost2 = 0.5 * (FK_minus - goal).dot(FK_minus - goal);
//		/*cost1 = pow(FK_plus(0) - goal(0), 2) + pow(FK_plus(1) - goal(1), 2) + pow(FK_plus(2) - goal(2), 2);
//		cost2 = pow(FK_minus(0) - goal(0), 2) + pow(FK_minus(1) - goal(1), 2) + pow(FK_minus(2) - goal(2), 2);*/
//		/*cost1 = 0.5 * ((FK_plus - goal).transpose()) * ((FK_plus - goal));
//		cost2 = 0.5 * ((FK_minus - goal).transpose()) * ((FK_minus - goal));*/
//		res = (cost1 - cost2) / (2 * delta);
//	}
//	else {
//		std::cout << "invalid jointNumber, differentiation not successful" << std::endl;
//	}
//	return res;
//}

void InverseKinematics::update()
{
	computeCostGradient();
	Vector3d update;

	// multiply each element of gradient with the suitable alpha to get the update
	int jointNumber = theArm->theJoints.size(); // get the number of joints
	for (int i = 0; i < jointNumber; i++) {
		if (theArm->theJoints[i]->type == Joint::revolute)
			update(i) = costGradient[i] * alpha;
		else if (theArm->theJoints[i]->type == Joint::prismatic)
			update(i) = costGradient[i] * alphaPris;
	}
	new_testJointVariables = testJointVariables - update;

	/*std::cout << "test joint variable " <<  jointVariableIndex << " updated from " 
		<< testJointVariables << " to " << new_testJointVariables << std::endl;*/

	/*std::cout << "Update done" << std::endl;*/
}

bool InverseKinematics::getIK()
{
	int counter = 1;
	double updateAmount;
	double costAfterConstraint;
 	theCurrCost = computeCost(costType::currCost);

	costChange = -1.;
	do {
		// computes new joint variables
		update();
		theNewCost = computeCost(costType::newCost);
		costChange = theNewCost - theCurrCost;

		
		//std::cout << "original test joint variables are " << testJointVariables << std::endl;
		//std::cout << "new test joint variables are " << new_testJointVariables << std::endl;
		/*std::cout << "Iteration " << counter << std::endl;*/
		/*std::cout << "costChange " << costChange << std::endl;*/


		// early terminate while loop if new cost is low or amount of update is small
		if (costChangeStopThreshold != NULL) {
			if (theNewCost < stopThreshold || abs(costChange) < costChangeStopThreshold) {
				std::cout << "Gradient descend early ended on iteration " << counter << std::endl;
				std::cout << "Ending cost after IK is " << theCurrCost << std::endl;
				constrainPrismatic(); // keep prismatic joint variable >= 0 
				std::cout << "Final joint variables are " << std::endl;
				std::cout << testJointVariables << std::endl;
				costAfterConstraint = computeCost(costType::currCost);
				std::cout << "Ending cost after constraint is " << costAfterConstraint << std::endl;
				return isInsideWorkspace(costAfterConstraint);

			}
		}
		else {
			if (theNewCost < stopThreshold) {
				std::cout << "Gradient descend early ended on iteration " << counter << std::endl;
				std::cout << "Ending cost after IK is " << theCurrCost << std::endl;
				constrainPrismatic(); // keep prismatic joint variable >= 0 
				std::cout << "Final joint variables are " << std::endl;
				std::cout << testJointVariables << std::endl;
				costAfterConstraint = computeCost(costType::currCost);
				std::cout << "Ending new cost after constraint is " << costAfterConstraint << std::endl;
				return isInsideWorkspace(costAfterConstraint);
			}
		}

		testJointVariables = new_testJointVariables;
		theCurrCost = theNewCost;
		counter++;

	} while (counter < maxIteration);
	

	//testJointVariables(2) = getPrismaticJointVar(); // correct prismatic joint variable computed, independent of the gradient descent

	constrainPrismatic(); // keep prismatic joint variable >= 0 
	std::cout << "Gradient descend ended after max iteration " << counter << std::endl;
	std::cout << "Ending cost is " << theCurrCost << std::endl;
	std::cout << "Final joint variables are " << testJointVariables;
	costAfterConstraint = computeCost(costType::currCost);
	std::cout << "Ending new cost after constraint is " << costAfterConstraint << std::endl;
	return isInsideWorkspace(costAfterConstraint);
}

bool InverseKinematics::isInsideWorkspace(double theCost)
{
	if (theCost > workspaceThreshold)
		return false;
	else
		return true;
}

void InverseKinematics::getIKAnalytical()
{
	double firstLinkLength = theArm->theFrames[0]->link->length;
	double secondLinkLength = theArm->theFrames[2]->link->length;
	double thirdLinkLength = theArm->theFrames[3]->link->length;

	testJointVariables(0) = std::atan2(goal(1),goal(0)); 
	std::cout << "first joint variable is " << testJointVariables(0) << std::endl;

	// joint variable becomes nan when the goal is almost directly above the base joint (constraining the numerator and denominator to mitigate)
	testJointVariables(1) = std::asin(std::min((goal(2) - firstLinkLength),20.) / std::max(std::sqrt(pow(goal(0), 2) + pow(goal(1), 2)),0.1));
	std::cout << "second joint variable is " << testJointVariables(1) << std::endl;

	testJointVariables(2) = std::max((std::sqrt(pow(goal(0), 2) + pow(goal(1), 2)) - secondLinkLength - thirdLinkLength),0.); // min extension is zero
	std::cout << "third joint variable is " << testJointVariables(2) << std::endl;
}

double InverseKinematics::computeCost(costType theCostType)
{
	double angleDeviation, distance;
	double cost = 0.;
	if (theCostType == costType::currCost) {
		std::vector<double>testJointVariables_regularVec = vector3dToRegularVector(testJointVariables);
		theArm->updateTestFrames(testJointVariables_regularVec);
		/*theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));*/
	}
	else if (theCostType == costType::newCost) {
		std::vector<double>new_testJointVariables_regularVec = vector3dToRegularVector(new_testJointVariables);
		theArm->updateTestFrames(new_testJointVariables_regularVec);
		/*theArm->updateTestFrames(new_testJointVariables(0), new_testJointVariables(1), new_testJointVariables(2));*/
	}

	cost = getDistance();
	// restore the original test joint variable (just to be extra safe)
	std::vector<double>testJointVariables_regularVec = vector3dToRegularVector(testJointVariables);
	theArm->updateTestFrames(testJointVariables_regularVec);
	return cost;
}


double InverseKinematics::computeCost(std::vector<double> jointVariables)
{
	double cost = 0.;
	std::vector<double> testJointVariables_regularVec;
	theArm->updateTestFrames(jointVariables);
	cost = getDistance();

	// restore the original test joint variable (just to be extra safe)
	testJointVariables_regularVec = vector3dToRegularVector(testJointVariables);
	theArm->updateTestFrames(testJointVariables_regularVec);

	return cost;
}

//double InverseKinematics::differentiateCost(/*costType theCostType, */int jointVariableNum)
//{
//	double cost_plus = 0.;
//	double cost_minus = 0.;
//	double res;
//
//	if (jointVariableNum == 1) {
//		// first joint var, plus
//		cost_plus = computeCost(testJointVariables(0) + delta, testJointVariables(1), testJointVariables(2));
//
//		// first joint var, minus
//		cost_minus = computeCost(testJointVariables(0) - delta, testJointVariables(1), testJointVariables(2));
//
//	}
//	else if (jointVariableNum == 2) {
//		// second joint var, plus
//		cost_plus = computeCost(testJointVariables(0), testJointVariables(1) + delta, testJointVariables(2));
//
//		// second joint var, minus
//		cost_minus = computeCost(testJointVariables(0), testJointVariables(1) - delta, testJointVariables(2));
//
//	}
//	else if (jointVariableNum == 3) {
//		// third joint var, plus
//		cost_plus = computeCost(testJointVariables(0), testJointVariables(1), testJointVariables(2) + delta);
//
//		// third joint var, minus
//		cost_minus = computeCost(testJointVariables(0), testJointVariables(1), testJointVariables(2) - delta);
//	}
//	else
//		std::cout << "invalid jointVariableNum, differentiation failed" << std::endl;
//		
//	res = (cost_plus - cost_minus) / (2 * delta); // compute result
//
//	// taken care of by the computeCost function
//	//theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2)); // restore values
//
//	return res;
//}

std::vector<double> InverseKinematics::differentiateCost()
{
	std::vector<double>res;
	int numJointVariable = theArm->frameWithJointVariable.size(); // get number of joint variables

	// differentiate the cost function for each joint variable
	for (int i = 0; i < numJointVariable; i++) {
		double cost_plus = 0.;
		double cost_minus = 0.;
		Vector3d testJointVariables_plus = testJointVariables;
		Vector3d testJointVariables_minus = testJointVariables;

		// perturb the joint variable upwards (plus)
		testJointVariables_plus(i) = testJointVariables(i) + delta;
		std::vector<double> testJointVariables_plus_regularVec = vector3dToRegularVector(testJointVariables_plus);
		cost_plus = computeCost(testJointVariables_plus_regularVec);

		// perturb the joint variable downwards (minus)
		testJointVariables_minus(i) = testJointVariables(i) - delta;
		std::vector<double> testJointVariables_minus_regularVec = vector3dToRegularVector(testJointVariables_minus);
		cost_minus = computeCost(testJointVariables_minus_regularVec);

		double result = (cost_plus - cost_minus) / (2 * delta); // compute result
		res.push_back(result);
	}

	return res;
}

double InverseKinematics::computeAngleDeviation()
{
	double angleDeviation;
	Vector3d FK_second;
	Vector3d secondLinkDir; // vector along the second link
	Vector3d goalDir; // vector between the second joint and goal

	FK_second = theArm->compute_test_FK(2); // compute the original FK to the SECOND DH frame (i.e. frame at the second joint)

	// penalize deviation from zero in the angle between the second link and the vector between the second joint and goal
	secondLinkDir = theArm->theFrames[2]->getZAxisWorldTest();
	goalDir = goal - FK_second;

	/*angleDeviation = atan2(goalDir.cross(secondLinkDir).norm(), goalDir.dot(secondLinkDir));*/
	angleDeviation = acos((secondLinkDir.dot(goalDir)) / (secondLinkDir.norm() * goalDir.norm())); // acos returns values in the interval [0,pi] radians

	/*std::cout << "secondLinkDir is " << secondLinkDir << std::endl;
	std::cout << "goalDir is " << goalDir << std::endl;
	std::cout << "angle deviation is " << angleDeviation << std::endl;*/

	return angleDeviation;
}

//void InverseKinematics::computeJacobian()
//{
//	// get the number of joints
//	int jointNumber = theArm->theJoints.size();
//	std::vector<Vector3d> jacobianCols;
//	for (int i = 0; i < jointNumber; i++) {
//		Vector3d temp;
//		if (theArm->theJoints[i]->type == Joint::revolute)
//			temp = computeJacobianColRev(i);
//		else if (theArm->theJoints[i]->type == Joint::prismatic)
//			temp = computeJacobianColPris(i);
//		jacobianCols.push_back(temp);
//	}
//	/*jacobian = jacobianCols[0];*/ //one-revolute arm (debugging)
//	jacobian << jacobianCols[0], jacobianCols[1], jacobianCols[2];
//	/*std::cout << "jacobian is " << jacobian << std::endl;*/
//}

//Vector3d InverseKinematics::computeJacobianCol(int jointVariableIndex)
//{
//	Vector3d jacobianCol;
//	Vector3d FK;
//	Vector3d perturbedFK_plus;
//	Vector3d perturbedFK_minus;
//	int frameIndex;
//	/*DHframe* theFrame;*/
//	// get the DH frame corresponding to the joint variable
//	if (jointVariableIndex == 0)
//		frameIndex = 1;
//	/*theFrame = theArm->theFrames[1];*/
//	else if (jointVariableIndex == 1)
//		frameIndex = 2;
//	/*theFrame = theArm->theFrames[2];*/
//	else if (jointVariableIndex == 2)
//		frameIndex = 4;
//		/*theFrame = theArm->theFrames[4];*/
//	else {
//		/*theFrame = nullptr;*/
//		std::cout << "Invalid joint variable index" << std::endl;
//	}
//		
//	/*DHframe* theFrame = theArm->jointFrameMap.at(theArm->theJoints[jointIndex]);*/
//	
//	// compute the jacobian column
//	// first put the current test joint variables into the arm's frames
//
//	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));
//
//	// second, compute the original FK
//	/*FK = theArm->compute_test_FK(theFrame);*/
//
//	// third, perturb (slightly increase) the relevant test joint variable 
//	if (jointVariableIndex == 0)
//
//		theArm->updateTestFrames(testJointVariables(0)+delta, testJointVariables(1), testJointVariables(2));
//	else if (jointVariableIndex == 1)
//
//		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1)+delta, testJointVariables(2));
//	else if (jointVariableIndex == 2)
//
//		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2) + delta);
//	else {
//		/*theFrame = nullptr;*/
//		std::cout << "Invalid joint variable index" << std::endl;
//	}
//	
//	// fourth, compute the perturbed FK with the perturbed joint variable
//	perturbedFK_plus = theArm->compute_test_FK(frameIndex);
//
//	// restore the original value of the joint variables for correct perturbation
//	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));
//
//	// fifth, perturb (slightly decrease) the relevant test joint variable 
//	if (jointVariableIndex == 0)
//		theArm->updateTestFrames(testJointVariables(0) - delta, testJointVariables(1), testJointVariables(2));
//	else if (jointVariableIndex == 1)
//		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1) - delta, testJointVariables(2));
//	else if (jointVariableIndex == 2)
//		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2) - delta);
//	else {
//		std::cout << "Invalid joint variable index" << std::endl;
//	}
//
//	// sixth, compute the perturbed FK with the perturbed joint variable
//	perturbedFK_minus = theArm->compute_test_FK(frameIndex);
//
//	// finally, get the jacobian column
//	jacobianCol = (perturbedFK_plus - perturbedFK_minus) / (2*delta);
//
//	// restore the original test joint variable 
//	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));
//
//	return jacobianCol;
//}

//Vector3d InverseKinematics::computeJacobianColRev(int jointVariableIndex)
//{
//	DHframe* theFrame;
//	Vector3d jacobianCol;
//	Vector3d FK;
//	Vector3d currFK;
//	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));
//	if (jointVariableIndex == 0) {
//		theFrame = theArm->theFrames[1];
//		currFK = theArm->compute_test_FK(0);
//	}
//	else if (jointVariableIndex == 1) {
//		theFrame = theArm->theFrames[2];
//		currFK = theArm->compute_test_FK(1);
//	}
//	else if (jointVariableIndex == 2) {
//		theFrame = theArm->theFrames[4];
//		currFK = theArm->compute_test_FK(3);
//	}
//	else {
//		theFrame = nullptr;
//		std::cout << "Invalid joint variable index" << std::endl;
//	}
//	
//	FK = theArm->compute_test_FK(4);
//	/*FK = theArm->compute_test_FK(2);*/ // single-revolute joint arm (for debugging)
//	
//	/*std::cout << "The parent frame z-axis is " << theFrame->parent->getZAxisWorldTest();
//	std::cout << "the vector from frame origin to end-effector is" << FK - currFK;
//	std::cout << "the cross product is" << (theFrame->parent->getZAxisWorldTest()).cross(FK - currFK);*/
//
//	jacobianCol = (theFrame->parent->getZAxisWorldTest()).cross(FK - currFK);
//	return jacobianCol;
//}

//Vector3d InverseKinematics::computeJacobianColPris(int jointVariableIndex)
//{
//	DHframe* theFrame;
//	Vector3d jacobianCol;
//	if (jointVariableIndex == 0)
//		theFrame = theArm->theFrames[1];
//	else if (jointVariableIndex == 1)
//		theFrame = theArm->theFrames[2];
//	else if (jointVariableIndex == 2)
//		theFrame = theArm->theFrames[4];
//	else {
//		theFrame = nullptr;
//		std::cout << "Invalid joint variable index" << std::endl;
//	}
//	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2)); // make sure it's updated
//	jacobianCol = theFrame->parent->getZAxisWorldTest();
//	return jacobianCol;
//}

void InverseKinematics::getResult(Vector3d& result)
{
	result = testJointVariables;
}

//double InverseKinematics::getPrismaticJointVar()
//{
//	double res;
//	double thirdLinkLength;
//	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));
//	thirdLinkLength = theArm->theFrames[3]->link->length;
//	Vector3d FK_thirdFrame = theArm->compute_test_FK(3); // compute the original FK
//	res = std::max((goal - FK_thirdFrame).norm() - thirdLinkLength,0.); // constrain prismatic joint length to be >= 0
//
//	return res;
//}

double InverseKinematics::getDistance()
{
	int endEffectorFrameIndex = theArm->theFrames.size()-1;
	Vector3d FK_endEffector = theArm->compute_test_FK(endEffectorFrameIndex); // compute the original FK
	double res = (goal - FK_endEffector).norm();
	return res;
}

std::vector<double> InverseKinematics::vector3dToRegularVector(Vector3d theVector)
{
	std::vector<double> res{ theVector(0), theVector(1), theVector(2) };
	return res;
}

void InverseKinematics::constrainPrismatic()
{
	int jointNumber = theArm->theJoints.size();
	// keep prismatic joint variable >=0
	for (int i = 0; i < jointNumber; i++) {
		if (theArm->theJoints[i]->type == Joint::prismatic)
			testJointVariables(i) = std::max(testJointVariables(i), 0.);
	}
}
