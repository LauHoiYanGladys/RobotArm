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

	alpha = 0.00001; // revolute joint learning rate
	alphaPris = 0.1; // prismatic joint need larger learning rate because its values are inherently larger than those of revolute

	maxIteration = 300;
	stopThreshold = 0.1;
	delta = 0.001;
}

void/*Vector3d*//*double*/ InverseKinematics::computeCostGradient()
{
	
	Vector3d FK;
	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));
	//FK = theArm->compute_test_FK(2); // end effector frame is 2nd frame (single-revolute joint arm (for debugging))
	FK = theArm->compute_test_FK(4); // end effector frame is 4th frame
	computeJacobian();
	/*costGradient = jacobian.dot(FK - goal);*/ //single-revolute joint arm (for debugging)
	costGradient = (jacobian.transpose()) * (FK - goal);
	/*std::cout << "cost gradient is " << costGradient << std::endl;*/

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
			update(i) = costGradient(i) * alpha;
		else if (theArm->theJoints[i]->type == Joint::prismatic)
			update(i) = costGradient(i) * alphaPris;
	}
	new_testJointVariables = testJointVariables - update;

	// make sure it is within joint limits
	/*new_testJointVariables = std::min(std::max(new_testJointVariables, -PI), PI);*/
	/*new_testJointVariables(0) = std::min(std::max(new_testJointVariables(0), - PI), PI);
	new_testJointVariables(1) = std::min(std::max(new_testJointVariables(1), - PI), PI);
	new_testJointVariables(2) = std::min(std::max(new_testJointVariables(2), 0.), 200.);*/

	/*std::cout << "test joint variable " <<  jointVariableIndex << " updated from " 
		<< testJointVariables << " to " << new_testJointVariables << std::endl;*/

	std::cout << "Update done" << std::endl;
}

void InverseKinematics::getIK()
{
	int counter = 1;
	double updateAmount;
 	theCurrCost = computeCost(costType::currCost);

	costChange = -1.;
	do {
		// computes new joint variables
		update();
		theNewCost = computeCost(costType::newCost);
		costChange = theNewCost - theCurrCost;

		counter++;

		std::cout << "original test joint variables are " << testJointVariables << std::endl;
		std::cout << "new test joint variables are " << new_testJointVariables << std::endl;

		
		updateAmount = costGradient.norm();
		// early terminate while loop if new cost is low or amount of update is small
		if (theNewCost < 1 || updateAmount < 10) {
			std::cout << "Gradient descend early ended on iteration " << counter << std::endl;
			std::cout << "Ending new cost is " << theNewCost << std::endl;
			std::cout << "Ending update amount is " << updateAmount << std::endl;
			return;

		}
		
		testJointVariables = new_testJointVariables;
		theCurrCost = theNewCost;

	} while (counter < maxIteration);
	
	std::cout << "Gradient descend ended after max iteration " << counter << std::endl;
	std::cout << "Ending new cost is " << theNewCost << std::endl;
	std::cout << "IK computation completed" << std::endl;
	
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
	Vector3d FK;
	
	double cost = 0.;
	if (theCostType == costType::currCost) {
		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));
		FK = theArm->compute_test_FK(4); // compute the original FK
		//FK = theArm->compute_test_FK(2); // compute the original FK of one-revolute arm (debugging)
		cost = 0.5 * (FK - goal).dot(FK - goal);

		// wrong cost
		/*cost = pow(goal(0) - FK(0), 2) + pow(goal(1) - FK(1), 2) + pow(goal(2) - FK(2), 2);*/
	}
	else if (theCostType == costType::newCost) {
	
		theArm->updateTestFrames(new_testJointVariables(0), new_testJointVariables(1), new_testJointVariables(2));
		FK = theArm->compute_test_FK(4); // compute the new FK
		//FK = theArm->compute_test_FK(2); // compute the original FK of one-revolute arm (debugging)

		cost = 0.5 * (FK - goal).dot(FK - goal);

		// wrong cost
		/*cost = pow(goal(0) - FK(0), 2) + pow(goal(1) - FK(1), 2) + pow(goal(2) - FK(2), 2);*/

		// restore the original test joint variable (just to be extra safe)
		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));

	}
	return cost;
}

void InverseKinematics::computeJacobian()
{
	// get the number of joints
	int jointNumber = theArm->theJoints.size();
	std::vector<Vector3d> jacobianCols;
	for (int i = 0; i < jointNumber; i++) {
		Vector3d temp;
		if (theArm->theJoints[i]->type == Joint::revolute)
			temp = computeJacobianColRev(i);
		else if (theArm->theJoints[i]->type == Joint::prismatic)
			temp = computeJacobianColPris(i);
		jacobianCols.push_back(temp);
	}
	/*jacobian = jacobianCols[0];*/ //one-revolute arm (debugging)
	jacobian << jacobianCols[0], jacobianCols[1], jacobianCols[2];
	std::cout << "jacobian is " << jacobian << std::endl;
}

Vector3d InverseKinematics::computeJacobianCol(int jointVariableIndex)
{
	Vector3d jacobianCol;
	Vector3d FK;
	Vector3d perturbedFK_plus;
	Vector3d perturbedFK_minus;
	int frameIndex;
	/*DHframe* theFrame;*/
	// get the DH frame corresponding to the joint variable
	if (jointVariableIndex == 0)
		frameIndex = 1;
	/*theFrame = theArm->theFrames[1];*/
	else if (jointVariableIndex == 1)
		frameIndex = 2;
	/*theFrame = theArm->theFrames[2];*/
	else if (jointVariableIndex == 2)
		frameIndex = 4;
		/*theFrame = theArm->theFrames[4];*/
	else {
		/*theFrame = nullptr;*/
		std::cout << "Invalid joint variable index" << std::endl;
	}
		
	/*DHframe* theFrame = theArm->jointFrameMap.at(theArm->theJoints[jointIndex]);*/
	
	// compute the jacobian column
	// first put the current test joint variables into the arm's frames

	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));

	// second, compute the original FK
	/*FK = theArm->compute_test_FK(theFrame);*/

	// third, perturb (slightly increase) the relevant test joint variable 
	if (jointVariableIndex == 0)

		theArm->updateTestFrames(testJointVariables(0)+delta, testJointVariables(1), testJointVariables(2));
	else if (jointVariableIndex == 1)

		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1)+delta, testJointVariables(2));
	else if (jointVariableIndex == 2)

		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2) + delta);
	else {
		/*theFrame = nullptr;*/
		std::cout << "Invalid joint variable index" << std::endl;
	}
	
	// fourth, compute the perturbed FK with the perturbed joint variable
	perturbedFK_plus = theArm->compute_test_FK(frameIndex);

	// restore the original value of the joint variables for correct perturbation
	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));

	// fifth, perturb (slightly decrease) the relevant test joint variable 
	if (jointVariableIndex == 0)
		theArm->updateTestFrames(testJointVariables(0) - delta, testJointVariables(1), testJointVariables(2));
	else if (jointVariableIndex == 1)
		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1) - delta, testJointVariables(2));
	else if (jointVariableIndex == 2)
		theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2) - delta);
	else {
		std::cout << "Invalid joint variable index" << std::endl;
	}

	// sixth, compute the perturbed FK with the perturbed joint variable
	perturbedFK_minus = theArm->compute_test_FK(frameIndex);

	// finally, get the jacobian column
	jacobianCol = (perturbedFK_plus - perturbedFK_minus) / (2*delta);

	// restore the original test joint variable 
	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));

	return jacobianCol;
}

Vector3d InverseKinematics::computeJacobianColRev(int jointVariableIndex)
{
	DHframe* theFrame;
	Vector3d jacobianCol;
	Vector3d FK;
	Vector3d currFK;
	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2));
	if (jointVariableIndex == 0) {
		theFrame = theArm->theFrames[1];
		currFK = theArm->compute_test_FK(0);
	}
	else if (jointVariableIndex == 1) {
		theFrame = theArm->theFrames[2];
		currFK = theArm->compute_test_FK(1);
	}
	else if (jointVariableIndex == 2) {
		theFrame = theArm->theFrames[4];
		currFK = theArm->compute_test_FK(3);
	}
	else {
		theFrame = nullptr;
		std::cout << "Invalid joint variable index" << std::endl;
	}
	
	FK = theArm->compute_test_FK(4);
	/*FK = theArm->compute_test_FK(2);*/ // single-revolute joint arm (for debugging)
	
	std::cout << "The parent frame z-axis is " << theFrame->parent->getZAxisWorldTest();
	std::cout << "the vector from frame origin to end-effector is" << FK - currFK;
	std::cout << "the cross product is" << (theFrame->parent->getZAxisWorldTest()).cross(FK - currFK);
	jacobianCol = (theFrame->parent->getZAxisWorldTest()).cross(FK - currFK);
	return jacobianCol;
}

Vector3d InverseKinematics::computeJacobianColPris(int jointVariableIndex)
{
	DHframe* theFrame;
	Vector3d jacobianCol;
	if (jointVariableIndex == 0)
		theFrame = theArm->theFrames[1];
	else if (jointVariableIndex == 1)
		theFrame = theArm->theFrames[2];
	else if (jointVariableIndex == 2)
		theFrame = theArm->theFrames[4];
	else {
		theFrame = nullptr;
		std::cout << "Invalid joint variable index" << std::endl;
	}
	theArm->updateTestFrames(testJointVariables(0), testJointVariables(1), testJointVariables(2)); // make sure it's updated
	jacobianCol = theFrame->parent->getZAxisWorldTest();
	return jacobianCol;
}

void InverseKinematics::getResult(Vector3d& result)
{
	result = testJointVariables;
}
