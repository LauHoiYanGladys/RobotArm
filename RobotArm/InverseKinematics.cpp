#include "InverseKinematics.h"

const double InverseKinematics::PI = 3.1415927;
InverseKinematics::InverseKinematics(double x, double y, double z, Arm* inputArm)
{
	xTarget = x;
	yTarget = y;
	zTarget = z;
	goal << x, y, z;
	theArm = inputArm;
	testJointVariables = theArm->getTestJointVariable();
	jacobian << 0, 0, 0,
				0, 0, 0,
				0, 0, 0;

	alpha = inputArm->alpha; // revolute joint learning rate
	alphaPris = inputArm->alphaPris; // prismatic joint need larger learning rate because its values are inherently larger than those of revolute
	costChangeStopThreshold = inputArm->costChangeStopThreshold;
	maxIteration = 700;
	stopThreshold = 1;
	delta = 0.001;
	workspaceThreshold = inputArm->workspaceThreshold;
}

void InverseKinematics::computeCostGradient()
{
	std::vector<double>testJointVariables_regularVec = vector3dToRegularVector(testJointVariables);
	theArm->updateTestFrames(testJointVariables_regularVec);

	// take partial derivative of the cost function
	costGradient = differentiateCost();

}

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

		// early terminate while loop if new cost is low or amount of update is small
		if (costChangeStopThreshold != NULL) {
			if (theNewCost < stopThreshold || abs(costChange) < costChangeStopThreshold) {
				constrainPrismatic(); // keep prismatic joint variable >= 0 
				costAfterConstraint = computeCost(costType::currCost);
				return isInsideWorkspace(costAfterConstraint);

			}
		}
		else {
			if (theNewCost < stopThreshold) {
				constrainPrismatic(); // keep prismatic joint variable >= 0 
				costAfterConstraint = computeCost(costType::currCost);
				return isInsideWorkspace(costAfterConstraint);
			}
		}

		testJointVariables = new_testJointVariables;
		theCurrCost = theNewCost;
		counter++;

	} while (counter < maxIteration);

	constrainPrismatic(); // keep prismatic joint variable >= 0 
	costAfterConstraint = computeCost(costType::currCost);
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

	// joint variable becomes nan when the goal is almost directly above the base joint (constraining the numerator and denominator to mitigate)
	testJointVariables(1) = std::asin(std::min((goal(2) - firstLinkLength),20.) / std::max(std::sqrt(pow(goal(0), 2) + pow(goal(1), 2)),0.1));

	testJointVariables(2) = std::max((std::sqrt(pow(goal(0), 2) + pow(goal(1), 2)) - secondLinkLength - thirdLinkLength),0.); // min extension is zero
}

double InverseKinematics::computeCost(costType theCostType)
{
	double angleDeviation, distance;
	double cost = 0.;
	if (theCostType == costType::currCost) {
		std::vector<double>testJointVariables_regularVec = vector3dToRegularVector(testJointVariables);
		theArm->updateTestFrames(testJointVariables_regularVec);
	}
	else if (theCostType == costType::newCost) {
		std::vector<double>new_testJointVariables_regularVec = vector3dToRegularVector(new_testJointVariables);
		theArm->updateTestFrames(new_testJointVariables_regularVec);
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

	angleDeviation = acos((secondLinkDir.dot(goalDir)) / (secondLinkDir.norm() * goalDir.norm())); // acos returns values in the interval [0,pi] radians

	return angleDeviation;
}

void InverseKinematics::getResult(Vector3d& result)
{
	result = testJointVariables;
}

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
