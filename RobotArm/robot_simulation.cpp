#include <iostream>
#include <Eigen/Dense>
#include "ViewManager.h"

using Eigen::MatrixXd;

int main()
{
	//MatrixXd m(2, 2);
	//m(0, 0) = 3;
	//m(1, 0) = 2.5;
	//m(0, 1) = -1;
	//m(1, 1) = m(1, 0) + m(0, 1);
	//std::cout << m << std::endl;

	ViewManager theManager;

	while (1) {
		theManager.manage();
	}
}