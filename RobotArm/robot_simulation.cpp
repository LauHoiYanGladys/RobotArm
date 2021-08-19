#include "ViewManager.h"

using Eigen::MatrixXd;

int main()
{
	ViewManager theManager;

	while (1) {
		theManager.manage();
	}
}