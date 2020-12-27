#pragma once
#include <algorithm>
#include "Camera3D.h"

class OrbitingViewer
{
public:
	double h, p;
	double dist;
	double focusX, focusY, focusZ;

	OrbitingViewer();
	void initialize(void);
	void setUpCamera(Camera3D& camera);
	void moveToSetPoint(double hSet, double pSet, double distSet, double angleRate, double distRate);
	void moveToHSetPoint(double hSet, double angleRate) {
		moveToSetPoint(hSet, p, dist, angleRate, 0);
	};
	void moveToPSetPoint(double pSet, double angleRate) {
		moveToSetPoint(h, pSet, dist, angleRate, 0);
	};
	void moveToDistSetPoint(double distSet, double distRate) {
		moveToSetPoint(h, p, distSet, 0, distRate);
	};
	void constantHOrbit(double hRate) {
		h += hRate;
		if (h >= 2 * Camera3D::PI) h -= 2 * Camera3D::PI;
		if (h <= -2 * Camera3D::PI) h += 2 * Camera3D::PI;
	};
};


