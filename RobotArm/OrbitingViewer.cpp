#include "fssimplewindow.h"
#include "OrbitingViewer.h"

using namespace std;

OrbitingViewer::OrbitingViewer()
{
	initialize();
}

void OrbitingViewer::initialize(void)
{
	h = 0;
	p = 0;
	dist = 120.0;
	focusX = 0.0;
	focusY = 15.0;
	focusZ = 0.0;
}

void OrbitingViewer::setUpCamera(Camera3D& camera)
{
	camera.h = h;
	camera.p = p;
	camera.b = 0.0;

	double vx, vy, vz;
	camera.getForwardVector(vx, vy, vz);
	camera.x = focusX - vx * dist;
	camera.y = focusY - vy * dist;
	camera.z = focusZ - vz * dist;
}

//moves orbiting viewer towards set point from current position, by value at most angleRate or distRate
void OrbitingViewer::moveToSetPoint(double hSet, double pSet, double distSet, double angleRate, double distRate) {
	
	//take care of h
	if (h > hSet) {
		h -= min(angleRate, (h - hSet));
	}
	if (h < hSet) {
		h += min(angleRate, (hSet - h));
	}

	//take care of p
	if (p > pSet) {
		p -= min(angleRate, (p - pSet));
	}
	if (p < pSet) {
		p += min(angleRate, (pSet - p));
	}

	//take care of dist
	if (dist > distSet) {
		dist -= min(distRate, (dist - distSet));
	}
	if (dist < distSet) {
		dist += min(distRate, (distSet - dist));
	}

}