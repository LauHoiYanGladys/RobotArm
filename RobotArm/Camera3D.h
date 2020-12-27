#pragma once


class Camera3D
{
public:
	static const double PI;
	double x, y, z;  // target (what the camera is looking at)
	double h, p, b;  // camera orientation (heading, pitch, bank)

	double fov, nearZ, farZ;

	Camera3D();
	void initialize(void);
	void setUpCameraProjection(void);
	void setUpCameraTransformation(void);

	void getForwardVector(double& vx, double& vy, double& vz);
	// returns unit vector components of camera orientation
};

