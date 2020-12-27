#pragma once
class Link
{
public:
	double x, y, z; //x,y,z coordinates of origin
	double rx, ry, rz; //rotations about x,y,z

	//draws a simple version of the link, at the origin & rotation
	void draw_simple();
};

