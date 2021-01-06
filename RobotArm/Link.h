#pragma once


class Link
{
public:
	//double x, y, z;		//x,y,z coordinates of origin
	//double rx, ry, rz;	//rotations about x,y,z
	double length;		//length of the link

	enum linkDirection {alongX, alongY, alongZ}; //which axis defines the direction of the link

	//Link* parent;		//pointer to parent link

	//blank constructor
	Link() {};

	//constructor using only length and parent link
	//assigns new origin to the end of the parent link
	Link(double _length){//, Link* _parent) {
		//zero out initial rotations
		//rx = 0.;
		//ry = 0.;
		//rz = 0.;
		
		//assign length & parent link
		length = _length;
		//parent = _parent;

		//create new origin based on parent origin, rotation & length
		//_parent->update_child_start_pos(x, y, z, rx, ry, rz);
	}

	//constructor using new origin, rotations, length & parent
	//Link(double _x, double _y, double _z, double _length, 
	//	 double _rx = 0, double _ry = 0, double _rz = 0, Link* _parent = nullptr) {

	//	x = _x;
	//	y = _y;
	//	z = _z;
	//	rx = _rx;
	//	ry = _ry;
	//	rz = _rz;
	//	length = _length;
	//	parent = _parent;

	//}

	//draws a simple version of the link, at the origin & rotation
	void draw_simple(double centerX, double centerY, double centerZ);

	//updates the starting position & rotation for the child link
	//void update_child_start_pos(double &in_x, double &in_y, double &in_z,
	//							double &in_rx, double &in_ry, double &in_rz);
};

