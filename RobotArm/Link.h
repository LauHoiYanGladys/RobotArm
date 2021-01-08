#pragma once


class Link
{
public:

	double length;				//length of the link
	
	enum linkDirection {alongX, alongY, alongZ}; 
	linkDirection direction;	//which axis defines the direction of the link

	//blank constructor
	Link() {};

	//constructor using only length and direction
	Link(double _length){
		
		//assign length & direction
		length = _length;
		direction = alongZ;

	}

	//draws a simple version of the link (cylinder) at (0,0,0) pointing in the desired direction
	void draw_simple();

	// draws the link (cylinder) of a prismatic joint at (0,0,z) where z is the extension of the joint
	// pointing in the z-direction, as prismatic joints always have links in that direction
	void draw_offset_prismatic(double offset);

	// assigns link direction
	void assignLinkDirection(linkDirection theDirection);
};

