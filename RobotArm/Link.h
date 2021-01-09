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
	//if link is prismatic, will use the extension input to extend length of link in appropriate direction
	void draw(double extension);

	//draws a simple version of the link (cylinder) at (0,0,0)
	//void draw_simple();

	// draws the link (cylinder) of a prismatic joint, with extension of the joint in the direction of the link
	//void draw_offset_prismatic(double offset);

	// assigns link direction
	void assignLinkDirection(linkDirection theDirection);
};

