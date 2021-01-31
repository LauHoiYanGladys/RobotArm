#pragma once


class Link
{
public:

	double length;				//length of the link
	
	enum class linkDirection {alongX, alongY, alongZ, alongX_negative, alongY_negative, alongZ_negative};
	linkDirection direction;	//which axis defines the direction of the link

	enum class linkOffsetDirection {alongXOffset, alongYOffset, alongZOffset};
	linkOffsetDirection offsetDirection; // direction of link offset from the joint center

	double offset; // amount of link offset from the joint center

	//blank constructor
	Link() {};

	//constructor using only length and direction
	Link(double _length){
		
		//assign length & direction
		length = _length;
		direction = linkDirection::alongZ;
		offsetDirection = linkOffsetDirection::alongYOffset;
		offset = 0.;
	}

	//draws a simple version of the link (cylinder) at (0,0,0) pointing in the desired direction
	//if joint is prismatic, will use the extension input to shift starting point of link in appropriate direction
	void draw(double extension);

	//draws a simple version of the link (cylinder) at (0,0,0)
	//void draw_simple();

	// draws the link (cylinder) of a prismatic joint, with extension of the joint in the direction of the link
	//void draw_offset_prismatic(double offset);

	// assigns link direction
	void assignLinkDirection(linkDirection theDirection);

	// assigns link offset direction and value of offset
	void assignLinkOffsetProperties(linkOffsetDirection theOffsetDirection, double theOffset);

};

