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
	Link(double _length, linkDirection _direction){
		
		//assign length & direction
		length = _length;
		direction = _direction;

	}

	//draws a simple version of the link, at the origin & rotation
	void draw_simple();

};

