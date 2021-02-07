#pragma once
#include "DrawingUtilNG.h"

class Obstacle
{
public:
	DrawingUtilNG::vertexF position;	//center of obstacle (rectangular prism)
	DrawingUtilNG::vertexF size;		//size of obstacle cube (rectangular prism centered about 'position')

	//default constructor
	Obstacle() {
		position = { 0,0,0 };
		size = { 0,0,0 };	
	};
	
	//constructor using vertexF inputs for position & size
	Obstacle(DrawingUtilNG::vertexF _position, DrawingUtilNG::vertexF _size) {
		position = _position;
		size = _size;
	}

	//function to draw obstacle
	void draw();

	//function to check whether coordinate is inside obstacle (false = within obstacle, true = clear of obstacle)
	bool isClear(DrawingUtilNG::vertexF coord);


};

