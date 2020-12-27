#pragma once
#include <vector>
#include "Link.h"

class Arm
{
public:

	std::vector<Link> theLinks; //vector of links that forms the arm. 0th link is the base

	//draws the Arm in its current configuration
	void draw();
};

