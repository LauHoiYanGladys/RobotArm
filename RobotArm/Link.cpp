#include "Link.h"
#include "DrawingUtilNG.h"

void Link::draw_simple()
{
	double radius = 1;

	//need to add cases depending on link direction
	DrawingUtilNG::drawCylinderZ(radius, radius, length, 0, 0, 0);
}
