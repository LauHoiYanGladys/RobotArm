#include "Link.h"
#include "DrawingUtilNG.h"

void Link::draw_simple(double centerX, double centerY, double centerZ)
{
	//change the type of cylinder based on the link direction
	DrawingUtilNG::drawCylinderZ(1., 1., length, centerX, centerY, centerZ);
}

//void Link::update_child_start_pos(double& in_x, double& in_y, double& in_z, double& in_rx, double& in_ry, double& in_rz)
//{
//}
