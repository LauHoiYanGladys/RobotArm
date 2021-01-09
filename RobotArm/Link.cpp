#include "Link.h"
#include "DrawingUtilNG.h"

void Link::draw(double extension)
{
	double radius = 1;

	//need to add cases depending on link direction
	switch (direction)
	{
	case alongX:
		DrawingUtilNG::drawCylinderXOffset(radius, radius, length, extension, 0, 0);
		return;
	case alongY:
		DrawingUtilNG::drawCylinderYOffset(radius, radius, length, 0, extension, 0);
		return;
	case alongZ:
		DrawingUtilNG::drawCylinderZOffset(radius, radius, length, 0, 0, extension);
		return;
	}
}

//void Link::draw_simple()
//{
//	double radius = 1;
//
//	//need to add cases depending on link direction
//	switch (direction)
//	{
//	case alongX:
//		DrawingUtilNG::drawCylinderXOffset(radius, radius, length, 0, 0, 0);
//		return;
//	case alongY:
//		DrawingUtilNG::drawCylinderYOffset(radius, radius, length, 0, 0, 0);
//		return;
//	case alongZ:
//		DrawingUtilNG::drawCylinderZOffset(radius, radius, length, 0, 0, 0);
//		return;
//	}		
//}
//
//void Link::draw_offset_prismatic(double offset)
//{
//	double radius = 1;
//	//DrawingUtilNG::drawCylinderZOffset(radius, radius, length, 0, 0, offset);
//	//need to add cases depending on link direction
//	switch (direction)
//	{
//	case alongX:
//		DrawingUtilNG::drawCylinderXOffset(radius, radius, length, offset, 0, 0);
//		return;
//	case alongY:
//		DrawingUtilNG::drawCylinderYOffset(radius, radius, length, 0, offset, 0);
//		return;
//	case alongZ:
//		DrawingUtilNG::drawCylinderZOffset(radius, radius, length, 0, 0, offset);
//		return;
//	}
//}

void Link::assignLinkDirection(linkDirection theDirection)
{
	direction = theDirection;
}
