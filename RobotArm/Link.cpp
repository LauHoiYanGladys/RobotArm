#include "Link.h"
#include "DrawingUtilNG.h"
#include "fssimplewindow.h"

void Link::draw(double extension)
{
	double radius = 1;

	//need to add cases depending on link direction
	switch (direction)
	{
	case linkDirection::alongX:
		switch (offsetDirection) {
		case linkOffsetDirection::alongYOffset:
			DrawingUtilNG::drawCylinderXOffset(radius, radius, length, extension, offset, 0);
			return;
		case linkOffsetDirection::alongZOffset:
			DrawingUtilNG::drawCylinderXOffset(radius, radius, length, extension, 0, offset);
			return;
		default:
			DrawingUtilNG::drawCylinderXOffset(radius, radius, length, extension, 0, 0);
			return;
		}

	case linkDirection::alongY:
		switch (offsetDirection) {
		case linkOffsetDirection::alongXOffset:
			DrawingUtilNG::drawCylinderYOffset(radius, radius, length, offset, extension, 0);
			return;
		case linkOffsetDirection::alongZOffset:
			DrawingUtilNG::drawCylinderYOffset(radius, radius, length, 0, extension, offset);
			return;
		default:
			DrawingUtilNG::drawCylinderYOffset(radius, radius, length, 0, extension, 0);
			return;
		}


	case linkDirection::alongZ:
		switch (offsetDirection) {
		case linkOffsetDirection::alongXOffset:
			DrawingUtilNG::drawCylinderZOffset(radius, radius, length, offset, 0, extension);
			return;
		case linkOffsetDirection::alongYOffset:
			DrawingUtilNG::drawCylinderZOffset(radius, radius, length, 0, offset, extension);
			return;
		default:
			DrawingUtilNG::drawCylinderZOffset(radius, radius, length, 0, 0, extension);
			return;
		}

	case linkDirection::alongX_negative:
		// store current matrix state
		glPushMatrix();

		// rotate about Z axis by 180 degrees to flip the x-axis to opposite direction
		glRotatef(180, 0, 0, 1);

		switch (offsetDirection) {
		case linkOffsetDirection::alongYOffset:
			DrawingUtilNG::drawCylinderXOffset(radius, radius, length, extension, offset, 0);
			return;
		case linkOffsetDirection::alongZOffset:
			DrawingUtilNG::drawCylinderXOffset(radius, radius, length, extension, 0, offset);
			return;
		default:
			DrawingUtilNG::drawCylinderXOffset(radius, radius, length, extension, 0, 0);
			return;
		}
		// restore original matrix state
		glPopMatrix();

	case linkDirection::alongY_negative:
		// store current matrix state
		glPushMatrix();

		// rotate about Z axis by 180 degrees to flip the y-axis to opposite direction
		glRotatef(180, 0, 0, 1);

		switch (offsetDirection) {
		case linkOffsetDirection::alongXOffset:
			DrawingUtilNG::drawCylinderYOffset(radius, radius, length, offset, extension, 0);
			return;
		case linkOffsetDirection::alongZOffset:
			DrawingUtilNG::drawCylinderYOffset(radius, radius, length, 0, extension, offset);
			return;
		default:
			DrawingUtilNG::drawCylinderYOffset(radius, radius, length, 0, extension, 0);
			return;
		}
		// restore original matrix state
		glPopMatrix();

	case linkDirection::alongZ_negative:
		// store current matrix state
		glPushMatrix();

		// rotate about X axis by 180 degrees to flip the z-axis to opposite direction
		glRotatef(180, 1, 0, 0);

		switch (offsetDirection) {
		case linkOffsetDirection::alongXOffset:
			DrawingUtilNG::drawCylinderZOffset(radius, radius, length, offset, 0, extension);
			return;
		case linkOffsetDirection::alongYOffset:
			DrawingUtilNG::drawCylinderZOffset(radius, radius, length, 0, offset, extension);
			return;
		default:
			DrawingUtilNG::drawCylinderZOffset(radius, radius, length, 0, 0, extension);
			return;
		}
		// restore original matrix state
		glPopMatrix();
	}
}

void Link::assignLinkDirection(linkDirection theDirection)
{
	direction = theDirection;
}

void Link::assignLinkOffsetProperties(linkOffsetDirection theOffsetDirection, double theOffset)
{
	offsetDirection = theOffsetDirection;
	offset = theOffset;
}


