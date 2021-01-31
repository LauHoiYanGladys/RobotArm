#include <stdio.h>
#include <algorithm>    // std::max
#include <string>
//#include "GL\glut.h"

#include "fssimplewindow.h"
#include "GraphicFont.h"

#include "DrawingUtilNG.h"

using namespace std;

using namespace DrawingUtilNG;

double DrawingUtilNG::getDistance(double x0, double y0, double x1, double y1)
{
	return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

float DrawingUtilNG::getDistance(vertexF v1, vertexF v2)
{
	return vectorLength({ v2.x - v1.x, v2.y - v1.y, v2.z - v1.z });
}

float DrawingUtilNG::vectorLength(vertexF v0)
{
	return sqrt(v0.x * v0.x + v0.y * v0.y + v0.z * v0.z);
}

vertexF DrawingUtilNG::getUnitVector(vertexF v0)
{
	return getUnitVector({ 0., 0., 0. }, v0);
}

vertexF DrawingUtilNG::getUnitVector(vertexF v1, vertexF v2)
{
	vertexF vector = { (v2.x - v1.x) , (v2.y - v1.y) , (v2.z - v1.z) };

	float length = vectorLength(vector);

	vector = { vector.x / length, vector.y / length, vector.z / length };

	return vector;
}

vertexF DrawingUtilNG::vectorSum(vertexF v1, vertexF v2)
{
	return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

vertexF DrawingUtilNG::vectorSum(vertexF v1, vertexF v2, vertexF v3)
{
	return { v1.x + v2.x + v3.x, v1.y + v2.y + v3.y, v1.z + v2.z + v3.z };
}

vertexF DrawingUtilNG::scalarProduct(vertexF v0, float scalar)
{
	return { v0.x * scalar, v0.y * scalar, v0.z * scalar };
}

float DrawingUtilNG::dotProduct(vertexF v1, vertexF v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

vertexF DrawingUtilNG::crossProduct(vertexF v1, vertexF v2)
{
	return {
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	};
}

vertexF DrawingUtilNG::rotateVector(vertexF v, vertexF k, float theta)
{
	double cosTheta = cos(theta * atan(1.) / 45.);
	double sinTheta = sin(theta * atan(1.) / 45.);

	// make sure k is a unit vector
	k = getUnitVector(k);

	// using Rodrigues' Rotation Formula
	vertexF firstTerm = scalarProduct(v, cosTheta);
	vertexF secondTerm = scalarProduct(crossProduct(k, v), sinTheta);
	vertexF thirdTerm = scalarProduct(k, dotProduct(k, v) * (1 - cosTheta));

	return vectorSum(firstTerm, secondTerm, thirdTerm);
}

void DrawingUtilNG::drawCircle(double centerX, double centerY,
	double radius, bool filled)
{
	float x, y, angle;
	float radianConvert = atan(1.) / 45.;
	radius = fabs(radius);

	// adapt the number of segments based on radius size
	int stepSize = 1;
	if (radius < 10)
		stepSize = 3;
	else if (radius < 200)
		stepSize = round((3. - 1.) / (10. - 200.) * (radius - 200.) + 1.);

	stepSize *= 6;  // always want stepSize to be a factor of 360

	if (filled)
		glBegin(GL_POLYGON);
	else
		glBegin(GL_LINE_LOOP);

	for (int i = 0; i < 360; i += stepSize) {
		angle = i * radianConvert;
		x = cos(angle) * radius + centerX;
		y = sin(angle) * radius + centerY;
		glVertex2d(x, y);
	}
	glEnd();
}

void DrawingUtilNG::drawCircleZ(double centerX, double centerY, double centerZ,
	double radius, bool filled)
{
	float x, y, angle;
	float radianConvert = atan(1.) / 45.;
	radius = fabs(radius);

	// adapt the number of segments based on radius size
	int stepSize = 1;
	if (radius < 10)
		stepSize = 3;
	else if (radius < 200)
		stepSize = round((3. - 1.) / (10. - 200.) * (radius - 200.) + 1.);

	stepSize *= 6;  // always want stepSize to be a factor of 360

	if (filled)
		glBegin(GL_POLYGON);
	else
		glBegin(GL_LINE_LOOP);

	for (int i = 0; i < 360; i += stepSize) {
		angle = i * radianConvert;
		x = cos(angle) * radius + centerX;
		y = sin(angle) * radius + centerY;
		glVertex3d(x, y, centerZ);
	}
	glEnd();
}

void DrawingUtilNG::drawCircleX(double centerX, double centerY, double centerZ,
	double radius, bool filled)
{
	float z, y, angle;
	float radianConvert = atan(1.) / 45.;
	radius = fabs(radius);

	// adapt the number of segments based on radius size
	int stepSize = 1;
	if (radius < 10)
		stepSize = 3;
	else if (radius < 200)
		stepSize = round((3. - 1.) / (10. - 200.) * (radius - 200.) + 1.);

	stepSize *= 6;  // always want stepSize to be a factor of 360

	if (filled)
		glBegin(GL_POLYGON);
	else
		glBegin(GL_LINE_LOOP);

	for (int i = 0; i < 360; i += stepSize) {
		angle = i * radianConvert;
		z = cos(angle) * radius + centerZ;
		y = sin(angle) * radius + centerY;
		glVertex3d(centerX, y, z);
	}
	glEnd();
}


void DrawingUtilNG::drawCylinderX(double r_top, double r_bot, double height, double centerX, double centerY, double centerZ) {
	float y_curr, y_prev, z_curr, z_prev, angle;
	float radianConvert = atan(1.) / 45.;
	r_top = fabs(r_top);
	r_bot = fabs(r_bot);

	// adapt the number of segments based on radius size
	int stepSize = 1;

	double checkRad = max(r_top, r_bot);
	if (checkRad < 10)
		stepSize = 3;
	else if (checkRad < 200)
		stepSize = round((3. - 1.) / (10. - 200.) * (checkRad - 200.) + 1.);

	stepSize *= 6;  // always want stepSize to be a factor of 360

	y_prev = 1;
	z_prev = 0;

	for (int i = stepSize; i <= 360; i += stepSize) {

		angle = i * radianConvert;
		y_curr = cos(angle);
		z_curr = sin(angle);

		//rectangle for outside of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((-height / 2) + centerX, (y_prev * r_bot) + centerY, (z_prev * r_bot) + centerZ);
		glVertex3d((height / 2) + centerX, (y_prev * r_top) + centerY, (z_prev * r_bot) + centerZ);
		glVertex3d((height / 2) + centerX, (y_curr * r_top) + centerY, (z_curr * r_bot) + centerZ);
		glVertex3d((-height / 2) + centerX, (y_curr * r_bot) + centerY, (z_curr * r_bot) + centerZ);
		glEnd();

		//triangle for top of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((height / 2) + centerX, (y_prev * r_top) + centerY, (z_prev * r_bot) + centerZ);
		glVertex3d((height / 2) + centerX, (y_curr * r_top) + centerY, (z_curr * r_bot) + centerZ);
		glVertex3d((height / 2) + centerX, 0 + centerY, 0 + centerZ);
		glEnd();

		//triangle for bottom of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((-height / 2) + centerX, (y_prev * r_bot) + centerY, (z_prev * r_bot) + centerZ);
		glVertex3d((-height / 2) + centerX, (y_curr * r_bot) + centerY, (z_curr * r_bot) + centerZ);
		glVertex3d((-height / 2) + centerX, 0 + centerY, 0 + centerZ);
		glEnd();

		y_prev = y_curr;
		z_prev = z_curr;
	}
}

void DrawingUtilNG::drawCylinderY(double r_top, double r_bot, double height, double centerX, double centerY, double centerZ) {
	float x_curr, x_prev, z_curr, z_prev, angle;
	float radianConvert = atan(1.) / 45.;
	r_top = fabs(r_top);
	r_bot = fabs(r_bot);

	// adapt the number of segments based on radius size
	int stepSize = 1;

	double checkRad = max(r_top, r_bot);
	if (checkRad < 10)
		stepSize = 3;
	else if (checkRad < 200)
		stepSize = round((3. - 1.) / (10. - 200.) * (checkRad - 200.) + 1.);

	stepSize *= 6;  // always want stepSize to be a factor of 360

	x_prev = 1;
	z_prev = 0;

	for (int i = stepSize; i <= 360; i += stepSize) {
	
		angle = i * radianConvert;
		x_curr = cos(angle);
		z_curr = sin(angle);
		
		//rectangle for outside of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((x_prev * r_bot) + centerX, (-height / 2) + centerY, (z_prev * r_bot) + centerZ);
		glVertex3d((x_prev * r_top) + centerX, (height / 2) + centerY, (z_prev * r_top) + centerZ);
		glVertex3d((x_curr * r_top) + centerX, (height / 2) + centerY, (z_curr * r_top) + centerZ);
		glVertex3d((x_curr * r_bot) + centerX, (-height / 2) + centerY, (z_curr * r_bot) + centerZ);
		glEnd();

		//triangle for top of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((x_prev * r_top) + centerX, (height / 2) + centerY, (z_prev * r_top) + centerZ);
		glVertex3d((x_curr * r_top) + centerX, (height / 2) + centerY, (z_curr * r_top) + centerZ);
		glVertex3d(0 + centerX, (height / 2) + centerY, 0 + centerZ);
		glEnd();

		//triangle for bottom of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((x_prev * r_bot) + centerX, (-height / 2) + centerY, (z_prev * r_bot) + centerZ);
		glVertex3d((x_curr * r_bot) + centerX, (-height / 2) + centerY, (z_curr * r_bot) + centerZ);
		glVertex3d(0 + centerX, (-height / 2) + centerY, 0 + centerZ);
		glEnd();

		x_prev = x_curr;
		z_prev = z_curr;
	}
}

void DrawingUtilNG::drawCylinderZ(double r_top, double r_bot, double height, double centerX, double centerY, double centerZ) {
	float x_curr, x_prev, y_curr, y_prev, angle;
	float radianConvert = atan(1.) / 45.;
	r_top = fabs(r_top);
	r_bot = fabs(r_bot);

	// adapt the number of segments based on radius size
	int stepSize = 1;

	double checkRad = max(r_top, r_bot);
	if (checkRad < 10)
		stepSize = 3;
	else if (checkRad < 200)
		stepSize = round((3. - 1.) / (10. - 200.) * (checkRad - 200.) + 1.);

	stepSize *= 6;  // always want stepSize to be a factor of 360

	x_prev = 1;
	y_prev = 0;

	for (int i = stepSize; i <= 360; i += stepSize) {

		angle = i * radianConvert;
		x_curr = cos(angle);
		y_curr = sin(angle);

		//rectangle for outside of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((x_prev * r_bot) + centerX, (y_prev * r_bot) + centerY, (-height / 2) + centerZ);
		glVertex3d((x_prev * r_top) + centerX, (y_prev * r_top) + centerY, (height / 2) + centerZ);
		glVertex3d((x_curr * r_top) + centerX, (y_curr * r_top) + centerY, (height / 2) + centerZ);
		glVertex3d((x_curr * r_bot) + centerX, (y_curr * r_bot) + centerY, (-height / 2) + centerZ);
		glEnd();

		//triangle for top of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((x_prev * r_top) + centerX, (y_prev * r_top) + centerY, (height / 2) + centerZ);
		glVertex3d((x_curr * r_top) + centerX, (y_curr * r_top) + centerY, (height / 2) + centerZ);
		glVertex3d(0 + centerX, 0 + centerY, (height / 2) + centerZ);
		glEnd();

		//triangle for bottom of cylinder
		glBegin(GL_POLYGON);
		glVertex3d((x_prev * r_bot) + centerX, (y_prev * r_bot) + centerY, (-height / 2) + centerZ);
		glVertex3d((x_curr * r_bot) + centerX, (y_curr * r_bot) + centerY, (-height / 2) + centerZ);
		glVertex3d(0 + centerX, 0 + centerY, (-height / 2) + centerZ);
		glEnd();

		x_prev = x_curr;
		y_prev = y_curr;
	}
}

void DrawingUtilNG::drawCylinderXOffset(double r_top, double r_bot, double height, double centerX, double centerY, double centerZ)
{
	drawCylinderX(r_top, r_bot, height, centerX + (height / 2), centerY, centerZ);
}

void DrawingUtilNG::drawCylinderYOffset(double r_top, double r_bot, double height, double centerX, double centerY, double centerZ)
{
	drawCylinderY(r_top, r_bot, height, centerX, centerY + (height / 2), centerZ);
}

void DrawingUtilNG::drawCylinderZOffset(double r_top, double r_bot, double height, double centerX, double centerY, double centerZ)
{
	drawCylinderZ(r_top, r_bot, height, centerX, centerY, centerZ + (height / 2));
}


void DrawingUtilNG::drawPrismZ(double x_size, double y_size, double height, double centerX, double centerY, double centerZ)
{
	double x1 = centerX - (x_size / 2);
	double x2 = centerX + (x_size / 2);
	double y1 = centerY - (y_size / 2);
	double y2 = centerY + (y_size / 2);
	double z1 = centerZ - (height / 2);
	double z2 = centerZ + (height / 2);

	drawCube(x1, y1, z1, x2, y2, z2);
}

void DrawingUtilNG::drawPrismZOffset(double x_size, double y_size, double height, double centerX, double centerY, double centerZ)
{
	drawPrismZ(x_size, y_size, height, centerX, centerY, centerZ + (height / 2));
}

void DrawingUtilNG::drawRectangle(double x, double y, int sizeX, int sizeY, bool filled)
{
	if (filled)
		glBegin(GL_POLYGON);
	else
		glBegin(GL_LINE_LOOP);

	glVertex2i(x, y);
	glVertex2i(x + sizeX, y);
	glVertex2i(x + sizeX, y + sizeY);
	glVertex2i(x, y + sizeY);

	glEnd();
}

void DrawingUtilNG::drawStar(double centerX, double centerY,
	double outerSize, double innerSize, int numbPoints, double startAngle, bool filled)
{
	float radianConvert = atan(1.) / 45.;
	double stepSize = 360. / 2. / numbPoints;

	if (filled) {
		// GL_POLYGON cannot do concave shapes, so must draw series of quads (rhombuses)
		// BTW, quad strip don't work because they would share the "wrong edge"
		double currAngle, longX, longY;
		double currShortX, currShortY, prevShortX, prevShortY;

		currAngle = (startAngle - stepSize) * radianConvert;
		prevShortX = cos(currAngle) * innerSize + centerX;
		prevShortY = -sin(currAngle) * innerSize + centerY;

		for (int i = 0; i < numbPoints; i++) {
			currAngle += stepSize * radianConvert;
			longX = cos(currAngle) * outerSize + centerX;
			longY = -sin(currAngle) * outerSize + centerY;
			currAngle += stepSize * radianConvert;
			currShortX = cos(currAngle) * innerSize + centerX;
			currShortY = -sin(currAngle) * innerSize + centerY;

			glBegin(GL_QUADS);
			glVertex2d(centerX, centerY);
			glVertex2d(prevShortX, prevShortY);
			glVertex2d(longX, longY);
			glVertex2d(currShortX, currShortY);
			glEnd();

			prevShortX = currShortX;
			prevShortY = currShortY;
		}
	}
	else {
		double x, y, currRadius, currAngle;
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < (numbPoints * 2); i++) {
			currAngle = (startAngle + i * stepSize) * radianConvert;

			// if i is even , use outerSize 
			// (remainder of i div 2 is zero which means false, 
			// which means use second choice on trinary operator)
			currRadius = i % 2 ? innerSize : outerSize;

			x = cos(currAngle) * currRadius + centerX;
			y = -sin(currAngle) * currRadius + centerY;
			glVertex2d(x, y);
		}
		glEnd();
	}
}

void DrawingUtilNG::drawStarGram(double centerX, double centerY, double size,
	int numbPoints, int numbSkip, double startAngle, bool filled)
{
	int i;
	float radianConvert = atan(1.) / 45.;
	double stepSize = 360. / numbPoints;

	if (numbSkip >= numbPoints / 2)
		numbSkip = numbPoints / 2 - 1;

	if (filled) {
		double x0, y0, x1, y1;
		double currAngle = startAngle * radianConvert;

		x0 = cos(currAngle) * size + centerX;
		y0 = -sin(currAngle) * size + centerY;

		for (i = 0; i < numbPoints / 2; i++) {
			currAngle += stepSize * radianConvert * (numbSkip + 1);
			x1 = cos(currAngle) * size + centerX;
			y1 = -sin(currAngle) * size + centerY;
			glBegin(GL_TRIANGLES);
			glVertex2d(centerX, centerY);
			glVertex2d(x0, y0);
			glVertex2d(x1, y1);

			glEnd();
			x0 = x1; y0 = y1;
		}
		if (!(numbPoints - numbSkip + 1)% 2) {// if even number of points
			currAngle -= stepSize * radianConvert;
			x0 = cos(currAngle) * size + centerX;
			y0 = -sin(currAngle) * size + centerY;
		}

		for (; i < numbPoints; i++) {
			currAngle += stepSize * radianConvert * (numbSkip + 1);
			x1 = cos(currAngle) * size + centerX;
			y1 = -sin(currAngle) * size + centerY;
			glBegin(GL_TRIANGLES);
			glVertex2d(centerX, centerY);
			glVertex2d(x0, y0);
			glVertex2d(x1, y1);

			glEnd();
			x0 = x1; y0 = y1;
		}
	}
	else {
		int i;
		double x, y;
		double currAngle = startAngle * radianConvert;

		x = cos(currAngle) * size + centerX;
		y = -sin(currAngle) * size + centerY;
		glBegin(GL_LINE_LOOP);
		glVertex2d(x, y);

		for (i = 0; i < numbPoints / 2; i++) {
			currAngle += stepSize * radianConvert * (numbSkip + 1);

			x = cos(currAngle) * size + centerX;
			y = -sin(currAngle) * size + centerY;
			glVertex2d(x, y);

		}
		if (!(numbPoints - numbSkip + 1)% 2) {// if even number of points
			glEnd();
			currAngle -= stepSize * radianConvert;
			x = cos(currAngle) * size + centerX;
			y = -sin(currAngle) * size + centerY;
			glBegin(GL_LINE_LOOP);
			glVertex2d(x, y);

		}
		for (; i < numbPoints; i++) {
			currAngle += stepSize * radianConvert * (numbSkip + 1);

			x = cos(currAngle) * size + centerX;
			y = -sin(currAngle) * size + centerY;
			glVertex2d(x, y);

		}
		glEnd();

	}
}

//// drawStar test
//int currStarAngle = 0;
//DrawingUtilNG::drawStarGram(550, 180, 100, 9, 1, 90, true);
//DrawingUtilNG::drawStar(400, 400, 100, 100, 8, 90, true);
//glColor3ub(0, 255, 255);
//DrawingUtilNG::drawStarGram(550, 180, 70, 9, 1, 90, false);
//DrawingUtilNG::drawStar(400, 400, 95, 40, 8, 0, false);


bool DrawingUtilNG::hsv2rgb(double H, double S, double V, double& red, double& green, double& blue)
{
	// using formula from:
	// https://www.rapidtables.com/convert/color/hsv-to-rgb.html

	// adjust H to be 0 <= H <= 360
	H = fabs(fmod(H, 360.));  // note that H=360 is the same as H=0

	double C = V * S;
	double X = C * (1 - fabs(fmod(H / 60., 2) - 1));
	double m = V - C;
	double Rprime, Gprime, Bprime;

	if (H < 0.) {  // unlikely due to adjustment above
		return false;  //  <<< possible exit from function
	}
	else if (H < 60.) {
		Rprime = C;
		Gprime = X;
		Bprime = 0.;
	}
	else if (H < 120.) {
		Rprime = X;
		Gprime = C;
		Bprime = 0.;
	}
	else if (H < 180.) {
		Rprime = 0.;
		Gprime = C;
		Bprime = X;
	}
	else if (H < 240.) {
		Rprime = 0.;
		Gprime = X;
		Bprime = C;
	}
	else if (H < 300.) {
		Rprime = X;
		Gprime = 0.;
		Bprime = C;
	}
	else if (H < 360.) {
		Rprime = C;
		Gprime = 0.;
		Bprime = X;
	}
	else
		return false;  //  <<< possible exit from function

	red = Rprime + m;
	green = Gprime + m;
	blue = Bprime + m;

	return true;
}

/*
void DrawingUtilNG::writeHelvetica(std::string outString, int locX, int locY, int size)
{
	void *fontCode;
	switch (size) {
	case 10: fontCode = GLUT_BITMAP_HELVETICA_10;
		break;
	case 12: fontCode = GLUT_BITMAP_HELVETICA_12;
		break;
	case 18: fontCode = GLUT_BITMAP_HELVETICA_18;
		break;
	default: fontCode = GLUT_BITMAP_HELVETICA_10;
		break;
	}

	glRasterPos2i(locX, locY);
	for (int i = 0; i < outString.length(); i++) {
		glutBitmapCharacter(fontCode,
			outString.substr(i, 1).c_str()[0]);
	}
}
void DrawingUtilNG::writeTimesRoman(std::string outString, int locX, int locY, int size)
{
	void *fontCode;
	switch (size) {
	case 10: fontCode = GLUT_BITMAP_TIMES_ROMAN_10;
		break;
	case 24: fontCode = GLUT_BITMAP_TIMES_ROMAN_24;
		break;
	default: fontCode = GLUT_BITMAP_TIMES_ROMAN_10;
		break;
	}

	glutInitWindowPosition(45, 76);
	glRasterPos2i(locX, locY);
	for (int i = 0; i < outString.length(); i++) {
		glutBitmapCharacter(fontCode,
			outString.substr(i, 1).c_str()[0]);
	}
}
*/
void DrawingUtilNG::coverAndFade(int x1, int y1, int x2, int y2, int totalT, int currT,
	int r, int g, int b)
{
	if (currT > totalT)
		currT = totalT;
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	int temp = currT / totalT * 255;
	glColor4ub(r, g, b, currT / (double)totalT * 255);
	//glColor4ub(0, 0, 0, 180);
	drawRectangle(x1, y1, x2, y2, true);

	glDisable(GL_BLEND);

}

void DrawingUtilNG::drawCube(vertexF v1, vertexF v2, bool sixColors)
{
	drawCube(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, sixColors);
}

void DrawingUtilNG::drawCube(double x1, double y1, double z1,
	double x2, double y2, double z2, bool sixColors, GraphicFont* aFont)
{
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1, 1);

	glBegin(GL_QUADS);

	if (sixColors)
		glColor3ub(0, 0, 255);  // blue face
	glVertex3d(x1, y1, z1);
	glVertex3d(x2, y1, z1);
	glVertex3d(x2, y2, z1);
	glVertex3d(x1, y2, z1);



	if (sixColors)
		glColor3ub(0, 255, 0);  // green
	glVertex3d(x1, y1, z2);
	glVertex3d(x2, y1, z2);
	glVertex3d(x2, y2, z2);
	glVertex3d(x1, y2, z2);

	if (sixColors)
		glColor3ub(0, 255, 255); // cyan
	glVertex3d(x1, y1, z1);
	glVertex3d(x2, y1, z1);
	glVertex3d(x2, y1, z2);
	glVertex3d(x1, y1, z2);

	if (sixColors)
		glColor3ub(255, 0, 0);  // red
	glVertex3d(x1, y2, z1);
	glVertex3d(x2, y2, z1);
	glVertex3d(x2, y2, z2);
	glVertex3d(x1, y2, z2);

	if (sixColors)
		glColor3ub(255, 0, 255); // magenta
	glVertex3d(x1, y1, z1);
	glVertex3d(x1, y2, z1);
	glVertex3d(x1, y2, z2);
	glVertex3d(x1, y1, z2);

	if (sixColors)
		glColor3ub(255, 255, 0);  // yellow
	glVertex3d(x2, y1, z1);
	glVertex3d(x2, y2, z1);
	glVertex3d(x2, y2, z2);
	glVertex3d(x2, y1, z2);

	glEnd();

	glDisable(GL_POLYGON_OFFSET_FILL);

	 //font tester (needs GraphicFont.h)
	if (aFont != NULL) {
		aFont->setColorHSV(30, 1, 1);
		//aFont->drawText3D("  ONE is the loneliest number",
		aFont->drawText3D(" 1ne",
			{ (float)x2, (float)y1, (float)z2 }, { (float)x2, (float)y2, (float)z2 }, { (float)x2, (float)y1, (float)z1 }, .15, 10.);

		aFont->drawText3D(" 2wo",
			{ (float)x2, (float)y1, (float)z1 }, { (float)x2, (float)y2, (float)z1 }, { (float)x1, (float)y1, (float)z1 }, .15, 10.);

		aFont->drawText3D(" 6ix",
			{ (float)x1, (float)y1, (float)z1 }, { (float)x1, (float)y2, (float)z1 }, { (float)x1, (float)y1, (float)z2 }, .15, 10.);

		aFont->drawText3D(" 5ive",
			{ (float)x1, (float)y1, (float)z2 }, { (float)x1, (float)y2, (float)z2 }, { (float)x2, (float)y1, (float)z2 }, .15, 10.);

		aFont->drawText3D(" 3ee",
			{ (float)x1, (float)y2, (float)z1 }, { (float)x2, (float)y2, (float)z1 }, { (float)x1, (float)y2, (float)z2 }, .15, 10.);

		aFont->drawText3D(" 4r",
			{ (float)x2, (float)y1, (float)z2 }, { (float)x2, (float)y1, (float)z1 }, { (float)x1, (float)y1, (float)z2 }, .15, 10.);

	}
}

void DrawingUtilNG::drawCube(double size)
{
	drawCube(-size / 2, size / 2, -size / 2, size / 2, -size / 2, size / 2);
}

bool DrawingUtilNG::buildStringFromFsInkey(int key, std::string& currString)
{
	bool shiftIsOn = FsGetKeyState(FSKEY_SHIFT);;

	// get key input (may want to move this someplace else for re-use)

	if (FSKEY_A <= key && key <= FSKEY_Z) {
		int adjustLetter = shiftIsOn ? 0 : 32;
		currString += char(int('A') + key - FSKEY_A + adjustLetter);
	}
	else if (FSKEY_0 <= key && key <= FSKEY_9) {
		if (shiftIsOn) {
			std::string shiftOnNumerals = ")!@#$%^&*(";
			currString += shiftOnNumerals.substr(key - FSKEY_0, 1);
		}
		else
			currString += char(int('0') + key - FSKEY_0);
	}
	else {
		// note that since switch jumps to proper case, 
		// having large number of cases doesn't slow down program
		switch (key) {
		case FSKEY_BS: // remove last character on string
			currString = currString.substr(0, currString.length() - 1);
			break;
		case FSKEY_SPACE:currString += " ";
			break;
		case FSKEY_TAB:currString += "\t";
			break;
		case FSKEY_DOT:
			if (shiftIsOn)
				currString += ">";
			else
				currString += ".";
			break;
		case FSKEY_ENTER: // allows soft line break
			if (shiftIsOn)
				currString += "\n";
			break;
		case FSKEY_COMMA:
			if (shiftIsOn)
				currString += "<";
			else
				currString += ",";
			break;
		case FSKEY_MINUS:
			if (shiftIsOn)
				currString += "_";
			else
				currString += "-";
			break;
		case FSKEY_PLUS:
			if (shiftIsOn)
				currString += "+";
			else
				currString += "=";
			break;
		case FSKEY_LBRACKET:
			if (shiftIsOn)
				currString += "{";
			else
				currString += "[";
			break;
		case FSKEY_RBRACKET:
			if (shiftIsOn)
				currString += "}";
			else
				currString += "]";
			break;
		case FSKEY_BACKSLASH:
			if (shiftIsOn)
				currString += "|";
			else
				currString += "\\"; // backslash
			break;
		case FSKEY_SEMICOLON:
			if (shiftIsOn)
				currString += ":";
			else
				currString += ";";
			break;
		case FSKEY_SINGLEQUOTE:
			if (shiftIsOn)
				currString += "\""; // double quote
			else
				currString += "'";
			break;
		case FSKEY_SLASH:
			if (shiftIsOn)
				currString += "?";
			else
				currString += "/";
			break;
		case FSKEY_TILDA:
			if (shiftIsOn)
				currString += "~";
			else
				currString += "`";
			break;

		}
	}
	return true; // may use this later ???
}

