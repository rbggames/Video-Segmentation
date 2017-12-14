#include "stdafx.h"
#include "Shape.h"

//Protected
Shape::Shape()
{
	Point points[1][4];
	points[0][0] = Point(0, 0);
	points[0][1] = Point(50, 0);
	points[0][2] = Point(50,50);
	points[0][3] = Point(0, 50);


	const Point* pts[1] = { points[0] };

	const int npt[1] = { 4 };
	initialise(20, 30, 50, 50, pts, npt,Scalar(255,0,255));
}

Shape::Shape(int x, int y, int width, int height, const Point** pts, const int* npt,Scalar colour)
{
	initialise(x, y, width, height, pts, npt,colour);
}


Shape::~Shape()
{
}

Mat Shape::getMatrix()
{
	return Mat();
}

void Shape::updateDraw(Mat input)
{
	x += motion_x;
	y += motion_y;
	draw(input, x, y);
}

void Shape::setMotionVector(double mX, double mY)
{
	motion_x = mX;
	motion_y = mY;
}





void Shape::initialise(int x, int y, int width, int height, const Point ** pts, const int * npt, Scalar colour)
{
	this->x = x;
	this->y = y;
	this->width = width;
	this->height = height;

	shapeMat = Mat::zeros(width, height, CV_8UC3);
	fillPoly(shapeMat, pts, npt, 1, colour, 8);
	motion_x = 1;
	motion_y = 1;
}

Mat Shape::getShapeFrameMat()
{
	return shapeFrameMat;
}

int Shape::getX()
{
	return x;
}

int Shape::getY()
{
	return y;
}

int Shape::getWidth()
{
	return width;
}

int Shape::getHeight()
{
	return height;
}



void Shape::draw(Mat input,int xPos, int yPos)
{
	Mat translated;
	Mat translationMat, fineMask;
	Point2f src[3] = { Point2f(0, 0), Point2f(0,shapeMat.rows-1),Point2f(shapeMat.cols-1,shapeMat.rows-1) };
	Point2f dst[3] = { Point2f(xPos, yPos), Point2f(xPos, shapeMat.rows-1 + yPos),Point2f(shapeMat.cols-1 + xPos, shapeMat.rows-1 + yPos) };
	translationMat = getAffineTransform(src,dst);
	warpAffine(shapeMat, translated, translationMat, input.size(),1,BORDER_TRANSPARENT);
	inRange(translated, Scalar(0, 0, 0), Scalar(0, 0, 0), fineMask);
	bitwise_not(fineMask, fineMask);
	translated.copyTo(input, fineMask);
	translated.copyTo(shapeFrameMat);
}


