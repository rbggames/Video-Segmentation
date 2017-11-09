#include "stdafx.h"
#include "Shape.h"



Shape::Shape(int x, int y, int width, int height, const Point** pts, const int* npt)
{
	this->x = x;
	this->y = y;
	this->width = width;
	this->height = height;

	shapeMat = Mat::zeros(width, height, CV_8UC3);
	fillPoly(shapeMat, pts, npt, 1, Scalar(255, 0, 255), 8);
	motion_x = 1;
	motion_y = 1;
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



void Shape::draw(Mat input,int xPos, int yPos)
{
	shapeMat.copyTo(input(cv::Rect(xPos, yPos, shapeMat.cols, shapeMat.rows)));
}


