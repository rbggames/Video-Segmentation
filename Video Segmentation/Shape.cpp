#include "stdafx.h"
#include "Shape.h"


Shape::Shape(int x, int y, int width, int height, const Point** pts)
{
	this->x = x;
	this->y = y;
	this->width = width;
	this->height = height;

	matrix = Mat::zeros(width, height, CV_8UC3);
}


Shape::~Shape()
{
}

Mat Shape::getMatrix()
{
	return Mat();
}
