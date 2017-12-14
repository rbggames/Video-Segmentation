#pragma once
#include "Shape.h"
class ShapeSquare : public Shape 
{
public:
	ShapeSquare(int x, int y, int width, Scalar colour = Scalar(255, 0, 0));
	~ShapeSquare();
private:
	Point hook_points[1][4];
};

