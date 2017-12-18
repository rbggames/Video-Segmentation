#pragma once
#include "Shape.h"

class ShapeHook : public Shape
{
public:
	ShapeHook(int x, int y,int width,Scalar colour=Scalar(255,0,0));
	ShapeHook(int x, int y, int width, String texturePath);
	~ShapeHook();
private:
	Point hook_points[1][7];
};

