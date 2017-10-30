#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

class Shape
{
public:
	int x, y, width, height;
	Shape(int x,int y, int width,int height, const Point** pts);
	~Shape();
	Mat getMatrix();
private:
	 Mat matrix;
};

