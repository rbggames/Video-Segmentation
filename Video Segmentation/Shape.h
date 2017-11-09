#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

class Shape
{
public:
	int x, y, width, height;
	int motion_x, motion_y;
	Shape(int x, int y, int width, int height, const Point** pts, const int* npt);
	~Shape();
	Mat getMatrix();
	void updateDraw(Mat input);
	void setMotionVector(double mX, double mY);
private:
	void draw(Mat input, int x, int y);
	Mat shapeMat;
};

