#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

class Shape
{
public:
	Shape();
	Shape(int x, int y, int width, int height, const Point** pts, const int* npt, Scalar colour=Scalar(255,0,255));
	~Shape();
	Mat getMatrix();
	void updateDraw(Mat input);
	void setMotionVector(double mX, double mY);
protected:
	int x, y, width, height;
	double motion_x, motion_y;
	Scalar colour;
	void initialise(int x, int y, int width, int height, const Point** pts, const int* npt,Scalar colour);
private:
	void draw(Mat input, int x, int y);
	Mat shapeMat;
};

