#pragma once
#include <opencv2/opencv.hpp>
#include "Shape.h"
using namespace cv;

class VideoSource
{
public:
	VideoSource();
	~VideoSource();

	bool getFrame(Mat frame);
	Shape** getShapes(int* numberShapes);
private:
	Shape* shapes[5];
	int numShapes;
};

