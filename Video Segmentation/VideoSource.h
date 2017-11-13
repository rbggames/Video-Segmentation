#pragma once
#include <opencv2/opencv.hpp>
#include "ShapeHook.h"

using namespace cv;

class VideoSource
{
public:
	Shape* shapes[5];

	VideoSource();
	~VideoSource();

	bool getFrame(Mat frame);
};

