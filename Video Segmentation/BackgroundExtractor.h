#pragma once
#include <opencv2/opencv.hpp>
#include "TrackedObject.h"

using namespace cv;

class BackgroundExtractor
{
public:
	Mat background;
	BackgroundExtractor(Mat frame,int threshold);
	~BackgroundExtractor();
	void update(Mat frame); 
	void update(Mat frame, TrackedObject** objects, int size);
private:
	int thresh;
	Mat previousFrame,mask;
};

