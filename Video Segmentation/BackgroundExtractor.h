#pragma once
#include <opencv2/opencv.hpp>
#include "TrackedObjects.h"

using namespace cv;

class BackgroundExtractor
{
public:
	Mat background;
	BackgroundExtractor(Mat frame,int threshold);
	~BackgroundExtractor();
	void update(Mat frame); 
	void update(Mat frame, TrackedObjects objects, int size);
	Mat getForground();
private:
	int thresh;
	Mat previousFrame,mask,forground;
};

