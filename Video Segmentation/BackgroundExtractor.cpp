#include "stdafx.h"
#include "BackgroundExtractor.h"


BackgroundExtractor::BackgroundExtractor(Mat frame, int threshold_)
{
	frame.copyTo(previousFrame);
	thresh = threshold_;
}

BackgroundExtractor::~BackgroundExtractor()
{
}

void BackgroundExtractor::update(Mat frame)
{
	Mat diff;
	absdiff(frame, previousFrame, diff);
	imshow("diff", diff);
	cvtColor(diff, mask, CV_BGR2GRAY);
	threshold(mask, mask, thresh, 255, THRESH_BINARY_INV);
	frame.copyTo(background, mask);
	imshow("Background", background);

	frame.copyTo(previousFrame);
}

void BackgroundExtractor::update(Mat frame, TrackedObject** objects, int size)
{
	Mat diff;
	absdiff(frame, previousFrame, diff);
	imshow("diff", diff);
	cvtColor(diff, mask, CV_BGR2GRAY);
	threshold(mask, mask, thresh, 255, THRESH_BINARY_INV);
	imshow("back mask pre", mask);
	for (int i = 0; i < size; i++) {
		mask = mask - objects[i]->mask;
	}
	imshow("back mask", mask);
	frame.copyTo(background, mask);
	imshow("Background", background);
	frame.copyTo(previousFrame);
}
