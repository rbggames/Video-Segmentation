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
	forground.setTo(Scalar(0, 0, 0));
	Mat diff;
	absdiff(frame, previousFrame, diff);
	imshow("diff", diff);
	cvtColor(diff, mask, CV_BGR2GRAY);
	threshold(mask, mask, thresh, 255, THRESH_BINARY_INV);
	frame.copyTo(forground, 1 - mask);
	frame.copyTo(background, mask);
	imshow("Background", background);

	frame.copyTo(previousFrame);
}

void BackgroundExtractor::update(Mat frame, TrackedObjects objects, int size)
{
	forground.setTo(Scalar(0, 0, 0));
	Mat diff;
	absdiff(frame, previousFrame, diff);
	imshow("diff", diff);
	cvtColor(diff, mask, CV_BGR2GRAY);
	threshold(mask, mask, thresh, 255, THRESH_BINARY_INV);
	imshow("back mask pre", mask);
	for (int i = 0; i < size; i++) {
		mask = mask - objects.getTrackedObject(i)->mask;
		//imshow("mask" + SSTR(i), objects.getTrackedObject(i)->mask);
	}
	//Dialate edges to close small gaps so contour detection creates one object
	Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(20, 20));
	for (int i = 0; i < 3; i++) {
		//dilate(mask, mask, structuringElement);
		erode(mask, mask, structuringElement);
	}
	//imshow("back mask", mask);

	frame.copyTo(background, mask);
	imshow("Background", background);


	absdiff(background, frame, diff);
	imshow("diff", diff);
	cvtColor(diff, mask, CV_BGR2GRAY);
	threshold(mask, mask, thresh, 255, THRESH_BINARY_INV);
	forground.setTo(0);
	frame.copyTo(forground, 1 - mask);
	imshow("forground", forground);
	frame.copyTo(previousFrame);
}

Mat BackgroundExtractor::getForground()
{
	return forground;
}
