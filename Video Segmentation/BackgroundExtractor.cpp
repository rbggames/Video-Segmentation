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

	UMat flow;
	Mat nextGrey, previousGrey, flowPolar, out(frame.rows, frame.cols, CV_8UC3);
	Mat backgroundMask(frame.size(),CV_8U), forgroundMask;
	forground.setTo(Scalar(0, 0, 0));
	cvtColor(frame, nextGrey, CV_BGR2GRAY);
	cvtColor(previousFrame, previousGrey, CV_BGR2GRAY);
	calcOpticalFlowFarneback(previousGrey, nextGrey, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

	vector<Mat> channels(2);
	Mat angle, mag;
	// split img:
	split(flow, channels);
	cartToPolar(channels[0], channels[1], mag, angle, true);

	Mat hsv[3];
	split(out, hsv);

	hsv[0] = (angle);
	normalize(mag, hsv[1], 255, 255, NORM_MINMAX);
	normalize(mag, hsv[2], 0, 255, NORM_MINMAX);
	merge(hsv, 3, out);
	//hsv[1].setTo(254.9);
	merge(hsv, 3, out);
	cvtColor(out, out, CV_HSV2BGR);
	threshold(hsv[2], backgroundMask, 20, 255, THRESH_BINARY_INV);
	backgroundMask.convertTo(backgroundMask, CV_8U);
	frame.copyTo(background, backgroundMask);

	absdiff(background, frame, forgroundMask);
	cvtColor(forgroundMask, forgroundMask, CV_BGR2GRAY);
	threshold(forgroundMask, forgroundMask, 20, 255, THRESH_BINARY);
	
	//Expand mask
	Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(1, 1));
	dilate(forgroundMask, forgroundMask, structuringElement);

	frame.copyTo(forground, forgroundMask);

	imshow("Background", background);

	imshow("Forground", forground);

	frame.copyTo(previousFrame);

}

Mat BackgroundExtractor::getForground()
{
	return forground;
}
