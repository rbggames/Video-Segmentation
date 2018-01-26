#include "stdafx.h"
#include "Utilities.h"
#include "TrackedObject.h"


void refineMask(Mat frame, Mat mask, Rect2d boundingBox) {
	//Mask or bounding box undefined
	if (mask.size().area() == 0 || boundingBox.area() == 0)
		return;

	//Get Edges
	Mat cannyFrame;
	Canny(frame, cannyFrame, 100, 200);
	imshow("can", cannyFrame);
	mask.setTo(0);

	//Initialise mask
	rectangle(mask, boundingBox, Scalar(255), CV_FILLED);
	bool leftEdgeFound, rightEdgeFound;
	int x;
	int thresh = 30;
	int borderSize = 4;
	x = boundingBox.x;
	for (int y = boundingBox.y; y < boundingBox.y + boundingBox.height; y++) {
		leftEdgeFound = false;
		rightEdgeFound = false;
		for (int xOffset = -borderSize; xOffset < (boundingBox.width); xOffset++) {
			if (y > 0 && y < frame.size().height) {
				if (x + xOffset > 0 && x + xOffset < frame.size().width) {
					if (!leftEdgeFound && cannyFrame.at<unsigned char>(y, x + xOffset) < thresh) {
						mask.at<unsigned char>(y, x + xOffset) = 0;
					}
					else {
						leftEdgeFound = true;
					}
				}
				else {
					leftEdgeFound = true;
				}

				if (x + boundingBox.width - xOffset > 0 && x + boundingBox.width - xOffset < frame.size().width) {
					if (!rightEdgeFound && cannyFrame.at<unsigned char>(y, x + boundingBox.width - xOffset) < thresh) {
						mask.at<unsigned char>(y, x + boundingBox.width - xOffset) = 0;
					}
					else {
						rightEdgeFound = true;
					}
				}
				else {
					rightEdgeFound = true;
				}
			}
		}
	}
}

bool compareContourSize(vector<cv::Point> i, vector<cv::Point> j) { 
	double areaI = boundingRect(i).area();
	double areaJ = boundingRect(j).area();
	return (areaI>areaJ);
}

Mat kNearestColours(Mat source, int k) {
	Mat src;
	cvtColor(source, src, CV_BGR2HSV);
	Mat samples(src.rows * src.cols, 3, CV_32F);
	for (int y = 0; y < src.rows; y++)
		for (int x = 0; x < src.cols; x++)
			for (int z = 0; z < 3; z++)
				samples.at<float>(y + x*src.rows, z) = src.at<Vec3b>(y, x)[z];

	Mat labels;
	int attempts = 2;
	Mat centers;
	Mat dst(src.size(),src.type());

	kmeans(samples, k, labels, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers);

	Mat sr(src.size(), src.type());
	for (int y = 0; y < src.rows; y++) {
		for (int x = 0; x < src.cols; x++)
		{
			int cluster_idx = labels.at<int>(y + x*src.rows, 0);
			dst.at<Vec3b>(y, x)[0] = centers.at<float>(cluster_idx, 0);
			dst.at<Vec3b>(y, x)[1] = centers.at<float>(cluster_idx, 1);
			dst.at<Vec3b>(y, x)[2] = centers.at<float>(cluster_idx, 2);
		}
	}
	Mat dest;

	cvtColor(dst, dest, CV_HSV2BGR);
	Scalar* colours = (Scalar*)malloc(sizeof(Scalar)*k);
	for (int i = 0; i < centers.rows; i++) {
		colours[i] = Scalar(centers.at<float>(i, 0), centers.at<float>(i, 1), centers.at<float>(i, 2));
		Scalar lower = Scalar(centers.at<float>(i, 0) - 10, centers.at<float>(i, 1) - 7, centers.at<float>(i, 2) - 1);
		Scalar upper = Scalar(centers.at<float>(i, 0) + 10, centers.at<float>(i, 1) + 7, centers.at<float>(i, 2) + 1);
		Mat temp(src.size(),src.type());
		inRange(dst, lower,upper, temp);  
		src.copyTo(temp, temp);
		imshow(SSTR(i) + "S", temp);
	}

	return dest;
}

Scalar averageColour(Mat frame, Mat mask) {
	Mat masked;
	frame.copyTo(masked, mask);
	Scalar averageColour = sum(masked) / sum(mask);
	return averageColour;
}
int Utilities::find_boundingBoxes(Mat image, cv::Rect2d* boundingBoxes, int maxBoundingBoxes)
{
	Mat src_mat, gray_mat, canny_mat;
	Mat contour_mat;
	Mat bounding_mat;
	Mat out;

	contour_mat = image.clone();
	bounding_mat = image.clone();

	cvtColor(image, gray_mat, CV_BGR2GRAY);

	// apply canny edge detection
	Canny(image, canny_mat, 100, 150, 3, false);



	//Dialate edges to close small gaps so contour detection creates one object
	Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(3, 3));
	for (int i = 0; i < 10; i++) {
		dilate(canny_mat, canny_mat, structuringElement);
		erode(canny_mat, canny_mat, structuringElement);
	}

	//3. Find & process the contours
	//3.1 find contours on the edge image.
	vector< vector< cv::Point> > contours;
	findContours(canny_mat, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//findContours(canny_mat, contours, CV_RETR_TREE, CHAIN_APPROX_SIMPLE);

	//3.2 draw contours & property value on the source image.
	Mat con;
	image.copyTo(con);
	image.setTo(00);
	drawContours(con, contours, -1, Scalar(0, 0, 0));
	imshow("CCC", con);
	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;


	image.copyTo(out);
	std::sort(contours.begin(), contours.end(), compareContourSize);
	for (size_t i = 0; i< contours.size(); i++) // iterate through each contour.
	{
		// draw rectangle around the contour:
		cv::Rect tempBoundingBox = boundingRect(contours[i]);
		boundingBoxes[i] = tempBoundingBox;
	}

	return contours.size();
}


vector<vector<Point>> Utilities::find_contours(Mat image, int maxBoundingBoxes)
{
	Mat src_mat, gray_mat, canny_mat;
	Mat contour_mat;
	Mat bounding_mat;
	Mat out;

	contour_mat = image.clone();
	bounding_mat = image.clone();

	cvtColor(image, gray_mat, CV_BGR2GRAY);
	// apply canny edge detection
	Canny(image, canny_mat, 100, 150, 3, false);



	//Dialate edges to close small gaps so contour detection creates one object
	Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(5, 5));
	dilate(canny_mat, canny_mat, structuringElement);
	for (int i = 0; i < 10; i++) {
		dilate(canny_mat, canny_mat, structuringElement);
		erode(canny_mat, canny_mat, structuringElement);
	}

	//3. Find & process the contours
	//3.1 find contours on the edge image.
	vector< vector<Point> > contours;
	findContours(canny_mat, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//findContours(canny_mat, contours, CV_RETR_TREE, CHAIN_APPROX_SIMPLE);
	
	return contours;
}
Mat prevdiff;
UMat flow2;
Mat Utilities::get_object_contours(Mat frame,Mat prevFrame, int maxBoundingBoxes) {
	Mat diff;
	if(prevdiff.size().area() ==0)
		absdiff(frame, prevFrame, prevdiff);
	absdiff(frame, prevFrame, diff);
	vector<vector<Point>> contours = find_contours(diff, maxBoundingBoxes);
	Mat objects(frame.size(), frame.type());
	UMat uPrevFrame, uNextGray;
	Mat flowColour;
	
	drawContours(objects, contours, -1, Scalar(1, 1, 1));
	imshow("objs", objects);

	cvtColor(diff, uNextGray, CV_BGR2GRAY);
	cvtColor(prevdiff, uPrevFrame, CV_BGR2GRAY);
	//calcOpticalFlowFarneback(uPrevFrame, uNextGray, flow, 0.5, 5, 30,30, 5, 1.2, OPTFLOW_USE_INITIAL_FLOW);
	calcOpticalFlowFarneback(uPrevFrame, uNextGray, flow2, 0.5, 3, 15, 3, 5, 1.2, OPTFLOW_USE_INITIAL_FLOW);
	//TODO: try cuda::FarnebackOpticalFlow? 


	vector<Mat> channels(2);
	Mat angle, mag;
	// split img:
	split(flow2, channels);
	cartToPolar(channels[0], channels[1], mag, angle, true);

	Mat hsv[3];
	split(flowColour, hsv);

	hsv[0] = (angle);
	normalize(mag, hsv[1], 255, 255, NORM_MINMAX);
	normalize(mag, hsv[2], 0, 255, NORM_MINMAX);
	merge(hsv, 3, flowColour);
	//hsv[1].setTo(254.9);
	merge(hsv, 3, flowColour);
	cvtColor(flowColour, flowColour, CV_HSV2BGR);
	imshow("Flow2", flowColour);
	
	multiply(flowColour,objects,objects,1.0,objects.type());
	

	objects = kNearestColours(objects, 6);
	contours=find_contours(objects, 100);

	TrackedObject** trackedObjects = (TrackedObject**)malloc(sizeof(TrackedObject*)*contours.size());
	for (int i = 0; i < contours.size(); i++) {
		rectangle(objects, boundingRect(contours[i]), Scalar(255, 255, 0));
		//trackedObjects[i] = new TrackedObject(frame, boundingRect(contours[i]),i);
		//trackedObjects[i]->update(frame);
		Mat mask(frame.size(), CV_8U);
		refineMask(frame, mask, boundingRect(contours[i]));
		Scalar averageCol = averageColour(frame, mask);
		//objects.setTo(averageCol, mask);
	}

	imshow("Coloured", objects);
	diff.copyTo(prevdiff);
	free(trackedObjects);
	return objects;
}

bool Utilities::isAngleBetween(double angle, double bound1, double bound2) {
	//n = (360 + (n % 360)) % 360;
	angle = fmod(360 + fmod(angle, 360), 360);
	//a = (3600000 + a) % 360;
	bound1 = fmod(3600000 + bound1, 360);
	//b = (3600000 + b) % 360;
	bound2 = fmod(3600000 + bound2, 360);

	if (bound1 < bound2)
		return bound1 <= angle && angle <= bound2;
	return bound1 <= angle || angle <= bound2;
}
