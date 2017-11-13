#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "Shape.h"
#include "ShapeHook.h"
#include "TrackedObject.h"
#include "VideoSource.h"

using namespace cv;
using namespace std;


const int maxObjects = 10;

int find_contour(Mat image,cv::Rect2d* boundingBoxes, int maxBoundingBoxes)
{
	Mat src_mat, gray_mat, canny_mat;
	Mat contour_mat;
	Mat bounding_mat;
	Mat out;

	contour_mat = image.clone();
	bounding_mat = image.clone();

	cvtColor(image, gray_mat, CV_BGR2GRAY);

	// apply canny edge detection
	Canny(image, canny_mat, 30, 128, 3, false);

	//Dialate edges to close small gaps so contour detection creates one object
	Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(4, 4));
	dilate(canny_mat, canny_mat, structuringElement);

	//3. Find & process the contours
	//3.1 find contours on the edge image.
	vector< vector< cv::Point> > contours;
	findContours(canny_mat, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	//3.2 draw contours & property value on the source image.

	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;


	image.copyTo(out);
	for (size_t i = 0; i< contours.size(); i++) // iterate through each contour.
	{
		// draw rectangle around the contour:
		cv::Rect boundingBox = boundingRect(contours[i]);
		if (i < maxBoundingBoxes)
			boundingBoxes[i] = boundingBox;
		cv::rectangle(out, boundingBox, cv::Scalar(255, 0, 255)); // if you want read and "image" is color image, use cv::Scalar(0,0,255) instead

																  // you aren't using the largest contour at all? no need to compute it...
																  /*
																  double area = contourArea(contours[i]);  //  Find the area of contour

																  if (area > largest_area)
																  {
																  largest_area = area;
																  largest_contour_index = i;               //Store the index of largest contour
																  bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
																  }
																  */
	}

	//drawContours(image, contours, largest_contour_index, Scalar(0, 255, 0), 2);

	imshow("Bounding ", out);

	return contours.size();
}

int main(int argc, char **argv)
{
	RNG rng(12345);
	TrackedObject** trackedObjects;

	// Read first frame
	Mat frame(600,600, CV_8UC3);

	VideoSource video;
	
	video.getFrame(frame);

	cv::Rect2d objectBoundingBoxes[maxObjects];
	const int numObjects = min(maxObjects, find_contour(frame, objectBoundingBoxes,maxObjects));
	
	trackedObjects = (TrackedObject**)malloc(numObjects * sizeof(TrackedObject*));
	// Display bounding box.
	for (int i=0; i < numObjects; i++) {
		rectangle(frame, objectBoundingBoxes[i], Scalar(255, 0, 0), 2, 1);
		trackedObjects[i] = new TrackedObject(frame,objectBoundingBoxes[i],i);
	}
	imshow("Tracking", frame);

	

	while (video.getFrame(frame))
	{
		// Start timer
		double timer = (double)getTickCount();
		
		// Update the tracking result
		bool* ok = (bool*) malloc(sizeof(bool)*numObjects);

		//TODO: Could be got from elsewhere
		Mat cannyFrame;
		Canny(frame, cannyFrame, 30, 200);

		for (int i = 0; i < numObjects; i++) {
			ok[i] = trackedObjects[i]->update(frame);
			if (!ok[i])
			{
				// Tracking failure detected.
				putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
			}
		}
		free(ok);

		// Calculate Frames per second (FPS)
		float fps = getTickFrequency() / ((double)getTickCount() - timer);
		
		// Display FPS on frame
		putText(frame, "FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

		// Display frame.
		imshow("Tracking", frame);

		// Exit if ESC pressed.
		int k = waitKey(1);
		if (k == 27)
		{
			break;
		}
		//Pause
		if (k == 'p') {
			k = 0;
			while (k != 'p') {
				k = waitKey(5);
			}
		}

	}
}