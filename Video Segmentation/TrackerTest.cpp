#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "Shape.h"
#include "ShapeHook.h"
#include "TrackedObject.h"
#include "TrackedObjects.h"
#include "VideoSource.h"
#include "BackgroundExtractor.h"
#include "Evaluator.h"
#include <math.h>

#include "Utilities.h"

using namespace cv;
using namespace std;


const int maxObjects = 10;
#define ACCEPTANCE_PROBABILITY 0.9
RNG rng(12345);


int main(int argc, char **argv)
{
	ocl::setUseOpenCL(true);
	RNG rng(12345);

	// Read first frame
	Mat frame(600, 600, CV_8UC3), outputFrame;
	//Mat frame, outputFrame;
	//imread("Images//background.jpg");
	VideoSource video;
	video.getFrame(frame);

	int numObjects=0;
	int num = 0;
	//VideoCapture cap("video.avi");
	//cap.read(frame);


	TrackedObjects trackedObjects(frame);

	BackgroundExtractor background(frame, 10);
	for (int i = 0; i < 10; i++) {
		video.getFrame(frame);
		if (num % 5 == 0) {
			//numObjects = trackedObjects.find(frame);
		}
		//trackedObjects.update(frame, outputFrame);
		num++;
	}
	background.update(frame);

	imshow("Tracking", frame);

	// Display bounding box.
	//cap.read(frame);
	//while(cap.read(frame))
	Mat prevFrame;
	frame.copyTo(prevFrame);
	while (video.getFrame(frame))
	{
		Utilities::get_object_contours(frame, prevFrame, 1000);

		frame.copyTo(prevFrame);
		imshow("framea", frame);
		imshow("frameb", prevFrame);

		// Start timer
		double timer = (double)getTickCount();
		Mat forground = background.getForground();
		if (num % 1 == 0) {
			//numObjects = trackedObjects.find(frame);
		}
		num++;
		frame.copyTo(outputFrame);

		//TODO: Could be got from elsewhere
		Mat cannyFrame;
		Canny(frame, cannyFrame, 30, 200);

		//trackedObjects.update(frame, outputFrame);
		background.update(frame,trackedObjects,numObjects);
		//background.update(frame);

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

		//Evaluation
		int numShapes;
		Shape** shapes = video.getShapes(&numShapes);
		//Evaluator::evaluateSegments(shapes, numShapes, trackedObjects, numObjects);
	}
}