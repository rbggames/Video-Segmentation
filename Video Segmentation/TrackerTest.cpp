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
int currentObjectId = 0;

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
	for (int i = 0; i < 20; i++) {
		video.getFrame(frame);
		background.update(frame);
	}
	for (int i = 0; i < 25; i++) {
		video.getFrame(frame);
		background.update(frame, trackedObjects, 0);
	}

	imshow("Tracking", frame);

	// Display bounding box.
	//cap.read(frame);
	//while(cap.read(frame))
	Mat prevFrame;
	frame.copyTo(prevFrame);
	while (video.getFrame(frame))
	{
		//Utilities::get_object_contours(frame, prevFrame, 1000);

		frame.copyTo(prevFrame);
		imshow("framea", frame);
		imshow("frameb", prevFrame);

		// Start timer
		double timer = (double)getTickCount();
		Mat forground = background.getForground();
		if (num % 10 == 0) {
			numObjects = trackedObjects.find(forground);
		}
		frame.copyTo(outputFrame);

		//TODO: Could be got from elsewhere
		//Mat cannyFrame;
		//Canny(frame, cannyFrame, 30, 200);

		trackedObjects.update(forground, outputFrame);
		background.update(frame,trackedObjects,numObjects);

		if (num % 30 == 0) {
			numObjects = trackedObjects.consolidateObjects();
		}
		num++;
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
				//Cycle Through Tracked Objects
				if (k == 'n') {
					currentObjectId = (currentObjectId + 1) % (numObjects > 0 ? numObjects : 1);
				}
				TrackedObject* curObject = trackedObjects.getTrackedObject(currentObjectId);
				if (curObject) {
					Mat obj;
					curObject->getObjectMat().copyTo(obj);
					rectangle(obj, curObject->getTrackerBoundingBox(), Scalar(0, 0, 0));
					rectangle(obj, curObject->getBoundingBox(), Scalar(255, 0, 0));
					putText(obj, "Tracked (" + SSTR(currentObjectId) + ") :" + SSTR(curObject->getId()), Point(10, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
					imshow("Object", obj);
				}
			}
		}
		//Cycle Through Tracked Objects
		if (k == 'n') {
			currentObjectId = (currentObjectId + 1) % (numObjects > 0 ? numObjects : 1);
		}
		TrackedObject* curObject = trackedObjects.getTrackedObject(currentObjectId);
		if (curObject) {
			Mat obj;
			curObject->getObjectMat().copyTo(obj);
			rectangle(obj, curObject->getTrackerBoundingBox(), Scalar(0, 0, 0));
			rectangle(obj, curObject->getBoundingBox(), Scalar(255, 0, 0));
			putText(obj, "Tracked ("+ SSTR(currentObjectId) +") :"+ SSTR(curObject->getId()), Point(10, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
			imshow("Object", obj);
		}

		//Evaluation
		int numShapes;
		Shape** shapes = video.getShapes(&numShapes);
		Evaluator::evaluateSegments(shapes, numShapes, trackedObjects);
	}
}