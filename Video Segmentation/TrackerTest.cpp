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

using namespace cv;
using namespace std;


const int maxObjects = 10;
#define ACCEPTANCE_PROBABILITY 0.9
RNG rng(12345);


int main(int argc, char **argv)
{
	ocl::setUseOpenCL(true);
	RNG rng(12345);
	//TrackedObject** trackedObjects;
	TrackedObjects trackedObjects;

	// Read first frame
	Mat frame(600, 600, CV_8UC3), outputFrame, prevframe;

	VideoSource video;
	video.getFrame(frame);
	//VideoCapture cap(0);
	//cap.read(frame);


	BackgroundExtractor background(frame, 10);
	for (int i = 0; i < 10; i++) {
		video.getFrame(frame);
	}
	background.update(frame);

	imshow("Tracking", frame);

	// Display bounding box.
	//cap.read(frame);
	int numObjects = trackedObjects.find(frame);
	
	int num = 0;
	//while(cap.read(frame))
	cvtColor(frame, prevframe, CV_BGR2GRAY);
	while (video.getFrame(frame))
	{
		UMat flow;
		Mat nextGrey, flowPolar, out(frame.rows,frame.cols, CV_8UC3);
		cvtColor(frame, nextGrey, CV_BGR2GRAY);
		calcOpticalFlowFarneback(prevframe, nextGrey, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

		vector<Mat> channels(2);
		Mat angle, mag;
		// split img:
		split(flow, channels);
		cartToPolar(channels[0],channels[1],mag,angle);
		
		Mat h(frame.size(), CV_8UC1), s(frame.size(), CV_8UC1), v(frame.size(), CV_8UC1);
		Mat hsv[3];
		hsv[0] = h; hsv[1] = s; hsv[2] = v;
		split(out, hsv);

		hsv[0] = (angle * 180 / 3.141592654 / 2);
		normalize(mag, hsv[1], 255, 255, NORM_MINMAX);
		normalize(mag, hsv[2], 0, 255, NORM_MINMAX);
		merge(hsv, 3, out);
		//hsv[1].setTo(254.9);
		merge(hsv, 3, out);
		cvtColor(out, out, CV_HSV2BGR);
		imshow("Flow", out);

		// Start timer
		double timer = (double)getTickCount();

		frame.copyTo(outputFrame);

		//TODO: Could be got from elsewhere
		Mat cannyFrame;
		Canny(frame, cannyFrame, 30, 200);

		trackedObjects.update(frame, outputFrame);
		background.update(frame,trackedObjects,numObjects);


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
		Evaluator::evaluateSegments(shapes, numShapes, trackedObjects, numObjects);
		cvtColor(frame, prevframe, CV_BGR2GRAY);
	}
}