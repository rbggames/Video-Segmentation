#include "stdafx.h"
#include "TrackedObject.h"


TrackedObject::TrackedObject(Mat frame, Rect2d boundingBox_,int id_)
{
	boundingBox = boundingBox_;
	position = boundingBox.tl();
	positionIndex = 0;
	numFramesForLastMaskRefinement = 0;
	id = id_;

	// List of tracker types in OpenCV 3.2
	// NOTE : GOTURN implementation is buggy and does not work.
	string trackerTypes[6] = { "BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN" };
	// vector <string> trackerTypes(types, std::end(types));

	// Create a tracker
	string trackerType = trackerTypes[2];

#if (CV_MINOR_VERSION < 3)
		{
			tracker[i] = Tracker::create(trackerType);
		}
#else
		{
			if (trackerType == "BOOSTING")
				tracker = TrackerBoosting::create();
			if (trackerType == "MIL")
				tracker = TrackerMIL::create();
			if (trackerType == "KCF")
				tracker = TrackerKCF::create();
			if (trackerType == "TLD")
				tracker = TrackerTLD::create();
			if (trackerType == "MEDIANFLOW")
				tracker = TrackerMedianFlow::create();
			if (trackerType == "GOTURN")
				tracker = TrackerGOTURN::create();
		}
#endif

	tracker->init(frame, boundingBox);
	object = Mat(frame.size(), CV_8UC3);
	mask = Mat(frame.size(), CV_8U, Scalar(0));

	refineMask(frame, mask, boundingBox);
}


TrackedObject::~TrackedObject()
{
}

bool TrackedObject::update(Mat frame)
{
	//Update tracker
	bool ok = tracker->update(frame, boundingBox);

	if (ok) {
		// Update Positions and motion vector
		previousPosition[positionIndex] = position;
		position = boundingBox.tl();
		motionVector.val[0] = position.x - previousPosition[positionIndex].x;
		motionVector.val[1] = position.y - previousPosition[positionIndex].y;

		positionIndex = (++positionIndex) % MAX_POSITIONS_TO_REMEMBER;

		//Clear Object
		object.setTo(Scalar(180, 180, 180));
		//Display Tracked object
		refineMask(frame, mask, boundingBox);
		frame.copyTo(object, mask);

		putText(object, "Motion : mx=" + SSTR(motionVector.val[0]) + " my=" + SSTR(motionVector.val[1]), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);
		imshow("Tracked " + SSTR(int(id)), object);


		//Draw bounding box on frame
		rectangle(frame, boundingBox, Scalar(255, 0, 0), 2, 1);
	}
	else {
		//Object Lost or obscured try to predict
		//1) Predict new position
		position.x += motionVector.val[0];
		position.y += motionVector.val[1];

		//2) Place object
		int height = boundingBox.height;
		int width = boundingBox.width;
		Point2f src[3] = { Point2f(0, 0), Point2f(0, 1),Point2f(1,0) };
		Point2f dst[3] = { Point2f(motionVector.val[0], motionVector.val[1]), 
			Point2f(motionVector.val[0], motionVector.val[1]+1),
			Point2f(motionVector.val[0]+1,motionVector.val[1]) };
		Mat translationMat = getAffineTransform(src, dst);
		warpAffine(object, object, translationMat, object.size(), 1);

		putText(object, "Motion : mx=" + SSTR(motionVector.val[0]) + " my=" + SSTR(motionVector.val[1]), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);
		putText(object, "Predicting", Point(100, 150), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);

		imshow("Tracked " + SSTR(int(id)), object);
	}
	return ok;
}

void TrackedObject::refineMask(Mat frame, Mat mask, Rect2d boundingBox) {
	//Get Edges
	Mat cannyFrame;
	Canny(frame, cannyFrame, 100, 200);

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
