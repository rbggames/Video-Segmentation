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

	// Update Positions
	previousPosition[positionIndex] = position;
	positionIndex = (++positionIndex) % MAX_POSITIONS_TO_REMEMBER;
	position = boundingBox.tl();

	//Clear Object
	object.setTo(Scalar(0, 0, 0));
	//Display Tracked object
	refineMask(frame, mask, boundingBox);
	frame.copyTo(object, mask);
	imshow("Tracked " + SSTR(int(id)), object);

	//Draw bounding box on frame
	rectangle(frame, boundingBox, Scalar(255, 0, 0), 2, 1);
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
			if (!leftEdgeFound && cannyFrame.at<unsigned char>(y, x + xOffset) < thresh) {
				mask.at<unsigned char>(y, x + xOffset) = 0;
			}
			else {
				leftEdgeFound = true;
			}
			if (!rightEdgeFound && cannyFrame.at<unsigned char>(y, x + boundingBox.width - xOffset) < thresh) {
				mask.at<unsigned char>(y, x + boundingBox.width - xOffset) = 0;
			}
			else {
				rightEdgeFound = true;
			}
		}
	}
}
