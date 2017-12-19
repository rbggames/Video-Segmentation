#include "stdafx.h"
#include "TrackedObject.h"


TrackedObject::TrackedObject(Mat frame, Rect2d boundingBox_,int id_)
{
	numFramesForLastMaskRefinement = 0;
	id = id_;

	updateTracker(frame, boundingBox_);
}


TrackedObject::~TrackedObject()
{
}

void TrackedObject::updateTracker(Mat frame, Rect2d boundingBox_) {
	boundingBox = boundingBox_;
	position = boundingBox.tl();
	positionIndex = 0;
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

	if (position.x > 0 && position.y >0 && boundingBox.width + position.x < object.cols && boundingBox.height + position.y < object.rows)
		Mat(object, Rect(position.x, position.y, boundingBox.width, boundingBox.height)).copyTo(savedObject);
}

bool TrackedObject::update(Mat frame)
{
	//Update tracker
	bool ok = tracker->update(frame, boundingBox);
	double alpha = 0.4;

	if (ok) {
		// Update Positions and motion vector (rolling average)
		previousPosition[positionIndex] = position;
		position = boundingBox.tl();
		if (motionVector[0] != 0) {
			motionVector.val[0] = alpha*(position.x - previousPosition[positionIndex].x) + (1 - alpha)*motionVector.val[0];
		}else {
			//Initialise
			motionVector.val[0] = (position.x - previousPosition[positionIndex].x);
		}
		if(motionVector[1] != 0){
			motionVector.val[1] = alpha*(position.y - previousPosition[positionIndex].y) +(1 - alpha)*motionVector.val[1];
		}
		else {
			//Initialise
			motionVector.val[1] = (position.y - previousPosition[positionIndex].y);
		}
		positionIndex = (++positionIndex) % MAX_POSITIONS_TO_REMEMBER;
	}
	return ok;
}

void TrackedObject::drawSegment(Mat frame,bool isOverlap, Mat outputFrame)
{
	//Clear Object
	object.setTo(Scalar(180, 180, 180));
	if (!isOverlap) {
		//Display Tracked object
		refineMask(frame, mask, boundingBox);
		frame.copyTo(object, mask);

		if(position.x > 0 && position.y >0 && boundingBox.width + position.x < object.cols && boundingBox.height + position.y < object.rows)
			Mat(object, Rect(position.x, position.y, boundingBox.width, boundingBox.height)).copyTo(savedObject);
		

		putText(object, "Motion : mx=" + SSTR(motionVector.val[0]) + " my=" + SSTR(motionVector.val[1]), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);

		imshow("Tracked " + SSTR(int(id)), object);
		//imshow("Mask " + SSTR(int(id)), mask);
		//imshow("Saved " + SSTR(int(id)), savedObject);

		//Draw bounding box on frame
		rectangle(outputFrame, boundingBox, Scalar(255, 0, 0), 2, 1);
	}
	else {
		//Object Lost or obscured try to predict
		//1) Predict new position
		position.x += motionVector.val[0];
		position.y += motionVector.val[1];

		//2) Place object
		int height = boundingBox.height;
		int width = boundingBox.width;
		Point2f src[3] = { Point2f(0, 0), Point2f(0,savedObject.rows - 1),Point2f(savedObject.cols - 1,savedObject.rows - 1) };
		Point2f dst[3] = { Point2f(position.x, position.y), 
			Point2f(position.x, savedObject.rows - 1 + position.y),
			Point2f(savedObject.cols - 1 + position.x, savedObject.rows - 1 + position.y) };


		//imshow("Saved " + SSTR(int(id)), savedObject);
		Mat translationMat = getAffineTransform(src, dst);
		warpAffine(savedObject, object, translationMat, object.size(), 1, BORDER_TRANSPARENT);

		//move mask
		Point2f src2[3] = { Point2f(0, 0), Point2f(0,100),Point2f(100,0) };
		Point2f dst2[3] = { Point2f(motionVector[0], motionVector[1]),
			Point2f(motionVector[0], motionVector[1]+100),
			Point2f(motionVector[0]+100, motionVector[1]), };
		translationMat = getAffineTransform(src2, dst2);
		warpAffine(mask, mask, translationMat, object.size(), 1);

		putText(object, "Motion : mx=" + SSTR(motionVector.val[0]) + " my=" + SSTR(motionVector.val[1]), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);
		putText(object, "Predicting", Point(100, 150), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);

		imshow("Tracked " + SSTR(int(id)), object);
	}
}

void TrackedObject::refineMask(Mat frame, Mat mask, Rect2d boundingBox) {
	//Get Edges
	Mat cannyFrame;
	Canny(frame, cannyFrame, 100, 200);
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

bool TrackedObject::boundingBoxOverlap(TrackedObject object)
{
	if (boundingBox.br().x < object.boundingBox.tl().x) return false; // this is to the left of object
	if (boundingBox.tl().x > object.boundingBox.br().x) return false; // this is to the right of object
	if (boundingBox.br().y <  object.boundingBox.tl().y) return false; // this is above object
	if (boundingBox.tl().y > object.boundingBox.br().y) return false; // this is below object
	return true; // bounding boxes overlap
}

Mat TrackedObject::getObjectMat()
{
	return object;
}
