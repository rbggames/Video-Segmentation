#include "stdafx.h"
#include "TrackedObject.h"
#include "Utilities.h"


TrackedObject::TrackedObject(Mat frame, Rect2d boundingBox_,double angle, int id_)
{
	markedToRemove = false;
	predicting = false;
	trackingReset = true;
	objectBoundingBox = Rect2d(boundingBox_);
	trackerBoundingBox = Rect2d(objectBoundingBox);
	position = objectBoundingBox.tl();
	positionIndex = 0;
	numFramesForLastMaskRefinement = 0;
	id = id_;

	//imshow(SSTR(id_), frame(boundingBox_));

	
	motionAngle = angle;








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

	tracker->init(frame, objectBoundingBox);
	object = Mat(frame.size(), CV_8UC3);
	mask = Mat(frame.size(), CV_8U, Scalar(0));

	refineMask(frame, mask, objectBoundingBox);

	if (position.x > 0 && position.y >0 && objectBoundingBox.width + position.x < object.cols && objectBoundingBox.height + position.y < object.rows)
		frame(Rect(position.x, position.y, objectBoundingBox.width, objectBoundingBox.height)).copyTo(savedObject);
	for (int i = 0; i < MAX_POSITIONS_TO_REMEMBER; i++) {
		previousPosition[i].x = trackerBoundingBox.x;
		previousPosition[i].y = trackerBoundingBox.y;
	}
}


TrackedObject::~TrackedObject()
{
}

void TrackedObject::updateTracker(Mat frame, Rect2d boundingBox_) {
	Rect2d oldBoundingBox = objectBoundingBox;
	int xOffset, yOffset;
	objectBoundingBox = boundingBox_ | objectBoundingBox;

	int oldx = objectBoundingBox.x;
	int oldy = objectBoundingBox.y;

	object = Mat(frame.size(), CV_8UC3);
	mask = Mat(frame.size(), CV_8U, Scalar(0));
	Rect2d newBoundingBoxes[10];//TODO: FIX
	Rect2d wholeFrame(0, 0, frame.size().width, frame.size().height);
	int expandingFactor = 5;
	Rect2d boundingToFindObject = Rect2d(objectBoundingBox.x - expandingFactor, objectBoundingBox.y - expandingFactor,
		objectBoundingBox.width + expandingFactor * 2, objectBoundingBox.height + expandingFactor * 2) & wholeFrame;//Cliped to frame size

	Mat potentalObject(boundingToFindObject.height + expandingFactor*2, boundingToFindObject.width + expandingFactor*2, frame.type());
	potentalObject.setTo(0);
	frame(boundingToFindObject).copyTo(potentalObject(Rect2d(5, 5, boundingToFindObject.width, boundingToFindObject.height)));
	//destroyWindow("P" + SSTR(id));
	//imshow("P" + SSTR(id), potentalObject);

	//find_contour(frame(boundingToFindObject), newBoundingBoxes, 1);
	int number = Utilities::find_boundingBoxes(potentalObject, newBoundingBoxes, 1);

	Rect2d bestBoundingBox;
	Rect2d currentBoundingBox;
	double bestAreaOverlap = 0.0,newAreaOverlap;
	for (int i = 0; i < number; i++) {
		currentBoundingBox.x = newBoundingBoxes[i].x + objectBoundingBox.x - expandingFactor * 2;
		currentBoundingBox.y = newBoundingBoxes[i].y + objectBoundingBox.y - expandingFactor * 2;
		currentBoundingBox.width = newBoundingBoxes[i].width;
		currentBoundingBox.height = newBoundingBoxes[i].height;
		newAreaOverlap = (trackerBoundingBox & currentBoundingBox).area();
		if (bestAreaOverlap < newAreaOverlap) {
			bestAreaOverlap = newAreaOverlap;
			objectBoundingBox = currentBoundingBox;
		}
	}
	objectBoundingBox = objectBoundingBox | trackerBoundingBox;

	/*objectBoundingBox.x = bestBoundingBox.x + objectBoundingBox.x - expandingFactor*2;
	objectBoundingBox.y = bestBoundingBox.y + objectBoundingBox.y - expandingFactor*2;
	objectBoundingBox.width = bestBoundingBox.width;
	objectBoundingBox.height = bestBoundingBox.height;*/

	//boundingBox = boundingBox.area() < oldBoundingBox.area() ? boundingToFindObject & wholeFrame : boundingBox;

	//position = boundingBox.tl();
	xOffset = oldBoundingBox.tl().x - position.x;
	yOffset = oldBoundingBox.tl().y - position.y;

	//Update previous positions
	/*for (int i = 0; i < MAX_POSITIONS_TO_REMEMBER; i++) {
		previousPosition[i].x -= xOffset;
		previousPosition[i].y -= yOffset;
	}*/

	//tracker->clear();
	//tracker->init(frame, boundingBox);

	refineMask(frame, mask, objectBoundingBox);
	frame.copyTo(object, mask);
	//if (position.x > 0 && position.y >0 && objectBoundingBox.width + position.x < object.cols && objectBoundingBox.height + position.y < object.rows)
		//object(Rect(position.x, position.y, objectBoundingBox.width, objectBoundingBox.height)).copyTo(savedObject);

	//TODO: Remove
	Mat temp;
	frame.copyTo(temp);
	rectangle(temp, objectBoundingBox, Scalar(255, 0, 0));
	rectangle(temp, boundingToFindObject, Scalar(255, 255, 0));

	rectangle(temp, newBoundingBoxes[0], Scalar(255, 255, 255));

	//imshow(SSTR(id), temp);
}

bool TrackedObject::update(Mat frame)
{
	//Update tracker
	bool ok = false;
	if (tracker)
		ok = tracker->update(frame, trackerBoundingBox);
	double alpha = 0.1;

	if (ok) {
		// Update Positions and motion vector (rolling average)
		Point trackerPosition = trackerBoundingBox.tl();
		if (!trackingReset  && motionVector.val[0] != 0) {
			motionVector.val[0] = alpha*(trackerPosition.x - previousPosition[positionIndex].x) + (1 - alpha)*motionVector.val[0];
		}
		else {
			//Initialise
			if(motionVector.val[0] == 0)
				motionVector.val[0] = (trackerPosition.x - previousPosition[positionIndex].x);
		}

		if (!trackingReset && motionVector.val[0] != 0) {
			motionVector.val[1] = alpha*(trackerPosition.y - previousPosition[positionIndex].y) + (1 - alpha)*motionVector.val[1];
		}
		else {
			//Initialise
			motionVector.val[1] = (trackerPosition.y - previousPosition[positionIndex].y);
			trackingReset = false;
		}

		//Update objectBoundingBoxPosition
		objectBoundingBox.x += trackerPosition.x - previousPosition[positionIndex].x;
		objectBoundingBox.y += trackerPosition.y - previousPosition[positionIndex].y;
		position = objectBoundingBox.tl();

		positionIndex = (++positionIndex) % MAX_POSITIONS_TO_REMEMBER;;
		previousPosition[positionIndex].x = trackerPosition.x;
		previousPosition[positionIndex].y = trackerPosition.y;
		

		/*if (positionIndex == 0) {
			tracker->clear();
			if (objectBoundingBox.area() > 25) {
				tracker->init(frame, objectBoundingBox);
				previousPosition[positionIndex].x = objectBoundingBox.x;
				previousPosition[positionIndex].y = objectBoundingBox.y;
			}
			trackingReset = true;
		}*/
		
		if (motionVector.val[0] * motionVector.val[0] + motionVector.val[0] * motionVector.val[0] > 0.01) {
			markedToRemove = false;
		}
	}

	
	return ok;
}

void TrackedObject::drawSegment(Mat frame,bool isOverlap, Mat outputFrame)
{
	//Clear Object
	object.setTo(Scalar(180, 180, 180));
	if (!isOverlap) {//TODO:CHANGE
		//Display Tracked object
		predicting = false;
		refineMask(frame, mask, objectBoundingBox);
		frame.copyTo(object, mask);

		
		object(objectBoundingBox & Rect2d(0,0, frame.cols, frame.rows)).copyTo(savedObject);
		//imshow(SSTR(id), savedObject);

		putText(object, "Motion : mx=" + SSTR(motionVector.val[0]) + " my=" + SSTR(motionVector.val[1]), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);

		//imshow("Tracked " + SSTR(int(id)), object);
		//imshow("Mask " + SSTR(int(id)), mask);
		//imshow("Saved " + SSTR(int(id)), savedObject);

		//Draw bounding box on frame
		rectangle(outputFrame, objectBoundingBox, Scalar(255, 0, 0), 2, 1);
	} else {
		//Object Lost or obscured try to predict
		//0) Check we have a savedObject
		predicting = true; 
		if (savedObject.size().area() < 25) {
			//Cannot predict therefore kills itself
			objectBoundingBox.width = 0;
			objectBoundingBox.height = 0;
			return;
		}
		//imshow(SSTR(id)+"!!", savedObject);
		//1) Predict new position
		position.x += motionVector.val[0];
		position.y += motionVector.val[1];

		//2) Place object
		int height = objectBoundingBox.height;
		int width = objectBoundingBox.width;
		Point2f src[3] = { Point2f(0, 0), Point2f(0,savedObject.rows - 1),Point2f(savedObject.cols - 1,savedObject.rows - 1) };
		Point2f dst[3] = { Point2f(position.x, position.y), 
			Point2f(position.x, savedObject.rows - 1 + position.y),
			Point2f(savedObject.cols - 1 + position.x, savedObject.rows - 1 + position.y) };


		//imshow("Saved " + SSTR(int(id)), savedObject);
		Mat translationMat = getAffineTransform(src, dst);
		warpAffine(savedObject, object, translationMat, object.size(), 1, BORDER_TRANSPARENT);

		//move mask
		Point2f src2[3] = { Point2f(0, 0), Point2f(0,100),Point2f(100,0) };
		Point2f dst2[3] = { Point2f(motionVector.val[0], motionVector.val[1]),
			Point2f(motionVector.val[0], motionVector.val[1]+100),
			Point2f(motionVector.val[0]+100, motionVector.val[1]) };
		translationMat = getAffineTransform(src2, dst2);
		warpAffine(mask, mask, translationMat, object.size(), 1);

		putText(object, "Motion : mx=" + SSTR(motionVector.val[0]) + " my=" + SSTR(motionVector.val[1]), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);
		putText(object, "Predicting", Point(100, 150), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);
		
		//imshow("Tracked " + SSTR(int(id)), object);
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
	if (objectBoundingBox.br().x < object.objectBoundingBox.tl().x) return false; // this is to the left of object
	if (objectBoundingBox.tl().x > object.objectBoundingBox.br().x) return false; // this is to the right of object
	if (objectBoundingBox.br().y <  object.objectBoundingBox.tl().y) return false; // this is above object
	if (objectBoundingBox.tl().y > object.objectBoundingBox.br().y) return false; // this is below object
	return true; // bounding boxes overlap
}

Mat TrackedObject::getObjectMat()
{
	return object;
}

Rect2d TrackedObject::getTrackerBoundingBox()
{
	return trackerBoundingBox;
}

Rect2d TrackedObject::getBoundingBox()
{
	return objectBoundingBox;
}

Vec2d TrackedObject::getMotionVector()
{
	return motionVector;
}

double TrackedObject::getMotionAngle()
{
	if (motionVector.val[0]* motionVector.val[0] + motionVector.val[1] * motionVector.val[1] > 0.01) {
		motionAngle = cvFastArctan(motionVector.val[1],motionVector.val[0]);
	}
	return motionAngle;
}

int TrackedObject::getId()
{
	return id;
}
