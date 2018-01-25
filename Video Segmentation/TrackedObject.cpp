#include "stdafx.h"
#include "TrackedObject.h"
#include "Utilities.h"

TrackedObject::TrackedObject(Mat frame, Rect2d boundingBox_,int id_)
{
	numFramesForLastMaskRefinement = 0;
	id = id_;
	boundingBox = boundingBox_;

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

	object = Mat(frame.size(), CV_8UC3);
	positionIndex = 0;
	updateTracker(frame, boundingBox_);
	previousPosition[positionIndex].x = position.x;
	previousPosition[positionIndex].y = position.y;
}


TrackedObject::~TrackedObject()
{
}

//TODO:: REMOVE/MOVE to utilities
int find_contour(Mat image, cv::Rect2d* boundingBoxes, int maxBoundingBoxes)
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

	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;


	image.copyTo(out);
	for (size_t i = 0; i< contours.size(); i++) // iterate through each contour.
	{
		// draw rectangle around the contour:
		cv::Rect tempBoundingBox = boundingRect(contours[i]);
		//if (i < maxBoundingBoxes)
		boundingBoxes[i] = tempBoundingBox;
		cv::rectangle(out, tempBoundingBox, cv::Scalar(255, 0, 255)); // if you want read and "image" is color image, use cv::Scalar(0,0,255) instead

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

	return contours.size();
}


void TrackedObject::updateTracker(Mat frame, Rect2d boundingBox_) {
	Rect2d oldBoundingBox = boundingBox;
	int xOffset, yOffset;
	boundingBox = boundingBox_|boundingBox;

	int oldx = boundingBox.x;
	int oldy = boundingBox.y;

	object = Mat(frame.size(), CV_8UC3);
	mask = Mat(frame.size(), CV_8U, Scalar(0));
	Rect2d newBoundingBoxes[1];
	Rect2d wholeFrame(0, 0, frame.size().width, frame.size().height);
	int expandingFactor = 5;
	Rect2d boundingToFindObject = Rect2d(boundingBox.x- expandingFactor, boundingBox.y-expandingFactor, 
		boundingBox.width+ expandingFactor*2, boundingBox.height+ expandingFactor*2) & wholeFrame;//Cliped to frame size

	Mat potentalObject(boundingToFindObject.height+10, boundingToFindObject.width+10, frame.type());
	potentalObject.setTo(0);
	frame(boundingToFindObject).copyTo(potentalObject(Rect2d(5, 5, boundingToFindObject.width, boundingToFindObject.height)));
	//destroyWindow("P" + SSTR(id));
	imshow("P"+SSTR(id), potentalObject);
	
	//find_contour(frame(boundingToFindObject), newBoundingBoxes, 1);
	Utilities::find_boundingBoxes(frame(boundingToFindObject), newBoundingBoxes, 1);
	
	boundingBox.x = newBoundingBoxes[0].x + boundingBox.x - expandingFactor ;
	boundingBox.y = newBoundingBoxes[0].y + boundingBox.y - expandingFactor ;
	boundingBox.width = newBoundingBoxes[0].width;
	boundingBox.height = newBoundingBoxes[0].height;

	//boundingBox = boundingBox.area() < oldBoundingBox.area() ? boundingToFindObject & wholeFrame : boundingBox;

	position = boundingBox.tl();
	xOffset = oldBoundingBox.tl().x - position.x;
	yOffset = oldBoundingBox.tl().y - position.y;

	//Update previous positions
	for (int i = 0; i < MAX_POSITIONS_TO_REMEMBER; i++) {
		previousPosition[i].x -= xOffset;
		previousPosition[i].y -= yOffset;
	}

	//tracker->clear();
	//tracker->init(frame, boundingBox);
	
	refineMask(frame, mask, boundingBox);
	frame.copyTo(object, mask);
	if (position.x > 0 && position.y >0 && boundingBox.width + position.x < object.cols && boundingBox.height + position.y < object.rows)
		Mat(object, Rect(position.x, position.y, boundingBox.width, boundingBox.height)).copyTo(savedObject);

	//TODO: Remove
	Mat temp;
	frame.copyTo(temp);
	rectangle(temp, boundingBox, Scalar(255, 0, 0));
	rectangle(temp, boundingToFindObject, Scalar(255, 255, 0));

	rectangle(temp, newBoundingBoxes[0], Scalar(255, 255, 255));

	imshow(SSTR(id), temp);
}

bool TrackedObject::update(Mat frame)
{
	//Update tracker
	bool ok = false;
	if(tracker)
		ok	= tracker->update(frame, boundingBox);
	double alpha = 0.1;

	if (ok) {
		// Update Positions and motion vector (rolling average)
		position = boundingBox.tl();
		if (motionVector.val[0] != 0) {
			motionVector.val[0] = alpha*(position.x - previousPosition[positionIndex].x) + (1 - alpha)*motionVector.val[0];
		}else {
			//Initialise
			motionVector.val[0] = (position.x - previousPosition[positionIndex].x);
		}
		if(motionVector.val[1] != 0){
			motionVector.val[1] = alpha*(position.y - previousPosition[positionIndex].y) +(1 - alpha)*motionVector.val[1];
		}
		else {
			//Initialise
			motionVector.val[1] = (position.y - previousPosition[positionIndex].y);
		}

		positionIndex = (++positionIndex) % MAX_POSITIONS_TO_REMEMBER;
		previousPosition[positionIndex].x = position.x;
		previousPosition[positionIndex].y = position.y;
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
		imshow("Mask " + SSTR(int(id)), mask);
		//imshow("Saved " + SSTR(int(id)), savedObject);

		//Draw bounding box on frame
		rectangle(outputFrame, boundingBox, Scalar(255, 0, 0), 2, 1);
	}
	else {
		//Check is a saved object
		if (savedObject.size().area() == 0) {
			return;
		}
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
	//Mask or bounding box undefined
	if (mask.size().area() == 0 || boundingBox.area() == 0)
		return;
	
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

Rect2d TrackedObject::getBoundingBox() {
	return boundingBox;
}

Vec2d TrackedObject::getMotionVector() {
	return motionVector;
}


