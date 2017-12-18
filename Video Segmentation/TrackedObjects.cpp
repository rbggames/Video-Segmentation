#include "stdafx.h"
#include "TrackedObjects.h"
#include "Evaluator.h"



TrackedObjects::TrackedObjects()
{
	trackedObjects = (TrackedObject**)malloc(maxObjects * sizeof(TrackedObject*));
}


TrackedObjects::~TrackedObjects()
{
}

TrackedObject* TrackedObjects::getTrackedObject(int i) {
	return trackedObjects[i];
}

int TrackedObjects::find(Mat frame, Rect2d* objectBoundingBoxes) {
	int objectsFound = find_contour(frame, objectBoundingBoxes, maxObjects * 1000);
	trackedObjects = (TrackedObject**)malloc(maxObjects * sizeof(TrackedObject*));
	// Display bounding box.
	int i = 0, j = 0;
	while (i < maxObjects && j < objectsFound) {
		if (objectBoundingBoxes[j].area() > 12) {
			//rectangle(frame, objectBoundingBoxes[i], Scalar(255, 0, 0), 2, 1);
			trackedObjects[i] = new TrackedObject(frame, objectBoundingBoxes[j], i);
			i++;
		}
		j++;
	}
	numObjects = i;
	return numObjects;
}

int TrackedObjects::find(Mat frame)
{
	int objectsFound = find_contour(frame, objectBoundingBoxes, maxObjects * 1000);
	return find(frame,objectBoundingBoxes);
}

int TrackedObjects::find_contour(Mat image, cv::Rect2d* boundingBoxes, int maxBoundingBoxes)
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

	//TODO: Remove
	Mat drawing = Mat::zeros(canny_mat.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(drawing, contours, i, color, 2, 8);
	}
	imshow("Contours", drawing);
	//3.2 draw contours & property value on the source image.

	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;


	image.copyTo(out);
	for (size_t i = 0; i< contours.size(); i++) // iterate through each contour.
	{
		// draw rectangle around the contour:
		cv::Rect boundingBox = boundingRect(contours[i]);
		//if (i < maxBoundingBoxes)
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


int TrackedObjects::findObjects(Mat frame, TrackedObject** trackedObjects, int numObjects, TrackedObject** newTrackedObjects) {//TODO: think about freeing newTrackedObjects
	cv::Rect2d objectBoundingBoxes[1000];//TODO: CHANGE
	int objectsFound = find_contour(frame, objectBoundingBoxes, maxObjects * 1000);

	trackedObjects = (TrackedObject**)malloc(maxObjects * sizeof(TrackedObject*));
	// Display bounding box.
	int i = 0, j = 0, k = 0;
	while (i < maxObjects && j < objectsFound) {
		if (objectBoundingBoxes[j].area() > 12) {
			//rectangle(frame, objectBoundingBoxes[i], Scalar(255, 0, 0), 2, 1);

			double bestMatchValue = -1;
			int bestMatchIndex = -1;

			for (int j = 0; j < numObjects; j++) {
				double similarity = Evaluator::evaluateSegmentsSimalarity(frame, trackedObjects[j]->getObjectMat(), objectBoundingBoxes[j]);
				if (bestMatchValue < similarity) {
					bestMatchIndex = j;
					bestMatchValue = similarity;
				}
			}

			if (bestMatchValue > ACCEPTANCE_PROBABILITY) {//TODO: Want to check is not already added
				newTrackedObjects[k] = trackedObjects[bestMatchIndex];
				k++;
			}
			else {
				trackedObjects[k] = new TrackedObject(frame, objectBoundingBoxes[j], i);
				k++;
			}


			i++;
		}
		j++;
	}
}


void TrackedObjects::update(Mat frame, Mat outputFrame) {
	for (int i = 0; i < numObjects; i++) {
		bool ok = trackedObjects[i]->update(frame);
		if (!ok)
		{
			// Tracking failure detected.
			putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
		}
	}
	for (int i = 0; i < numObjects; i++) {
		bool isOverlap = false;
		for (int j = 0; j < numObjects; j++) {
			if (i != j) {
				isOverlap = trackedObjects[i]->boundingBoxOverlap(*trackedObjects[j]);
				break;
			}
		}
		trackedObjects[i]->drawSegment(frame, isOverlap, outputFrame);
	}
}
