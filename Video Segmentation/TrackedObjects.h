#pragma once
#include "TrackedObject.h"


#define ACCEPTANCE_PROBABILITY 0.9
class TrackedObjects
{
public:
	TrackedObjects(Mat frame);
	~TrackedObjects();

	TrackedObject * getTrackedObject(int i);

	int find(Mat frame, Rect2d * objectBoundingBoxes, int objectsFound, int indexStart);

	int find(Mat frame);

	int find_contour(Mat image, cv::Rect2d * boundingBoxes, int maxBoundingBoxes);

	int findObjects(Mat frame, TrackedObject ** trackedObjects, int numObjects, TrackedObject ** newTrackedObjects);

	void update(Mat frame, Mat outputFrame);

	const int maxObjects = 5;
	int numObjects;
	RNG rng;
private:
	TrackedObject** trackedObjects;
	Rect2d objectBoundingBoxes[3000];//TODO CHANGE
	Mat prevFrame;

};

