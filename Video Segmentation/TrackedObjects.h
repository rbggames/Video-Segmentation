#pragma once
#include "TrackedObject.h"


#define ACCEPTANCE_PROBABILITY 0.9
class TrackedObjects
{
public:
	TrackedObjects(Mat frame);
	~TrackedObjects();

	TrackedObject * getTrackedObject(int i);

	int find(Mat frame);
	int consolidateObjects();

	int find_contour(Mat image, cv::Rect2d * boundingBoxes, int maxBoundingBoxes);

	void update(Mat frame, Mat outputFrame);

	int TrackedObjects::getNumTrackedObjects();
	const int maxObjects = 50;
	RNG rng;
private:
	TrackedObject** trackedObjects;
	std::vector<TrackedObject> trackedObjectList;
	Rect2d objectBoundingBoxes[3000];//TODO CHANGE
	Mat prevFrame;
	double smallestObjectArea;
};

