#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#define MAX_POSITIONS_TO_REMEMBER 5
#define MAX_FRAMES_BETWEEN_MASK_REFINEMENT 20

using namespace cv;
using namespace std;

class TrackedObject
{
public:
	TrackedObject(Mat frame, Rect2d boundingBox,int id_);
	~TrackedObject();

	bool update(Mat frame);
private:
	int id;
	int positionIndex;
	int numFramesForLastMaskRefinement;
	Rect2d boundingBox;
	Point position;
	Point previousPosition[5];
	Vec2d motionVector;
	Mat mask;
	Mat object;
	Ptr<Tracker> tracker;

	void refineMask(Mat image, Mat mask, Rect2d boundingBox);
};

