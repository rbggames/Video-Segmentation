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
	Mat mask;

	TrackedObject(Mat frame, Rect2d boundingBox,int id_);
	~TrackedObject();

	bool update(Mat frame);
	void drawSegment(Mat frame,bool isOverlap,Mat outputFrame);
	bool boundingBoxOverlap(TrackedObject object);
private:
	int id;
	int positionIndex;
	int numFramesForLastMaskRefinement;
	Rect2d boundingBox;
	Point position;
	Point previousPosition[5];
	Vec2d motionVector;
	Mat object;
	Mat savedObject;
	Ptr<Tracker> tracker;

	void refineMask(Mat image, Mat mask, Rect2d boundingBox);
};

