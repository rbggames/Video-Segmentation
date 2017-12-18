#pragma once
#include "Shape.h"
#include "TrackedObject.h"
#include "TrackedObjects.h"

class Evaluator
{
public:
	Evaluator();
	~Evaluator();


	static void evaluateSegments(Shape** actualShapes, int numShapes, TrackedObjects trackedShapes, int numTrackedShapes);
	static double evaluateSegmentsSimalarity(Shape* actualShape, TrackedObject* object);
	static double evaluateSegmentsSimalarity(Mat actualShape, Mat object,Rect2d roi);
};

