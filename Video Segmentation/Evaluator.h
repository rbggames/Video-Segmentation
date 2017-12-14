#pragma once
#include "Shape.h"
#include "TrackedObject.h"

class Evaluator
{
public:
	Evaluator();
	~Evaluator();


	static void evaluateSegments(Shape** actualShapes, int numShapes, TrackedObject** trackedShapes, int numTrackedShapes);
	static double evaluateSegmentsSimalarity(Shape* actualShape, TrackedObject* object);
	static double evaluateSegmentsSimalarity(Mat actualShape, Mat object,Rect2d roi);
};

