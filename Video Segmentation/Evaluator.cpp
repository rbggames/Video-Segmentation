#include "stdafx.h"
#include "Evaluator.h"
#define VideoWidth 600
#define VideoHeight 600

Evaluator::Evaluator()
{
}


Evaluator::~Evaluator()
{
}

void Evaluator::evaluateSegments(Shape** actualShapes, int numShapes, TrackedObjects trackedShapes)
{
	int numTrackedShapes = trackedShapes.getNumTrackedObjects();
	for (int i = 0; i < numShapes; i++) {
		int bestMatchIndex=-1;
		double bestMatchValue=-1;
		for (int j = 0; j < numTrackedShapes; j++) {
			double similarity = evaluateSegmentsSimalarity(actualShapes[i], trackedShapes.getTrackedObject(j));
			if (bestMatchValue < similarity) {
				bestMatchIndex = j;
				bestMatchValue = similarity;
			}
		}
		std::cout << "Similarity Actual: " + SSTR(i) + " Tracked: " + SSTR(bestMatchIndex) + "is" + SSTR(bestMatchValue) + "\n";
	}
	std::cout << "\n";
}


double Evaluator::evaluateSegmentsSimalarity(Mat actualShape, Mat object,Rect2d roi) {
	Mat output, grey, shape, shapeObject, finalOutput(object.size(), object.type());
	Mat mask, preMask;

	shape = actualShape;
	shapeObject = object;
	absdiff(shape, shapeObject, output);
	cvtColor(output, grey, CV_BGR2GRAY);
	cvtColor(actualShape, preMask, CV_BGR2GRAY);

	threshold(preMask, mask, 0, 255, THRESH_BINARY);
	grey.copyTo(finalOutput, mask);

	return 1 - sum(finalOutput(roi))[0] / (sum(mask)[0]);
	
}

double Evaluator::evaluateSegmentsSimalarity(Shape* actualShape, TrackedObject* object)
{
	Mat output,grey,shape,shapeObject,finalOutput(object->getObjectMat().size(),object->getObjectMat().type());
	Mat mask,preMask;
	/*matchTemplate(object->getObjectMat(), actualShape->getShapeFrameMat(), output, TM_CCOEFF_NORMED);
	double min, max;
	minMaxLoc(output,&min,&max);
	return max;*/
	shape = actualShape->getShapeFrameMat();
	shapeObject = object->getObjectMat();
	absdiff(shape, shapeObject,output);
	cvtColor(output, grey, CV_BGR2GRAY);
	cvtColor(actualShape->getShapeFrameMat(), preMask,CV_BGR2GRAY);

	threshold(preMask, mask, 0, 255, THRESH_BINARY);
	grey.copyTo(finalOutput,mask) ;
	
	int xPos = actualShape->getX() < 0 ? 0 : actualShape->getX();
	int yPos = actualShape->getY() < 0 ? 0 : actualShape->getY();
	int width = actualShape->getX() + actualShape->getWidth() < VideoWidth ? actualShape->getWidth() : VideoWidth - actualShape->getX();
	int height = actualShape->getY() + actualShape->getHeight() < VideoHeight ? actualShape->getHeight() : VideoWidth - actualShape->getY();
	if (width > 0 && height > 0) {
		return 1 - sum(finalOutput(Rect2d(xPos, yPos, width, height)))[0] / (sum(mask)[0]);
	}
	else {
		return 0;
	}
}
