#include "stdafx.h"
#include "VideoSource.h"

#include "ShapeHook.h"
#include "ShapeSquare.h"

VideoSource::VideoSource()
{
	int w = 60;
	shapes[0] = new ShapeHook(200, 300, w,"Images\\bright.jpg");
	shapes[0]->setMotionVector(1.0, 0.0);
	shapes[1] = new ShapeHook(400, 330, w, "Images\\wood.jpg");
	shapes[1]->setMotionVector(-1.0, 0.0);

	shapes[2] = new ShapeSquare(200, 300, w, "Images\\lava.jpg");
	shapes[2]->setMotionVector(1.0, 1.0);
	//shapes[3] = new ShapeHook(400, 530, w, Scalar(0, 255, 0));
	//shapes[3]->setMotionVector(-1.0, 0.0);

	//shapes[4] = new ShapeHook(200, 20, w);
	//shapes[4]->setMotionVector(1.0, 0.0);
	//shapes[5] = new ShapeHook(400, 30, w, Scalar(0, 255, 0));
	//shapes[5]->setMotionVector(-1.0, 0.0);

	numShapes = 3;
}


VideoSource::~VideoSource()
{
}

bool VideoSource::getFrame(Mat frame)
{
	Mat backgroundImg;
	backgroundImg = imread("Images\\background.jpg");
	backgroundImg(Rect2d(0,0, frame.cols, frame.rows)).copyTo(frame);
	//frame.setTo(Scalar(0, 0, 0));
	for (int i = 0; i < numShapes; i++)
		shapes[i]->updateDraw(frame);
	return true;
}

Shape** VideoSource::getShapes(int * numberShapes)
{
	*numberShapes = numShapes;
	return shapes;
}
