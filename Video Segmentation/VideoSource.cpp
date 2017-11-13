#include "stdafx.h"
#include "VideoSource.h"


VideoSource::VideoSource()
{
	int w = 60;
	shapes[0] = new ShapeHook(200, 300, w);
	shapes[0]->setMotionVector(1.0, 0.0);
	shapes[1] = new ShapeHook(400, 330, w, Scalar(0,255,0));
	shapes[1]->setMotionVector(-1.0, 0.0);

	shapes[2] = new ShapeHook(200, 500, w);
	shapes[2]->setMotionVector(1.0, 0.0);
	shapes[3] = new ShapeHook(400, 530, w, Scalar(0, 255, 0));
	shapes[3]->setMotionVector(-1.0, 0.0);

	shapes[4] = new ShapeHook(200, 20, w);
	shapes[4]->setMotionVector(1.0, 0.0);
	shapes[5] = new ShapeHook(400, 30, w, Scalar(0, 255, 0));
	shapes[5]->setMotionVector(-1.0, 0.0);

}


VideoSource::~VideoSource()
{
}

bool VideoSource::getFrame(Mat frame)
{
	frame.setTo(Scalar(0, 0, 0));
	for (int i = 0; i < 2; i++)
		shapes[i]->updateDraw(frame);
	return true;
}
