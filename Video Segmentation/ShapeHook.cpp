#include "stdafx.h"
#include "ShapeHook.h"


ShapeHook::ShapeHook(int x, int y,int width,Scalar colour)
{
	int w = width;
	hook_points[0][0] = Point(w / 4.0, 7 * w / 8.0);
	hook_points[0][1] = Point(3 * w / 4.0, 7 * w / 8.0);
	hook_points[0][2] = Point(3 * w / 4.0, 13 * w / 16.0);
	hook_points[0][3] = Point(11 * w / 16.0, 13 * w / 16.0);
	hook_points[0][4] = Point(19 * w / 32.0, 3 * w / 8.0);
	hook_points[0][5] = Point(3 * w / 4.0, 3 * w / 8.0);
	hook_points[0][6] = Point(3 * w / 4.0, w / 8.0);
	hook_points[0][7] = Point(26 * w / 40.0, w / 8.0);


	const Point* pts[1] = { hook_points[0] };

	const int npt[1] = { 7 };
	initialise(x, y, width, width, pts, npt, colour);
}

ShapeHook::ShapeHook(int x, int y, int width, String texturePath)
{
	int w = width;
	hook_points[0][0] = Point(w / 4.0, 7 * w / 8.0);
	hook_points[0][1] = Point(3 * w / 4.0, 7 * w / 8.0);
	hook_points[0][2] = Point(3 * w / 4.0, 13 * w / 16.0);
	hook_points[0][3] = Point(11 * w / 16.0, 13 * w / 16.0);
	hook_points[0][4] = Point(19 * w / 32.0, 3 * w / 8.0);
	hook_points[0][5] = Point(3 * w / 4.0, 3 * w / 8.0);
	hook_points[0][6] = Point(3 * w / 4.0, w / 8.0);
	hook_points[0][7] = Point(26 * w / 40.0, w / 8.0);


	const Point* pts[1] = { hook_points[0] };

	const int npt[1] = { 7 };
	initialise(x, y, width, width, pts, npt, texturePath);
}


ShapeHook::~ShapeHook()
{
	for (int i = 0; i < 7; i++) {
		//free(hook_points[0]);
	}
	free(hook_points);
}
