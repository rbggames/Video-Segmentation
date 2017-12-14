#include "stdafx.h"
#include "ShapeSquare.h"


ShapeSquare::ShapeSquare(int x, int y, int width, Scalar colour)
{
	int w = width;
	hook_points[0][0] = Point(0, 0);
	hook_points[0][1] = Point(0, w);
	hook_points[0][2] = Point(w, w);
	hook_points[0][3] = Point(w, 0);


	const Point* pts[1] = { hook_points[0] };

	const int npt[1] = { 4 };
	initialise(x, y, width, width, pts, npt, colour);
}


ShapeSquare::~ShapeSquare()
{
	for (int i = 0; i < 7; i++) {
		//free(hook_points[0]);
	}
	free(hook_points);
}
