#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/ocl.hpp>

using namespace cv;
using namespace std;

class Utilities
{
public:
	static int find_boundingBoxes(Mat image, cv::Rect2d * boundingBoxes, int maxBoundingBoxes);
	static vector<vector<Point>> find_contours(Mat image, int maxBoundingBoxes);
	static Mat get_object_contours(Mat frame, Mat prevFrame, int maxBoundingBoxes);
};

