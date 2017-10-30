// Video Segmentation.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void motionVect() {
	VideoCapture cap(0);


	Mat flow, frame;
	// some faster than mat image container
	UMat  flowUmat, prevgray;
	
	for (;;)
	{

		bool Is = cap.grab();
		if (Is == false) {
			// if video capture failed
			cout << "Video Capture Fail" << endl;
			break;
		}
		else {
			Mat img;
			Mat original;
			// capture frame from video file
			Mat img2;
			cap.retrieve(img2, CV_CAP_OPENNI_BGR_IMAGE);
			GaussianBlur(img2, img2, Size(7, 7), 0, 0);
			Canny(img2, img, 100, 150);
			resize(img, img, Size(640, 480));

			// save original for later
			img.copyTo(original);

			// just make current frame gray
			//cvtColor(img, img, COLOR_BGR2GRAY);


			// For all optical flow you need a sequence of images.. Or at least 2 of them. Previous                           //and current frame
			//if there is no current frame
			// go to this part and fill previous frame
			//else {
			// img.copyTo(prevgray);
			//   }
			// if previous frame is not empty.. There is a picture of previous frame. Do some                                  //optical flow alg. 

			if (prevgray.empty() == false) {

				// calculate optical flow 
				calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);
				// copy Umat container to standard Mat
				flowUmat.copyTo(flow);


				// By y += 5, x += 5 you can specify the grid 
				for (int y = 0; y < original.rows; y += 5) {
					for (int x = 0; x < original.cols; x += 5)
					{
						// get the flow from y, x position * 10 for better visibility
						const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
						// draw line at flow direction
						line(original, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255, 0, 0));
						// draw initial point
						circle(original, Point(x, y), 1, Scalar(0, 0, 0), -1);


					}

				}

				// draw the results
				namedWindow("prew", WINDOW_AUTOSIZE);
				imshow("prew", original);

				// fill previous image again
				img.copyTo(prevgray);

			}
			else {

				// fill previous image in case prevgray.empty() == true
				img.copyTo(prevgray);

			}


			int key1 = waitKey(20);

		}
	}
}

int main()
{
	//motionVect();


	VideoCapture cap(0);
	Mat frame;
	Mat prevFrame,prevEdgeFrame;
	Mat diffFrame, diffEdgeFrame,edgeFrame;
	Point rook_points[1][7];
	int w = 60;
	rook_points[0][0] = Point(w / 4.0, 7 * w / 8.0);
	rook_points[0][1] = Point(3 * w / 4.0, 7 * w / 8.0);
	rook_points[0][2] = Point(3 * w / 4.0, 13 * w / 16.0);
	rook_points[0][3] = Point(11 * w / 16.0, 13 * w / 16.0);
	rook_points[0][4] = Point(19 * w / 32.0, 3 * w / 8.0);
	rook_points[0][5] = Point(3 * w / 4.0, 3 * w / 8.0);
	rook_points[0][6] = Point(3 * w / 4.0, w / 8.0);
	rook_points[0][7] = Point(26 * w / 40.0, w / 8.0);
	RNG rng(12345);

	const Point* ppt[1] = { rook_points[0] };
	int npt[] = { 7 };
	int x = 0;
	int y = 0;
	bool ret = cap.read(frame);
	Canny(frame, edgeFrame, 100, 150);
	while (true) {
		edgeFrame.copyTo(prevEdgeFrame);
		frame.copyTo(prevFrame);
		ret = cap.read(frame);
		GaussianBlur(frame, frame, Size(7, 7), 0, 0);

		for (int i = 0; i < 7; i++){
			rook_points[0][i].x += 1;
			rook_points[0][i].y += 3;
		}
		fillPoly(frame, ppt, npt,1,
			Scalar(255, 0, 255),
			8);
		//absdiff(frame, prevFrame, diffFrame);
		Canny(frame, edgeFrame,100,150);
		/// Find contours
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(edgeFrame, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		absdiff(edgeFrame, prevEdgeFrame, diffEdgeFrame);
		absdiff(prevFrame,frame,diffFrame);
		
		diffFrame.mul(diffEdgeFrame);
		Mat greyMat;
		cvtColor(diffFrame, greyMat, cv::COLOR_BGR2GRAY);
		
		/*for (int i = 0; i< contours.size(); i++)
		{
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());
		}*/
		

		imshow("cam", frame);
		imshow("edge", edgeFrame);
		imshow("diff", greyMat);

		if (waitKey(30) >= 0)
			break;
	}
    return 0;
}

