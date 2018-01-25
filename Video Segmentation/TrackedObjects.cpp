#include "stdafx.h"
#include "TrackedObjects.h"
#include "Evaluator.h"

#include "opencv2/xfeatures2d.hpp"

TrackedObjects::TrackedObjects(Mat frame)
{
	trackedObjects = (TrackedObject**)malloc(maxObjects * sizeof(TrackedObject*));
	cvtColor(frame, prevFrame, CV_BGR2GRAY);
}


TrackedObjects::~TrackedObjects()
{
}


bool isAngleBetween(double angle, double bound1, double bound2) {
	//n = (360 + (n % 360)) % 360;
	angle = fmod(360 + fmod(angle, 360), 360);
	//a = (3600000 + a) % 360;
	bound1 = fmod(3600000 + bound1, 360);
	//b = (3600000 + b) % 360;
	bound2 = fmod(3600000 + bound2, 360);

	if (bound1 < bound2)
		return bound1 <= angle && angle <= bound2;
	return bound1 <= angle || angle <= bound2;
}


TrackedObject* TrackedObjects::getTrackedObject(int i) {
	return trackedObjects[i];
}

int TrackedObjects::find(Mat frame, Rect2d* objectBoundingBoxes,int objectsFound,int indexStart) {
	//int objectsFound = find_contour(frame, objectBoundingBoxes, maxObjects * 1000);
	// Display bounding box.
	int i = 0, j = 0;
	while (i+indexStart < maxObjects && j+indexStart < objectsFound) {
		if (objectBoundingBoxes[j].area() > 12) {
			//rectangle(frame, objectBoundingBoxes[i], Scalar(255, 0, 0), 2, 1);
			trackedObjects[i+indexStart] = new TrackedObject(frame, objectBoundingBoxes[j], i);
			i++;
		}
		j++;
	}
	numObjects = i;
	return numObjects;
}

UMat flow;
int TrackedObjects::find(Mat frame)
{

	//trackedObjects = (TrackedObject**)malloc(maxObjects * sizeof(TrackedObject*));
	int objectsFound = 0;
	Mat nextGray, flowPolar, out(frame.rows, frame.cols, CV_8UC3);

	cvtColor(frame, nextGray, CV_BGR2GRAY);
	UMat uPrevFrame, uNextGray;//TODO: UMat
	prevFrame.copyTo(uPrevFrame);
	nextGray.copyTo(uNextGray);
	imshow("prev", uPrevFrame);
	imshow("next", uNextGray);
	//calcOpticalFlowFarneback(uPrevFrame, uNextGray, flow, 0.5, 5, 30,30, 5, 1.2, OPTFLOW_USE_INITIAL_FLOW);
	calcOpticalFlowFarneback(uPrevFrame, uNextGray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
	//TODO: try cuda::FarnebackOpticalFlow? 


	vector<Mat> channels(2);
	Mat angle, mag;
	// split img:
	split(flow, channels);
	cartToPolar(channels[0], channels[1], mag, angle, true);

	Mat hsv[3];
	split(out, hsv);

	hsv[0] = (angle );
	normalize(mag, hsv[1], 255, 255, NORM_MINMAX);
	normalize(mag, hsv[2], 0, 255, NORM_MINMAX);
	merge(hsv, 3, out);
	//hsv[1].setTo(254.9);
	merge(hsv, 3, out);
	cvtColor(out, out, CV_HSV2BGR);
	imshow("Flow", out);

	Mat hist;
	/// Establish the number of bins
	int histSize = 36;
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound((double)hist_w / histSize);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));


	/// Set the ranges ( for H) )
	float range[] = { 1, 360 };
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;
	
	//discard small motions
	Mat smallMotionMask;
	threshold(hsv[2], smallMotionMask, 255, 200,THRESH_BINARY);
	imshow("sm", smallMotionMask);
	smallMotionMask.convertTo(smallMotionMask, CV_8U);
	angle.setTo(0, smallMotionMask);

	/// Compute the histograms:
	calcHist(&angle, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
	//Mat inverseS;
	//threshold(hsv[1], inverseS, 2, 1, THRESH_BINARY_INV);
	//std::cout << "/n" +SSTR(hist.at<float>(0))+" " +SSTR(hist.at<float>(1)) +"/n";// sum(inverseS)[0];//Remove background;
	/*for (int i = 0; i < 36; i++) {
		std::cout << SSTR(hist.at<float>(i)) + "\n";
	}*/
	//normalize(hist, hist, 0, hist.rows, NORM_MINMAX);

	for (int i = 0; i < histSize; i++)
	{
		if (hist.at<float>(i) > 350) {
			line(histImage, Point(bin_w*(i), hist_h),
				Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
				Scalar(255, 0, 0), 2, 8, 0);
			//get masks
			Mat mask;
			Mat maskV;
			inRange(hsv[0], (i *10-25), (i*10+25) , mask);
			inRange(hsv[2], 30, 255, maskV);
			//imshow("S"+SSTR(i), mask);
			/*Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(10, 10));
			dilate(mask, mask, structuringElement);
			erode(mask, mask, structuringElement);*/
			mask &= maskV;
			//imshow("q", mask);
			//mask |= ~smallMotionMask;//remove small motion areas
			//imshow("Sqq", mask);
			//imshow("Q"+SSTR(i), mask);
			Mat masked;
			frame.copyTo(masked, mask);
			//imshow("Masked"+SSTR(i*10), masked);
			int potentialObjectsFound = find_contour(masked, objectBoundingBoxes, maxObjects * 1000);

			int k = 0;
			int j = 0;
			while (k + objectsFound < maxObjects && j < potentialObjectsFound) {
				if (objectBoundingBoxes[j].area() > 200) {
					//rectangle(frame, objectBoundingBoxes[i], Scalar(255, 0, 0), 2, 1);
					//trackedObjects[k + objectsFound] = new TrackedObject(frame, objectBoundingBoxes[j], k+objectsFound);

					bool foundExistingObject = false;
					for (int objectId = 0; objectId < numObjects; objectId++) {
						//If overlap
						//Expand to group separate bits
						int expandingFactor = 50;
						Rect2d expandedBoundingBox(objectBoundingBoxes[j].tl().x-expandingFactor, objectBoundingBoxes[j].tl().y-expandingFactor,
							objectBoundingBoxes[j].width +2*expandingFactor, objectBoundingBoxes[j].height+2*expandingFactor);
						if (((expandedBoundingBox & trackedObjects[objectId]->getBoundingBox()).area() > 0)){
							//If going in same direction
							Vec2d objectMotionVector = trackedObjects[objectId]->motionVector;//getMotionVector();
							float angle = cvFastArctan(trackedObjects[objectId]->motionVector.val[1], trackedObjects[objectId]->motionVector.val[0]);
							printf("x%f y%f %f angle \n", trackedObjects[objectId]->motionVector.val[0], trackedObjects[objectId]->motionVector.val[1],angle);
							if (isAngleBetween(angle,(i * 10 - 25), (i * 10 + 25))) {
								//trackedObjects[objectId]->updateTracker(frame,objectBoundingBoxes[j]);
								//foundExistingObject = true;
								break;
							}
						}
					}
					
					if (!foundExistingObject && objectBoundingBoxes[j].area() > 30 && numObjects < maxObjects) {
						trackedObjects[numObjects] = new TrackedObject(frame, objectBoundingBoxes[j], numObjects);
						numObjects++;
					}
					k++;

					/*
					//Ptr<cv::xfeatures2d::SIFT> sift = xfeatures2d::SIFT::create();
					Ptr<cv::xfeatures2d::SURF> sift = xfeatures2d::SURF::create();
					sift->setHessianThreshold(400);

					Mat img_1 = frame(objectBoundingBoxes[j]);
					Mat img_2 = trackedObjects[max(k + objectsFound - 1,0)]->getObjectMat();
					std::vector<KeyPoint> keypoints_1, keypoints_2;
					Mat descriptors_1, descriptors_2;
					sift->detectAndCompute(img_1, Mat(), keypoints_1, descriptors_1);
					sift->detectAndCompute(img_2, Mat(), keypoints_2, descriptors_2);


					//-- Step 2: Matching descriptor vectors using FLANN matcher
					FlannBasedMatcher matcher;
					std::vector< DMatch > matches;
					matcher.match(descriptors_1, descriptors_2, matches);
					double max_dist = 0; double min_dist = 100;
					//-- Quick calculation of max and min distances between keypoints
					for (int i = 0; i < descriptors_1.rows; i++)
					{
						double dist = matches[i].distance;
						if (dist < min_dist) min_dist = dist;
						if (dist > max_dist) max_dist = dist;
					}
					printf("-- Max dist : %f \n", max_dist);
					printf("-- Min dist : %f \n", min_dist);
					//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
					//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
					//-- small)
					//-- PS.- radiusMatch can also be used here.
					std::vector< DMatch > good_matches;
					for (int i = 0; i < descriptors_1.rows; i++)
					{
						if (matches[i].distance <= max(2 * min_dist, 0.02))
						{
							good_matches.push_back(matches[i]);
						}
					}
					//-- Draw only "good" matches
					Mat img_matches(1920,1080,CV_8UC3);
					drawMatches(img_1, keypoints_1, img_2, keypoints_2,
						good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
						vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
					//-- Show detected matches
					imshow("Good Matches"+SSTR(k+objectsFound), img_matches);
					for (int i = 0; i < (int)good_matches.size(); i++)
					{
						printf("-- Good Match%d [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", k+objectsFound,i, good_matches[i].queryIdx, good_matches[i].trainIdx);
					}*/

					
				}
				j++;
			}
			objectsFound += k;
			
		}
	}
	/// Display
	imshow("calcHist Demo", histImage);
	cvtColor(frame, prevFrame, CV_BGR2GRAY);
	//numObjects = objectsFound;
	return numObjects;
	
}

int TrackedObjects::find_contour(Mat image, cv::Rect2d* boundingBoxes, int maxBoundingBoxes)
{
	Mat src_mat, gray_mat, canny_mat;
	Mat contour_mat;
	Mat bounding_mat;
	Mat out;

	contour_mat = image.clone();
	bounding_mat = image.clone();

	cvtColor(image, gray_mat, CV_BGR2GRAY);

	// apply canny edge detection
	Canny(image, canny_mat, 100, 150, 3, false);



	//Dialate edges to close small gaps so contour detection creates one object
	Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(3, 3));
	for (int i = 0; i < 10; i++) {
		dilate(canny_mat, canny_mat, structuringElement);
		erode(canny_mat, canny_mat, structuringElement);
	}

	//3. Find & process the contours
	//3.1 find contours on the edge image.
	vector< vector< cv::Point> > contours;
	findContours(canny_mat, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//findContours(canny_mat, contours, CV_RETR_TREE, CHAIN_APPROX_SIMPLE);

	//TODO: Remove
	Mat drawing = Mat::zeros(canny_mat.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(drawing, contours, i, color, 2, 8);
	}
	imshow("Contours", drawing);
	//3.2 draw contours & property value on the source image.

	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;


	image.copyTo(out);
	for (size_t i = 0; i< contours.size(); i++) // iterate through each contour.
	{
		// draw rectangle around the contour:
		cv::Rect boundingBox = boundingRect(contours[i]);
		//if (i < maxBoundingBoxes)
		boundingBoxes[i] = boundingBox;
		cv::rectangle(out, boundingBox, cv::Scalar(255, 0, 255)); // if you want read and "image" is color image, use cv::Scalar(0,0,255) instead

																  // you aren't using the largest contour at all? no need to compute it...
																  /*
																  double area = contourArea(contours[i]);  //  Find the area of contour

																  if (area > largest_area)
																  {
																  largest_area = area;
																  largest_contour_index = i;               //Store the index of largest contour
																  bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
																  }
																  */
	}

	//drawContours(image, contours, largest_contour_index, Scalar(0, 255, 0), 2);

	imshow("Bounding ", out);

	return contours.size();
}


int TrackedObjects::findObjects(Mat frame, TrackedObject** trackedObjects, int numObjects, TrackedObject** newTrackedObjects) {//TODO: think about freeing newTrackedObjects
	cv::Rect2d objectBoundingBoxes[1000];//TODO: CHANGE
	int objectsFound = find_contour(frame, objectBoundingBoxes, maxObjects * 1000);

	trackedObjects = (TrackedObject**)malloc(maxObjects * sizeof(TrackedObject*));
	// Display bounding box.
	int i = 0, j = 0, k = 0;
	while (i < maxObjects && j < objectsFound) {
		if (objectBoundingBoxes[j].area() > 12) {
			//rectangle(frame, objectBoundingBoxes[i], Scalar(255, 0, 0), 2, 1);

			double bestMatchValue = -1;
			int bestMatchIndex = -1;

			for (int j = 0; j < numObjects; j++) {
				double similarity = Evaluator::evaluateSegmentsSimalarity(frame, trackedObjects[j]->getObjectMat(), objectBoundingBoxes[j]);
				if (bestMatchValue < similarity) {
					bestMatchIndex = j;
					bestMatchValue = similarity;
				}
			}

			if (bestMatchValue > ACCEPTANCE_PROBABILITY) {//TODO: Want to check is not already added
				newTrackedObjects[k] = trackedObjects[bestMatchIndex];
				k++;
			}
			else {
				trackedObjects[k] = new TrackedObject(frame, objectBoundingBoxes[j], i);
				k++;
			}


			i++;
		}
		j++;
	}
}


void TrackedObjects::update(Mat frame, Mat outputFrame) {
	cvtColor(frame, prevFrame, CV_BGR2GRAY);
	for (int i = 0; i < numObjects; i++) {
		if (trackedObjects[i]) {
			bool ok = trackedObjects[i]->update(frame);
			if (!ok)
			{
				// Tracking failure detected.
				putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
			}
		}
	}
	for (int i = 0; i < numObjects; i++) {
		bool isOverlap = false;
		for (int j = 0; j < numObjects; j++) {
			if (i != j) {
				isOverlap = trackedObjects[i]->boundingBoxOverlap(*trackedObjects[j]);
				break;
			}
		}
		trackedObjects[i]->drawSegment(frame, isOverlap, outputFrame);
	}
}

