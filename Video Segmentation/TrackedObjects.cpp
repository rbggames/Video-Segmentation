#include "stdafx.h"
#include "TrackedObjects.h"
#include "Evaluator.h"
#include "Utilities.h"



TrackedObjects::TrackedObjects(Mat frame)
{
	cvtColor(frame, prevFrame, CV_BGR2GRAY);
}


TrackedObjects::~TrackedObjects()
{
}

TrackedObject* TrackedObjects::getTrackedObject(int i) {
	if (i < trackedObjectList.size())
		return &trackedObjectList.at(i);
	else
		return NULL;
}

int id = 0;
int TrackedObjects::find(Mat frame)
{
	int objectsFound = 0;
	UMat flow;
	Mat nextGrey, flowPolar, out(frame.rows, frame.cols, CV_8UC3);
	cvtColor(frame, nextGrey, CV_BGR2GRAY);
	calcOpticalFlowFarneback(prevFrame, nextGrey, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

	vector<Mat> channels(2);
	Mat angle, mag;
	// split img:
	split(flow, channels);
	cartToPolar(channels[0], channels[1], mag, angle, true);

	Mat hsv[3];
	split(out, hsv);


	hsv[0] = angle;
	normalize(mag, hsv[1], 255, 255, NORM_MINMAX);
	normalize(mag, hsv[2], 0, 255, NORM_MINMAX);
	merge(hsv, 3, out);
	//hsv[1].setTo(254.9);
	merge(hsv, 3, out);
	

	//cvtColor(out, out, CV_HSV2BGR);
	//out = Utilities::kNearestColours(out, 5);
	imshow("Flow", out);

	//cvtColor(out, out, CV_BGR2HSV);
	//split(out, hsv);

	Mat hist;
	/// Establish the number of bins
	int histSize = 12;
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound((double)hist_w / histSize);
	int bin_size = cvRound(360.0 / histSize);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));


	/// Set the ranges ( for H) )
	float range[] = { 1, 360 };
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;

	/// Compute the histograms:
	calcHist(&angle, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
	//Mat inverseS;
	//threshold(hsv[1], inverseS, 2, 1, THRESH_BINARY_INV);
	//std::cout << "/n" +SSTR(hist.at<float>(0))+" " +SSTR(hist.at<float>(1)) +"/n";// sum(inverseS)[0];//Remove background;
	//normalize(hist, hist, 0, hist.rows, NORM_MINMAX);

	for (int i = 0; i < histSize; i++)
	{
		if (hist.at<float>(i) > 150) {
			std::cout << "Hist: " + SSTR(i) +" "+ SSTR(hist.at<float>(i)) + "\n";
			line(histImage, Point(bin_w*(i), hist_h),
				Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
				Scalar(255, 0, 0), 2, 8, 0);
			//get masks
			Mat mask;
			Mat maskV;
			inRange(hsv[0], (i *bin_size), (i*bin_size + bin_size) , mask);
			inRange(hsv[2], 0, 30, maskV);
			//imshow("S", maskV);
			Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(7, 7));
			dilate(mask, mask, structuringElement);
			mask |= maskV;
			//imshow("Q"+SSTR(i), mask);
			Mat masked;
			frame.copyTo(masked, mask);
			imshow("Masked"+SSTR(i*bin_size), masked);
			int potentialObjectsFound = find_contour(masked, objectBoundingBoxes, maxObjects * 1000);
			//find(frame, objectBoundingBoxes, potentialObjectsFound, objectsFound);
			//objectsFound += potentialObjectsFound;
			int k = 0, j = 0;
			while (trackedObjectList.size() < maxObjects && j < potentialObjectsFound) {
				if (objectBoundingBoxes[j].area() > 100) {
					//rectangle(frame, objectBoundingBoxes[i], Scalar(255, 0, 0), 2, 1);
					//trackedObjects[k + objectsFound] = new TrackedObject(frame, objectBoundingBoxes[j], k+objectsFound);
					
					bool foundExistingObject = false;
					for (int objectId = 0; objectId < trackedObjectList.size(); objectId++) {
						//If overlap
						//Expand to group separate bits
						int expandingFactor = 50;
						Rect2d expandedBoundingBox(objectBoundingBoxes[j].tl().x - expandingFactor, objectBoundingBoxes[j].tl().y - expandingFactor,
							objectBoundingBoxes[j].width + 2 * expandingFactor, objectBoundingBoxes[j].height + 2 * expandingFactor);
						Rect2d objectBoundingBox = trackedObjectList.at(objectId).getBoundingBox();
						if (((expandedBoundingBox & objectBoundingBox).area() > 0)) {
							//If going in same direction
							float angle = trackedObjectList.at(objectId).getMotionAngle();

							//printf("x%f y%f %f angle \n", trackedObjectList.at(objectId).motionVector.val[0], trackedObjectList.at(objectId).motionVector.val[1], angle);
							if (Utilities::isAngleBetween(angle, (i * bin_size - bin_size), (i * bin_size + bin_size*2))) {
								//trackedObjects[objectId]->updateTracker(frame,objectBoundingBoxes[j]);
								//get masks
								Mat mask2;
								Mat maskV2;
								inRange(hsv[0], (i *bin_size - bin_size), (i*bin_size + bin_size*2), mask2);
								inRange(hsv[2], 0, 30, maskV2);
								//imshow("S", maskV);
								Mat structuringElement = getStructuringElement(MORPH_DILATE, Size(7, 7));
								dilate(mask2, mask2, structuringElement);
								mask2 |= maskV2;
								//imshow("Q"+SSTR(i), mask);
								Mat masked2;
								frame.copyTo(masked2, mask2);
								trackedObjectList.at(objectId).updateTracker(masked2, objectBoundingBoxes[j]);
								foundExistingObject = true;
								break;
							}
						}
					}

					if (!foundExistingObject && objectBoundingBoxes[j].area() > 30 && trackedObjectList.size() < maxObjects) {
						TrackedObject* newObject = new TrackedObject(frame, objectBoundingBoxes[j], i * bin_size + bin_size / 2,id++);
						std:cout << "ID " + SSTR(id) + " angle " + SSTR(i * bin_size);
						trackedObjectList.push_back(*newObject);
					}
					
					k++;
				}
				j++;
			}
			objectsFound += k;
			
		}
	}
	/// Display
	imshow("calcHist Demo", histImage);
	cvtColor(frame, prevFrame, CV_BGR2GRAY);
	return trackedObjectList.size();
	
}

int TrackedObjects::consolidateObjects()
{
	int i = 0;
	while (i < trackedObjectList.size()) {
		Vec2d motionVector = trackedObjectList.at(i).getMotionVector();
		double magnituedSquared = motionVector.val[0] * motionVector.val[0] + motionVector.val[1] * motionVector.val[1];
		Rect2d newBoundingBoxes[1000];//TODO: fix
		Mat object = trackedObjectList.at(i).getObjectMat();
		int number = 0;
		if(object.size().area() > 0)
			number = Utilities::find_boundingBoxes(object, newBoundingBoxes, 1);
		if (magnituedSquared < 0.01 || number == 0  || 
			(number > 0 && (newBoundingBoxes[0]& trackedObjectList.at(i).getTrackerBoundingBox()).area()< 25) ) {

			trackedObjectList.erase(trackedObjectList.begin() + i);
		}
		else {
			i++;
		}
		/*
		if (magnituedSquared < 0.01) {
			if(trackedObjectList.at(i).markedToRemove || number == 0 || (number > 0 && (newBoundingBoxes[0] & trackedObjectList.at(i).getTrackerBoundingBox()).area() < 25) ) {
					trackedObjectList.erase(trackedObjectList.begin() + i);
			}
			else {
				trackedObjectList.at(i).markedToRemove = true;
				i++;
			}
		}
		else {
			i++;
		}
		*/
	}

	return trackedObjectList.size();
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


void TrackedObjects::update(Mat frame, Mat outputFrame) {
	for (int i = 0; i < trackedObjectList.size(); i++) {
		bool ok = trackedObjectList.at(i).update(frame);
		if (!ok)
		{
			// Tracking failure detected.
			putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 255), 2);
		}
	}
	int i = 0, j = 0;
	double motionAngleI, motionAngleJ;
	while ( i < trackedObjectList.size()) {
		bool isOverlap = false;
		j = 0;
		while (j < trackedObjectList.size()) {
			if (i != j) {
				isOverlap = isOverlap || (trackedObjectList.at(i).getBoundingBox() & trackedObjectList.at(j).getBoundingBox()).area() > 0.0;
				//trackedObjectList.at(i).boundingBoxOverlap(trackedObjectList.at(j));

				//If objects are overlapping and moving in the same direction then they are likely the same, hence delete one
				motionAngleI = trackedObjectList.at(i).getMotionAngle();
				motionAngleJ = trackedObjectList.at(j).getMotionAngle();
				if (isOverlap && Utilities::isAngleBetween(motionAngleJ, motionAngleI-15, motionAngleI+15)) {
					trackedObjectList.erase(trackedObjectList.begin() + j);
					//Now need to make sure i is pointing to the correct position
					if (i > j) {
						//This has removed something before it therefore need to move i back by one
						i--;
					}
					isOverlap = false;
				} else {
					j++;
				}
			}
			else {
				j++;
			}
		}
		trackedObjectList.at(i).drawSegment(frame, isOverlap, outputFrame);
		i++;
	}
}

int TrackedObjects::getNumTrackedObjects() {
	return trackedObjectList.size();
}
