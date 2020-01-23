#include <iostream>
#include <chrono>
#include <algorithm>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d.hpp>

#include <possibleObject.h>
#include <Camera.h>
#include <Graph.h>
#include <imgProcessing.h>
#include <ObjectDetection.h>
#include <Statistics.h>
#include "Map.h"

using namespace std;
using namespace cv;
using namespace chrono;


vector<possibleObject> RemoveOverlaps(vector<possibleObject> sortedData)
{
	if (sortedData.size() < 2) { return sortedData; }
	reverse(sortedData.begin(), sortedData.end());
	for (auto it = sortedData.begin(); it != sortedData.end(); ++it)
	{
		if (it->avgVal == 0)
		{
			sortedData.erase(it);
			it = sortedData.begin();
		}

		for (auto cit = it; cit != sortedData.end(); ++cit)
		{
			if (it == cit)
			{
				continue;
			}

			bool overlaps = it->overlap(*cit);

			if (overlaps)
			{
				sortedData.erase(cit);
				cit = it;
			}
		}
	}

	return sortedData;
}

int main(int argc, char* argv[])
{
	VideoCapture cap(CAP_DSHOW);
	VideoWriter videoWriter;
	Mat base;
	if (!Initialise_Camera(cap, videoWriter, base))
	{
		cin.get();
		return -1;
	}

	/*CONTROLS*/
	namedWindow("Control", WINDOW_AUTOSIZE);
	int distanceThresh = 40;
	int iAlpha = 8;
	int iHThreshold = 50;
	int iVThreshold = 40;
	int contrastChange = 150;

	int poseAngle = 0;
	
	createTrackbar("Alpha", "Control", &iAlpha, 10);
	createTrackbar("HThreshold", "Control", &iHThreshold, 100);
	createTrackbar("VThreshold", "Control", &iVThreshold, 100);
	createTrackbar("DThreshold", "Control", &distanceThresh, 255);
	createTrackbar("Contrast", "Control", &contrastChange, 200);
	createTrackbar("Angle", "Control", &poseAngle, 360);
	int h_bins = 179, s_bins = 255, l_bins = 255;
	/*END CONTROLS*/

	Map myMap;
	Pose defaultPose;
	
	while (true)
	{
		defaultPose.heading = degToRad(poseAngle);
		
		Mat frame;
		bool readSuccess = cap.read(frame);

		if (!readSuccess)
		{
			cout << "Stream ended" << endl;
			break;
		}
		
		if (waitKey(1) == 27) break;

		int topBorder = 5, bottomBorder = topBorder;
		int leftBorder = 5, rightBorder = topBorder;

		copyMakeBorder(frame, frame, topBorder, bottomBorder, leftBorder, rightBorder, BORDER_CONSTANT, Scalar(0, 0, 0));

		Mat lPlane = applyChannelFilter(frame);
		Mat distant = increaseDistance(lPlane, distanceThresh);
		lPlane = applyMask(lPlane, distant);
		lPlane = lPlane * (contrastChange / 100.0);

		auto rowCol = calculateRowColumnHistograms(lPlane);
		auto lightnessRow = rowCol.first;
		auto lightnessCol = rowCol.second;

		Mat outputFrame = frame.clone();
		Graph g(outputFrame);

		int graph_h = outputFrame.rows, graph_w = outputFrame.cols;

		int h_bin_w = cvRound((double)graph_h / (double)lightnessRow.size());
		int v_bin_w = cvRound((double)graph_w / (double)lightnessCol.size());

		auto filteredRow = applyExponentialFilter<float>(lightnessRow, (iAlpha / 10.0));
		auto filteredCol = applyExponentialFilter<float>(lightnessCol, (iAlpha / 10.0));

		g.drawLineGraph<float>(filteredRow, Scalar(255, 0, 0), false);
		g.drawLineGraph<float>(filteredCol, Scalar(0, 0, 255), true);

		auto rowChange = changeValue<float>(filteredRow);
		auto colChange = changeValue<float>(filteredCol);

		float rowThresh = calcThreshold(filteredRow, iHThreshold);
		float colThresh = calcThreshold(filteredCol, iVThreshold);

		auto rowTriggers = changeDetect(rowChange, rowThresh);
		rowTriggers = removeAdjacent<int>(rowTriggers);
		auto colTriggers = changeDetect(colChange, colThresh);
		colTriggers = removeAdjacent<int>(colTriggers);
		
		g.drawLineGraph<float>(rowChange, Scalar(255, 115, 115), false, 10);
		g.drawLineGraph<float>(colChange, Scalar(115, 115, 255), true, 50);

		vector<Point> intersects;

		for (auto it = rowTriggers.begin(); it != rowTriggers.end(); ++it)
		{
			g.drawLine(Point(0, h_bin_w * (*it)), Point(graph_w, h_bin_w * (*it)), Scalar(0, 255, 0));
			for (auto cit = colTriggers.begin(); cit != colTriggers.end(); ++cit)
			{
				Point intersect = Point(*cit, *it);
				intersects.push_back(intersect);
			}
		}
		for (auto it = colTriggers.begin(); it != colTriggers.end(); ++it)
		{
			g.drawLine(Point(h_bin_w * (*it), 0), Point(h_bin_w * (*it), graph_h), Scalar(0, 255, 0));
		}

		vector<possibleObject> possible_objects;
		if(intersects.size() > 1)
		{
			for (int i = 0; i < intersects.size() - 1; ++i)
			{
				Point x = intersects[i];
				for (int j = i + 1; j < intersects.size(); ++j)
				{
					Point y = intersects[j];
					auto r = Rect(x, y);
					float viewArea = outputFrame.cols * outputFrame.rows;
					if (r.area() >= viewArea * 0.05) //5% of view, possibly too harsh?
					{
						possible_objects.push_back(possibleObject(Rect(x, y)));
					}

				}
			}
		}
		
		std::sort(possible_objects.begin(), possible_objects.end());

		auto middling = possible_objects;//unique(possible_objects.begin(), possible_objects.end());
		for (auto it = middling.begin(); it != middling.end(); ++it)
		{
			it->computeAvgVal(distant);
			if (it->avgVal <= 125) //Halfway value, meaning half or more of the box isn't on the object
			{
				it->avgVal = 0;
			}
			else if (it->avgVal > 230) //90% of the box is over the object
			{
				it->avgVal = 255;
			}
		}
		std::sort(middling.begin(), middling.end());
		auto detectedObjects = RemoveOverlaps(middling);
		
		const float ObjectWidth = 0.038; //m
		const float redObjectHeight = 0.079; //m
		const float blueObjectHeight = 0.118; //m
		const float yellowObjectHeight = ObjectWidth; //Cube
		
		Mat hsvFrame;
		vector<Mat> hsvPlanes;
		cvtColor(frame, hsvFrame, COLOR_BGR2HSV);
		split(hsvFrame, hsvPlanes);
		for (auto it = detectedObjects.begin(); it != detectedObjects.end(); ++it)
		{
			Mat mask = Mat(hsvPlanes[0].rows, hsvPlanes[0].cols, CV_8UC1, Scalar(0, 0, 0));
			rectangle(mask, it->bounds, Scalar::all(255), FILLED);
			float colour = mean(hsvPlanes[0], mask)[0];

			if (colour >= 140.0) //Probably quite blue
			{
				it->estimatedSize = Size2f(ObjectWidth, blueObjectHeight);
			} else if(colour > 90.0)
			{
				it->estimatedSize = Size2f(ObjectWidth, redObjectHeight);
			} else
			{
				it->estimatedSize = Size2f(ObjectWidth, yellowObjectHeight);
			}
		}

		
		const float HorizontalFOV = degToRad(70.42);
		const float VerticalFOV = degToRad(43.3);
		const float focalLength = 0.00367; //m
	
		Mat objMask = Mat(frame.rows, frame.cols, CV_8UC1, Scalar(0, 0, 0));
		for (auto it = detectedObjects.begin(); it != detectedObjects.end(); ++it)
		{
			//cout << "Avg Val: " << pO.avgVal << " for:" << pO.bounds << endl;
			int xMax, yMax, xMin, yMin;
			xMax = max(it->bounds.tl().x, it->bounds.br().x);
			yMax = max(it->bounds.tl().y, it->bounds.br().y);

			xMin = min(it->bounds.tl().x, it->bounds.br().x);
			yMin = min(it->bounds.tl().y, it->bounds.br().y);

			g.drawLine(Point(xMin, yMax), Point(xMax, yMin), Scalar(255, 0, 0), 3);
			g.drawLine(Point(xMax, yMax), Point(xMin, yMin), Scalar(255, 0, 0), 3);

			rectangle(objMask, it->bounds, Scalar(255, 255, 255), FILLED);

			//Attempt some depth perception
			float estimateDepth = it->computeDistance(it->estimatedSize,Size2f(frame.cols,frame.rows),Size2f(HorizontalFOV,VerticalFOV),focalLength)*100.0;
			myMap.addMeasurement(defaultPose, estimateDepth, it->maxMinAngles);
			string depthMsg = std::to_string(estimateDepth) + "cm";
			//cout << "Estimated depth: " << depthMsg << endl;
			int font = FONT_HERSHEY_SIMPLEX;
			Size textSize = getTextSize(depthMsg, font, 1, 2, 0);
			Point textLoc(it->bounds.x, it->bounds.y);
			putText(g.drawing, depthMsg, textLoc, font, 1, Scalar::all(75), 2);
		}
		
		outputFrame += g.drawing;
		imshow("Source image", outputFrame);
		int scaling = 10;
		Size displaySize = Size(myMap.map.cols*scaling,myMap.map.rows*scaling);
		Mat displayMap;
		resize(myMap.map, displayMap, displaySize);
		cvtColor(displayMap, displayMap, COLOR_GRAY2RGB);
		Point2f startPoint = Point2f((displayMap.cols / 2) + 1 + defaultPose.x, (displayMap.rows / 2) + 1 + defaultPose.y);
		Point2f endPoint(startPoint.x + scaling * sin(defaultPose.heading), startPoint.y + scaling * cos(defaultPose.heading));
		circle(displayMap, startPoint, scaling, Scalar(255, 0, 0));
		line(displayMap, startPoint, endPoint, Scalar(255, 0, 0));
		imshow("Map", displayMap);
		//if(waitKey() == 27) break;
	}
	return 0;
}
