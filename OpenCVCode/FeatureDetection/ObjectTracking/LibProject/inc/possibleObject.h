#pragma once

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)

using namespace cv;
class possibleObject
{
public:
	Rect bounds;
	float avgVal;

	possibleObject(Rect _b) : bounds(_b), avgVal(0) {}

	float computeAvgVal(Mat img);

	bool overlap(const possibleObject& other) const;

	bool operator<(const possibleObject& other) const;
	bool operator==(const possibleObject& other) const;

};