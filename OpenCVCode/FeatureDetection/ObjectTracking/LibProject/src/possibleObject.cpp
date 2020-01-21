#include "possibleObject.h"

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d.hpp>

float possibleObject::computeAvgVal(Mat img)
{
	Mat mask = Mat(img.rows, img.cols, CV_8UC1, Scalar(0, 0, 0));
	rectangle(mask, bounds, Scalar::all(255), FILLED);
	auto avg = mean(img, mask);
	avgVal = avg[0];
	return avgVal;
}

bool possibleObject::overlap(const possibleObject& other) const
{
	Point l1 = Point(min(bounds.tl().x, bounds.br().x), max(bounds.tl().y, bounds.br().y));
	Point r1 = Point(max(bounds.tl().x, bounds.br().x), min(bounds.tl().y, bounds.br().y));

	Point l2 = Point(min(other.bounds.tl().x, other.bounds.br().x), max(other.bounds.tl().y, other.bounds.br().y));
	Point r2 = Point(max(other.bounds.tl().x, other.bounds.br().x), min(other.bounds.tl().y, other.bounds.br().y));


	// If one rectangle is on left side of other 
	if (l1.x > r2.x || l2.x > r1.x)
	{
		return false;
	}


	// If one rectangle is above other 
	if (l1.y < r2.y || l2.y < r1.y)
	{
		return false;
	}


	return true;
}

bool possibleObject::operator<(const possibleObject& other) const
{
	if (avgVal == other.avgVal)
	{
		return bounds.area() < other.bounds.area();
	}

	return avgVal < other.avgVal;
}

bool possibleObject::operator==(const possibleObject& other) const
{
	if (bounds != other.bounds)
	{
		return false;
	}

	if (avgVal != other.avgVal)
	{
		return false;
	}

	return true;
}



