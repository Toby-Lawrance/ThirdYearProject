#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;
class possibleObject
{
public:
	Rect bounds;
	float avgVal;

	possibleObject(Rect _b)
	{
		bounds = _b;
		avgVal = 0;
	}

	float computeAvgVal(Mat img)
	{
		Mat mask = Mat(img.rows, img.cols, CV_8UC1, Scalar(0, 0, 0));
		imshow("Mean mask", mask);
		rectangle(mask, bounds, Scalar::all(255), FILLED);
		auto avg = mean(img, mask);
		avgVal = avg[0];
		return avgVal;
	}

	bool overlap(const possibleObject& other) const
	{
		//cout << "This: " << bounds << " Other: " << other.bounds << endl;
		Point l1 = Point(min(bounds.tl().x, bounds.br().x), max(bounds.tl().y, bounds.br().y));
		Point r1 = Point(max(bounds.tl().x, bounds.br().x), min(bounds.tl().y, bounds.br().y));

		Point l2 = Point(min(other.bounds.tl().x, other.bounds.br().x), max(other.bounds.tl().y, other.bounds.br().y));
		Point r2 = Point(max(other.bounds.tl().x, other.bounds.br().x), min(other.bounds.tl().y, other.bounds.br().y));

		//cout << "L1:" << l1 << " R1: " << r1 << " L2:" << l2 << " R2: " << r2 << endl;

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

	bool operator<(const possibleObject& other) const
	{
		if (avgVal == other.avgVal)
		{
			return bounds.area() < other.bounds.area();
		}

		return avgVal < other.avgVal;
	}

	bool operator==(const possibleObject& other) const
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

};