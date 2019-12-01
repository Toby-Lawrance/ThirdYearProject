#pragma once
#include <vector>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <chrono>

using namespace std;
using namespace cv;

class TimeObjInf
{
public:
	chrono::system_clock::time_point now;
	Point2d loc;
	double area;
	TimeObjInf(Point2d l, double a) : now(chrono::system_clock::now()), loc(l), area(a) {}
};

class ObjInfo
{
	vector<TimeObjInf> infs;

public:
	ObjInfo() {}
	ObjInfo(TimeObjInf t);
	ObjInfo(double x, double y, double a);
};

