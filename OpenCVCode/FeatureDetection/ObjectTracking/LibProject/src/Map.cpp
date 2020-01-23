#include "Map.h"

#include <opencv2/core.hpp>
#include <Statistics.h>

using namespace cv;

Map::Map(int viewDistance)
{
	int size = 2 * viewDistance + 1;
	map = Mat(size, size, CV_8UC1, Scalar(0, 0, 0));
}

void Map::increaseSize(int additional)
{
	copyMakeBorder(map, map, additional, additional, additional, additional, BORDER_CONSTANT, Scalar::all(0));
}

void Map::addMeasurement(Pose currentPose, float depth, Size2f& maxMinAngles)
{
	int x = (map.cols / 2) + 1 + currentPose.x, y = (map.rows / 2) + 1 + currentPose.y;
	const int distance = cvRound(depth);

	while(x+distance > map.cols || x - distance < 0 || y + distance > map.rows || y - distance < 0)
	{
		increaseSize(distance);
		x = (map.cols / 2) + 1 + currentPose.x, y = (map.rows / 2) + 1 + currentPose.y;
	}

	maxMinAngles.width *= -1;
	maxMinAngles.height *= -1;
	
	
	int x0 = x + (depth * cos(currentPose.heading + maxMinAngles.width));
	int y0 = y + (depth * sin(currentPose.heading + maxMinAngles.width));
	int x1 = x + (depth * cos(currentPose.heading + maxMinAngles.height));
	int y1 = y + (depth * sin(currentPose.heading + maxMinAngles.height));
	
	drawIncrementingLine(x0, y0, x1, y1);
}

void Map::incrementPixel(int x, int y)
{
	auto pixel = map.at<uchar>(x, y);
	if(pixel < 255)
	{
		pixel = (pixel + 1);
		map.at<uchar>(x, y) = pixel;
	}
}

void Map::drawLineLow(int x0, int y0, int x1, int y1)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	int yi = 1;

	if(dy < 0)
	{
		yi = -1;
		dy = -dy;
	}

	int D = 2 * dy - dx;
	int y = y0;

	for(int x = x0; x <= x1; ++x)
	{
		incrementPixel(x, y);
		if(D > 0)
		{
			y = y + yi;
			D = D - 2 * dx;
		}
		D = D + 2 * dy;
	}
}

void Map::drawLineHigh(int x0, int y0, int x1, int y1)
{
	int dx = x1 - x0;
	int dy = y1 - y0;
	int xi = 1;

	if (dx < 0)
	{
		xi = -1;
		dx = -dx;
	}

	int D = 2 * dx - dy;
	int x = x0;

	for (int y = y0; y <= y1; ++y)
	{
		incrementPixel(x, y);
		if (D > 0)
		{
			x = x + xi;
			D = D - 2 * dy;
		}
		D = D + 2 * dx;
	}
}

void Map::drawIncrementingLine(int x0, int y0, int x1, int y1)
{
	if(abs(y1 - y0) < abs(x1 - x0))
	{
		if(x0 > x1)
		{
			drawLineLow(x1, y1, x0, y0);
		} else
		{
			drawLineLow(x0, y0, x1, y1);
		}
	} else
	{
		if(y0 > y1)
		{
			drawLineHigh(x1, y1, x0, y0);
		}
		else
		{
			drawLineHigh(x0, y0, x1, y1);
		}
	}
}

float Pose::rotateDeg(float angle)
{
	return heading = radBound(heading + degToRad(angle));
}
