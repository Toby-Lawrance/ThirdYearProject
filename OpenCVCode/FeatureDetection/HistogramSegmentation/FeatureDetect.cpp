#include <iostream>
#include <chrono>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;
using namespace chrono;

int frame_width,frame_height;

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
		Mat mask = Mat(img.rows, img.cols, CV_8UC1,Scalar(0,0,0));
		imshow("Mean mask", mask);
		rectangle(mask, bounds, Scalar::all(255), FILLED);
		auto avg = mean(img, mask);
		avgVal = avg[0];
		return avgVal;
	}

	bool overlap(const possibleObject &other) const
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

	bool operator<(const possibleObject &other) const
	{
		if(avgVal == other.avgVal)
		{
			return bounds.area() < other.bounds.area();
		}

		return avgVal < other.avgVal;
	}

	bool operator==(const possibleObject &other) const
	{
		if(bounds != other.bounds)
		{
			return false;
		}

		if(avgVal != other.avgVal)
		{
			return false;
		}

		return true;
	}
	
};

bool Initialise_Camera(VideoCapture cap, VideoWriter& videoWriter, int& value1)
{
	if (!cap.isOpened())
	{
		cout << "Cannot open webcam" << endl;
		cap.open(0);
		if (!cap.isOpened())
		{
			cout << "Really can't open it" << endl;
			value1 = -1;
			return true;
		}
	}

	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT, 360);
	cap.set(CAP_PROP_FPS, 60);

	frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH)); //get the width of frames of the video
	frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT)); //get the height of frames of the video
	int frames_per_second = static_cast<int>(cap.get(CAP_PROP_FPS));//cap.get(CAP_PROP_XI_FRAMERATE);

	cout << "Frame dimensions: " << frame_width << "x" << frame_height << endl;
	cout << "FPS: " << frames_per_second << endl;

	Size frame_size(frame_width, frame_height);

	videoWriter = VideoWriter("tracking.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),
		frames_per_second, frame_size, true);

	if (!videoWriter.isOpened())
	{
		cout << "Cannot save video to file" << endl;
		cin.get();
		value1 = -1;
		return true;
	}
	return false;
}



Mat increaseDistance(Mat img,double thresh = 200)
{
	Mat edited;
	threshold(img, edited, thresh, 255, img.type());
	return edited;
}

Mat red(Mat bgrImg)
{
	vector<Mat> bgr_planes;
	split(bgrImg, bgr_planes);
	return max(bgr_planes[2] - 0.5 * bgr_planes[1] - 0.5 * bgr_planes[0],0);
}

Mat blue(Mat bgrImg)
{
	vector<Mat> bgr_planes;
	split(bgrImg, bgr_planes);
	return max(bgr_planes[1] - 0.5 * bgr_planes[0] - 0.5 * bgr_planes[0], 0);
}

Mat green(Mat bgrImg)
{
	vector<Mat> bgr_planes;
	split(bgrImg, bgr_planes);
	return max(bgr_planes[0] - 0.5 * bgr_planes[1] - 0.5 * bgr_planes[2], 0);
}

Mat applyChannelFilter(Mat bgrImg)
{
	return max(red(bgrImg), max(green(bgrImg), blue(bgrImg)));
}

float calcThreshold(vector<float> data,float factor = 100.0)
{
	float max = 0;
	float min = FP_INFINITE;

	for(auto it = data.begin(); it != data.end(); ++it)
	{
		float val = *it;
		if(val > max)
		{
			max = val;
		}
		if(val < min)
		{
			min = val;
		}
	}

	return (max - min)/factor;
}

vector<float> applyExponentialFilter(vector<float> data,float alpha)
{
	vector<float> filtered = vector<float>(data);
	for(int i = 1; i < data.size(); ++i)
	{
		filtered[i] = alpha * filtered[i - 1] + (1 - alpha) * data[i];
	}
	return filtered;
}

vector<int> applyExponentialFilter(vector<int> data, float alpha)
{
	vector<int> filtered = vector<int>(data);
	for (int i = 1; i < data.size(); ++i)
	{
		filtered[i] = alpha * filtered[i - 1] + (1 - alpha) * data[i];
	}
	return filtered;
}

vector<float> changeValue(vector<float> data)
{
	vector<float> change_values;
	for (auto it = data.begin(); it != (data.end() - 1); ++it)
	{
		change_values.push_back(abs(*(it + 1) - *it));
	}
	return change_values;
}

vector<int> removeAdjacent(vector<int> data)
{
	if (data.size() == 0) { return data; }
	std::sort(data.begin(), data.end());
	vector<int> cleansed;
	for(int i = 0; i < data.size()-1; ++i)
	{
		int val = data[i];
		if (data[i + 1] == val + 1)
		{
			int j = i+1;
			int last = val;
			while(j < data.size() && data[j] == last + 1)
			{
				last = data[j];
				j++;
			}
			int mid = ceil((last + val) / 2.0);
			cleansed.push_back(mid);
			i = j;
		} else
		{
			cleansed.push_back(val);
		}
	}
	return cleansed;
}

vector<possibleObject> getInterquartileRange(vector<possibleObject> data)
{
	if (data.size() < 3) { return data; }
	int fq = ceil(data.size() / 4.0);
	int tq = 3 * fq;
	fq = fq < 0 ? 0 : fq;
	fq = fq >= data.size() ? data.size() - 1 : fq;
	tq = tq >= data.size() ? data.size() - 1 : tq;

	vector<possibleObject> iqr;
	
	for(int i = fq; i <= tq; ++i)
	{
		iqr.push_back(data[i]);
	}

	return iqr;
}

pair<int, int> getFQTQ(vector<int> data) //Get First and Third quartile
{
	if (data.size() == 0) { return pair<int, int>(0, 0); }
	if (data.size() == 2) { return pair<int, int>(data[0], data[1]); }
	int fq,tq;
	std::sort(data.begin(), data.end());
	fq = (int)ceil(data.size() / 4.0);
	tq = 3 * fq;

	fq = fq < 0 ? 0 : fq;
	fq = fq >= data.size() ? data.size() - 1 : fq;
	tq = tq >= data.size() ? data.size() - 1 : tq;
	return pair<int, int>(data[fq], data[tq]);
}

vector<int> changeDetect(vector<float> change_values, float threshold, bool topTwo = false)
{
	if(!topTwo)
	{
		vector<int> triggerPoints;
		int max = 0, min = change_values.size();
		for (int i = 0; i < change_values.size(); ++i)
		{
			auto val = change_values[i];
			if (val > threshold)
			{
				triggerPoints.push_back(i);
				if(i > max)
				{
					max = i;
				}
				if(i < min)
				{
					min = i;
				}
			}
		}
		//return vector<int>({ min, max });
		return triggerPoints;
	} else
	{
		int maxIndex = 0, secondMaxIndex = 0;
		float maxVal = 0, secondMaxVal = 0;
		for (int i = 0; i < change_values.size(); ++i)
		{
			auto val = change_values[i];
			if (val > maxVal)
			{
				secondMaxIndex = maxIndex;
				maxIndex = i;
				secondMaxVal = maxVal;
				maxVal = val;
			}
			else if (val > secondMaxVal)
			{
				secondMaxIndex = i;
				secondMaxVal = val;
			}
		}
		return vector<int>({ maxIndex, secondMaxIndex });
	}
	
}

void drawLineGraph(vector<float> data,Scalar color, int bin_w, Mat* drawingSurface,bool vertical = true, float scale = 1.0)
{
	for (int i = 1; i < data.size(); i++)
	{
		Point p1 = vertical ? Point(bin_w * (i - 1), drawingSurface->rows - (data[i - 1])) : Point(cvRound(data[i - 1] * scale), bin_w * (i - 1));
		Point p2 = vertical ? Point(bin_w * (i), drawingSurface->rows - (data[i])) : Point(cvRound(data[i] * scale), bin_w * (i));
		line(*drawingSurface,p1,p2,color,2);
	}
}

vector<possibleObject> RemoveOverlaps(vector<possibleObject> sortedData)
{
	if (sortedData.size() < 2) { return sortedData; }
	reverse(sortedData.begin(), sortedData.end());
	for(auto it = sortedData.begin(); it != sortedData.end(); ++it)
	{
		if(it->avgVal == 0)
		{
			sortedData.erase(it);
			it = sortedData.begin();
		}
			
		for(auto cit = it; cit != sortedData.end(); ++cit)
		{
			if (it == cit)
			{continue;}

			bool overlaps = it->overlap(*cit);
				
			if(overlaps)
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
	int value1;
	if (Initialise_Camera(cap, videoWriter, value1)) return value1;

	/*CONTROLS*/
	namedWindow("Control", WINDOW_AUTOSIZE);
	int distanceThresh = 40;
	int iAlpha = 9;
	int iHThreshold = 100;
	int iVThreshold = 75;
	int contrastChange = 150;
	createTrackbar("Alpha", "Control", &iAlpha, 10);
	createTrackbar("HThreshold", "Control", &iHThreshold, 100);
	createTrackbar("VThreshold", "Control", &iVThreshold, 100);
	createTrackbar("DThreshold", "Control", &distanceThresh, 255);
	createTrackbar("Contrast", "Control", &contrastChange, 200);
	int h_bins = 179, s_bins = 255, l_bins = 255;
	/*END CONTROLS*/

	int boxHistoryCount = 2;
	vector<int> topHistory = vector<int>(boxHistoryCount), botHistory = vector<int>(boxHistoryCount), leftHistory = vector<int>(boxHistoryCount), rightHistory = vector<int>(boxHistoryCount);
	while(true)
	{
		Mat frame;
		bool readSuccess = cap.read(frame);

		if(!readSuccess)
		{
			cout << "Stream ended" << endl;
			break;
		}

		if (waitKey(1) == 27) break;

		int topBorder = 5, bottomBorder = topBorder;
		int leftBorder = 5, rightBorder = topBorder;

		copyMakeBorder(frame, frame, topBorder, bottomBorder, leftBorder, rightBorder, BORDER_CONSTANT, Scalar(0, 0, 0));

		
		vector<Mat> bgr_planes;
		split(frame, bgr_planes);
		Mat maxRGB = max(bgr_planes[2], max(bgr_planes[1], bgr_planes[0]));
		Mat minRGB = min(bgr_planes[2], max(bgr_planes[1], bgr_planes[0]));
		Mat lightnessPlane = (maxRGB - minRGB)/2.0;
		//imshow("True lightness", lightnessPlane);
		Mat lPlane = applyChannelFilter(frame);
		Mat distant = increaseDistance(lPlane,distanceThresh);
		//imshow("With thresholding",distant);
		Mat masked;
		lPlane.copyTo(masked, distant);
		lPlane = masked.clone();
		lPlane = lPlane * (contrastChange / 100.0);
		//imshow("lightness", lPlane);
		//imshow("Masked", masked);
		
		vector<float> lightnessRow = vector<float>(lPlane.rows);
		vector<float> lightnessCol = vector<float>(lPlane.cols);

		for(int i = 0; i < lightnessRow.size(); i++)
		{
			auto row = lPlane.row(i);
			float total = 0;
			for(int j = 0; j < row.cols; ++j)
			{
				auto pixel = row.at<uchar>(0, j);
				float value = static_cast<float>(pixel);
				total += value;
			}
			lightnessRow[i] = total/row.cols;
		}

		for (int i = 0; i < lightnessCol.size(); i++)
		{
			auto col = lPlane.col(i);
			float total = 0;
			for (int j = 0; j < col.rows; ++j)
			{
				auto pixel = col.at<uchar>(j, 0);
				float value = static_cast<float>(pixel);
				total += value;
			}
			lightnessCol[i] = total/col.rows;
		}
		
		Mat outputFrame = frame.clone();
		int graph_h = frame_height, graph_w = frame_width;

		int h_bin_w = cvRound((double)graph_h / (double)lightnessRow.size());
		int v_bin_w = cvRound((double)graph_w / (double)lightnessCol.size());

		vector<float> filteredRow = applyExponentialFilter(lightnessRow, (iAlpha / 10.0));
		vector<float> filteredCol = applyExponentialFilter(lightnessCol, (iAlpha / 10.0));
		
		drawLineGraph(filteredRow, Scalar(255, 0, 0), h_bin_w, &outputFrame, false);
		drawLineGraph(filteredCol, Scalar(0, 0, 255), v_bin_w, &outputFrame, true);

		auto rowChange = changeValue(filteredRow);
		auto colChange = changeValue(filteredCol);

		float rowThresh = calcThreshold(filteredRow,iHThreshold);
		float colThresh = calcThreshold(filteredCol,iVThreshold);
		
		auto rowTriggers = changeDetect(rowChange, rowThresh);
		rowTriggers = removeAdjacent(rowTriggers);
		//auto rowBorders = getFQTQ(rowTriggers);
		auto colTriggers = changeDetect(colChange, colThresh);
		colTriggers = removeAdjacent(colTriggers);
		//auto colBorders = getFQTQ(colTriggers);
		/*
		float historyAlpha = 0.2;
		
		topHistory[0] = topHistory[1];
		topHistory[1] = rowBorders.first;//rowTriggers[0];
		topHistory[boxHistoryCount-1] = applyExponentialFilter(topHistory,historyAlpha)[boxHistoryCount-1];

		botHistory[0] = botHistory[1];
		botHistory[1] = rowBorders.second;//rowTriggers[1];
		botHistory[boxHistoryCount-1] = applyExponentialFilter(botHistory, historyAlpha)[boxHistoryCount-1];

		leftHistory[0] = leftHistory[1];
		leftHistory[1] = colBorders.first;//colTriggers[0];
		leftHistory[boxHistoryCount - 1] = applyExponentialFilter(leftHistory, historyAlpha)[boxHistoryCount - 1];

		rightHistory[0] = rightHistory[1];
		rightHistory[1] = colBorders.second;//colTriggers[1];
		rightHistory[boxHistoryCount - 1] = applyExponentialFilter(rightHistory, historyAlpha)[boxHistoryCount - 1];

		int top = topHistory[boxHistoryCount - 1];
		int bot = botHistory[boxHistoryCount - 1];
		int left = leftHistory[boxHistoryCount - 1];
		int right = rightHistory[boxHistoryCount - 1];
		*/
		drawLineGraph(rowChange, Scalar(255, 115, 115), h_bin_w, &outputFrame, false,10);
		drawLineGraph(colChange, Scalar(115, 115, 255), v_bin_w, &outputFrame, true,50);

		vector<Point> intersects;
		
		for(auto it = rowTriggers.begin(); it != rowTriggers.end(); ++it)
		{
			line(outputFrame, Point(0, h_bin_w * (*it)), Point(graph_w, h_bin_w * (*it)), Scalar(0, 255, 0), 1);
			for (auto cit = colTriggers.begin(); cit != colTriggers.end(); ++cit)
			{
				Point intersect = Point(*cit, *it);
				intersects.push_back(intersect);
			}
		}
		for (auto it = colTriggers.begin(); it != colTriggers.end(); ++it)
		{
			line(outputFrame, Point(h_bin_w * (*it),0), Point(h_bin_w * (*it),graph_h), Scalar(0, 255, 0), 1);
		}

		vector<possibleObject> possible_objects;
		for(int i = 0; i < intersects.size()-1; ++i)
		{
			Point x = intersects[i];
			for(int j = i + 1; j < intersects.size(); ++j)
			{
				Point y = intersects[j];
				auto r = Rect(x, y);
				float viewArea = outputFrame.cols * outputFrame.rows;
				if(r.area() >= viewArea*0.05) //5% of view, possibly too harsh?
				{
					possible_objects.push_back(possibleObject(Rect(x, y)));
				}
				
			}
		}
		std::sort(possible_objects.begin(),possible_objects.end());
		
		auto middling = possible_objects;//unique(possible_objects.begin(), possible_objects.end());
		for(auto it = middling.begin(); it != middling.end(); ++it)
		{
			it->computeAvgVal(distant);
			if(it->avgVal <= 125) //Halfway value, meaning half or more of the box isn't on the object
			{
				it->avgVal = 0;
			} else if(it->avgVal > 230) //90% of the box is over the object
			{
				it->avgVal = 255;
			}
		}
		std::sort(middling.begin(), middling.end());
		middling = RemoveOverlaps(middling);
		

		Mat objMask = Mat(frame.rows, frame.cols, CV_8UC1, Scalar(0, 0, 0));
		for(auto pO: middling)
		{
			//cout << "Avg Val: " << pO.avgVal << " for:" << pO.bounds << endl;
			int xMax, yMax, xMin, yMin;
			xMax = max(pO.bounds.tl().x, pO.bounds.br().x);
			yMax = max(pO.bounds.tl().y, pO.bounds.br().y);

			xMin = min(pO.bounds.tl().x, pO.bounds.br().x);
			yMin = min(pO.bounds.tl().y, pO.bounds.br().y);

			line(outputFrame, Point(xMin, yMax), Point(xMax, yMin), Scalar(255, 0, 0), 3);
			line(outputFrame, Point(xMax, yMax), Point(xMin, yMin), Scalar(255, 0, 0), 3);
			
			rectangle(objMask, pO.bounds, Scalar(255,255,255), FILLED);
		}
		Mat justObject;
		frame.copyTo(justObject, objMask);
		imshow("Just object", justObject);
		//graphing(h_bins, s_bins, l_bins, hsv_planes);
		
		
		imshow("Source image", outputFrame);
		/*if(waitKey() == 27) break;
		for(int i = 0; i < 100; i++)
		{
			cout << endl;
		}*/
	}
	return 0;
}
