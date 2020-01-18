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

void graphing(int h_bins, int s_bins, int l_bins, vector<Mat> hsv_planes)
{
	int histSize[] = { h_bins, s_bins, l_bins };

	//Hue 0-179, sat 0-255, upper exclusive
	float Histranges[] = { 0, 256 };

	const float* ranges[] = { Histranges };

	bool uniform = true, accumulate = false;

	vector<Mat> hsvHists = { Mat(), Mat(), Mat() };
	for(int i = 0; i < hsv_planes.size(); i++)
	{
		calcHist(&hsv_planes[i], 1, 0, Mat(), hsvHists[i], 1, histSize, ranges, uniform, accumulate);
	}

	int hist_w = 512, hist_h = 400;
	int h_bin_w = cvRound((double)hist_w / histSize[0]);
	int s_bin_w = cvRound((double)hist_w / histSize[1]);
	int l_bin_w = cvRound((double)hist_w / histSize[2]);
	int bin_w[] = { h_bin_w, s_bin_w, l_bin_w };

		
	vector<Mat> histGraphs = { Mat(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0)), Mat(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0)),  Mat(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0)) };
	for(int i = 0; i < histGraphs.size(); i++)
	{
		Scalar color;
		switch(i)
		{
		case 0: color = Scalar(255, 0, 0); break;
		case 1: color = Scalar(0, 255, 0); break;
		case 2: color = Scalar(0, 0, 255); break;
		}
		normalize(hsvHists[i], hsvHists[i], 0, histGraphs[i].rows, NORM_MINMAX, -1, Mat());
		for (int h = 1; h < histSize[i]; h++)
		{
			line(histGraphs[i], 
			     Point(bin_w[i] *(h-1),hist_h - cvRound(hsvHists[i].at<float>(h-1))),
			     Point(bin_w[i]*h,hist_h - cvRound(hsvHists[i].at<float>(h))),
			     color);
		}
		imshow(("Graph" + std::to_string(i)), histGraphs[i]);
	}
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

vector<int> changeDetect(vector<float> change_values, float threshold, bool topTwo = false)
{
	cout << "Threshold: " << threshold << endl;
	if(!topTwo)
	{
		int max = 0, min = change_values.size();
		for (int i = 0; i < change_values.size(); ++i)
		{
			auto val = change_values[i];
			if (val > threshold)
			{
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
		return vector<int>({ min, max });
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

int main(int argc, char* argv[])
{
	VideoCapture cap(CAP_DSHOW);

	VideoWriter videoWriter;
	int value1;
	if (Initialise_Camera(cap, videoWriter, value1)) return value1;

	/*CONTROLS*/
	namedWindow("Control", WINDOW_AUTOSIZE);

	int iAlpha = 9;
	int iHThreshold = 50;
	int iVThreshold = 50;
	createTrackbar("Alpha", "Control", &iAlpha, 10);
	createTrackbar("HThreshold", "Control", &iHThreshold, 100);
	createTrackbar("VThreshold", "Control", &iVThreshold, 100);
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

		int topBorder = 1, bottomBorder = topBorder;
		int leftBorder = 1, rightBorder = topBorder;

		copyMakeBorder(frame, frame, topBorder, bottomBorder, leftBorder, rightBorder, BORDER_CONSTANT, Scalar(0, 0, 0));

		vector<Mat> bgr_planes;
		split(frame, bgr_planes);
		Mat maxRGB = max(bgr_planes[2], max(bgr_planes[1], bgr_planes[0]));
		Mat minRGB = min(bgr_planes[2], max(bgr_planes[1], bgr_planes[0]));
		Mat lPlane = (maxRGB - minRGB);
		imshow("lightness", lPlane);
		
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
		auto colTriggers = changeDetect(colChange, colThresh);

		float historyAlpha = 0.2;
		
		topHistory[0] = topHistory[1];
		topHistory[1] = rowTriggers[0];
		topHistory[boxHistoryCount-1] = applyExponentialFilter(topHistory,historyAlpha)[boxHistoryCount-1];

		botHistory[0] = botHistory[1];
		botHistory[1] = rowTriggers[1];
		botHistory[boxHistoryCount-1] = applyExponentialFilter(botHistory, historyAlpha)[boxHistoryCount-1];

		leftHistory[0] = leftHistory[1];
		leftHistory[1] = colTriggers[0];
		leftHistory[boxHistoryCount - 1] = applyExponentialFilter(leftHistory, historyAlpha)[boxHistoryCount - 1];

		rightHistory[0] = rightHistory[1];
		rightHistory[1] = colTriggers[1];
		rightHistory[boxHistoryCount - 1] = applyExponentialFilter(rightHistory, historyAlpha)[boxHistoryCount - 1];

		int top = topHistory[boxHistoryCount - 1];
		int bot = botHistory[boxHistoryCount - 1];
		int left = leftHistory[boxHistoryCount - 1];
		int right = rightHistory[boxHistoryCount - 1];

		drawLineGraph(rowChange, Scalar(255, 115, 115), h_bin_w, &outputFrame, false,10);
		drawLineGraph(colChange, Scalar(115, 115, 255), v_bin_w, &outputFrame, true,20);

		Mat objMask = Mat(frame.rows, frame.cols, CV_8UC1,Scalar(0,0,0));
		rectangle(objMask, Point(left, bot), Point(right, top), Scalar(255, 255,255),FILLED);
		Mat justObject;
		outputFrame.copyTo(justObject,objMask);
		imshow("Just object", justObject);

		line(outputFrame, Point(left, bot), Point(right, top), Scalar(0, 255, 0), 3);
		line(outputFrame, Point(left, top), Point(right, bot), Scalar(0, 255, 0), 3);

		//line(outputFrame, Point(h_bin_w * left, 0), Point(h_bin_w * left, graph_h), Scalar(0, 255, 0), 3);
		//line(outputFrame, Point(h_bin_w * right, 0), Point(h_bin_w * right, graph_h), Scalar(0, 255, 0), 3);
		
		/*for(auto it = rowTriggers.begin(); it != rowTriggers.end(); ++it)
		{
			line(outputFrame, Point(0, h_bin_w * (*it)), Point(graph_w, h_bin_w * (*it)), Scalar(0, 255, 0), 3);
		}

		for (auto it = colTriggers.begin(); it != colTriggers.end(); ++it)
		{
			line(outputFrame, Point(h_bin_w * (*it),0), Point(h_bin_w * (*it),graph_h), Scalar(0, 255, 0), 3);
		}*/
		
		//graphing(h_bins, s_bins, l_bins, hsv_planes);
		
		
		imshow("Source image", outputFrame);
	}
	return 0;
}
