#include <iostream>
#include <chrono>
#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace chrono;

void opticalFlow(Mat imgTmp, vector<Scalar> colors, vector<Point2f> p0, vector<Point2f> p1, Mat imgGray, vector<Point2f>& good_new)
{
	goodFeaturesToTrack(imgTmp, p0, 100, 0.3, 7);
	// Create a mask image for drawing purposes
	Mat mask = Mat::zeros(imgTmp.size(), imgTmp.type());
	vector<uchar> status;
	vector<float> err;
	TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
	calcOpticalFlowPyrLK(imgTmp, imgGray, p0, p1, status, err, Size(15, 15), 2, criteria);
	for (uint i = 0; i < p0.size(); i++)
	{
		// Select good points
		if (status[i] == 1) {
			good_new.push_back(p1[i]);
			// draw the tracks
			line(mask, p1[i], p0[i], colors[i], 2);
			circle(imgGray, p1[i], 5, colors[i], -1);
		}
	}

	Mat img;
	add(imgGray, mask, img);

	imshow("Frame", img);
}

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
	cap.set(CAP_PROP_FPS, 10);

	int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH)); //get the width of frames of the video
	int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT)); //get the height of frames of the video
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

int main(int argc, char* argv[])
{
	VideoCapture cap(CAP_DSHOW);

	VideoWriter videoWriter;
	int value1;
	if (Initialise_Camera(cap, videoWriter, value1)) return value1;

	namedWindow("Control", WINDOW_AUTOSIZE);

	//int maxFeatures = 10;
	//createTrackbar("maxFeatures", "Control", &maxFeatures, 100);
	/*
	int lowThresh = 10;
	int highThresh = 100;
	createTrackbar("Canny Low Threshold", "Control", &lowThresh, 500);
	createTrackbar("Canny High Threshold", "Control", &highThresh, 500);

	int houghThresh = 100;
	createTrackbar("Hough Threshold", "Control", &houghThresh, 500);

	int lineLength = 100;
	createTrackbar("Hough lineLength", "Control", &lineLength, 500);

	int lineGap = 10;
	createTrackbar("Hough lineGap", "Control", &lineGap, 100);
	*/
	int approxEpsilon = 3;
	createTrackbar("Approx Poly", "Control", &approxEpsilon, 10);
	
	/*
	int minArea = 10000;
	createTrackbar("MinArea", "Control", &minArea, 1000000);
	*/

	int HarrisBlockSize = 13;
	int HarriskSize = 8;
	int HarrisK = 10;
	createTrackbar("HarrisBlock", "Control", &HarrisBlockSize, 100);
	createTrackbar("HarriskSize", "Control", &HarriskSize, 31);
	createTrackbar("HarrisK", "Control", &HarrisK, 100);
	
	Mat imgTmp;
	cap.read(imgTmp);
	cvtColor(imgTmp, imgTmp, COLOR_BGR2GRAY);
	
	double activeFrameRate;

	high_resolution_clock hrc;
	auto lastFrame = hrc.now();

	auto orb = ORB::create();
	auto bgs = createBackgroundSubtractorMOG2();

	// Create some random colors
	vector<Scalar> colors;
	RNG rng;
	for (int i = 0; i < 100; i++)
	{
		int r = rng.uniform(0, 256);
		int g = rng.uniform(0, 256);
		int b = rng.uniform(0, 256);
		colors.push_back(Scalar(r, g, b));
	}
	vector<Point2f> p0, p1;
	while(true)
	{
		Mat imgOriginal;
		bool bSuccess = cap.read(imgOriginal);

		if (!bSuccess)
		{
			cout << "Stream ended" << endl;
			break;
		}

		//orb->setMaxFeatures(maxFeatures);
		
		if (waitKey(30) == 27) break;

		Mat imgGray;
		cvtColor(imgOriginal, imgGray, COLOR_BGR2GRAY);

		Mat imgCorner;
		int kernelSize = HarriskSize % 2 == 1 ? HarriskSize : HarriskSize + 1;
		kernelSize = kernelSize > 31 ? 31 : kernelSize;
		cornerHarris(imgGray,imgCorner,HarrisBlockSize,kernelSize,((double)HarrisK)/100.0,BORDER_DEFAULT);
		Mat contourFinding;
		imgCorner.convertTo(contourFinding, CV_8UC1);
		vector<vector<Point>> contours;
		findContours(contourFinding, contours,RETR_TREE, CHAIN_APPROX_SIMPLE);

		cout << contours.size() << endl;

		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());

		for (size_t i = 0; i < contours.size(); ++i)
		{
			approxPolyDP(contours[i], contours_poly[i], approxEpsilon, true);
			boundRect[i] = boundingRect(contours_poly[i]);
		}

		for (size_t i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
			drawContours(imgOriginal, contours_poly, (int)i, color);
			rectangle(imgOriginal, boundRect[i].tl(), boundRect[i].br(), color, 2);
		}

		vector<Point> rectangleCentres;
		for (auto rect : boundRect)
		{
			int avgX = (rect.tl().x + rect.br().x) / 2;
			int avgY = (rect.tl().y + rect.br().y) / 2;
			rectangleCentres.push_back(Point(avgX,avgY));
		}

		Rect r = boundingRect(rectangleCentres);
		rectangle(imgOriginal, r.tl(), r.br(), Scalar(255, 0, 0), 3);

		/*
		//Mat mask;
		//bgs->apply(imgOriginal, mask);
		//vector<Point2f> good_new;
		//opticalFlow(imgTmp, colors, p0, p1, imgGray, good_new);

		//Processing time

		Mat blurred;
		GaussianBlur(imgGray, blurred, Size(3,3), 0);
		Mat lappy;
		Laplacian(blurred, lappy, CV_16S);
		convertScaleAbs(lappy, lappy);
		Mat edges;
		Canny(lappy, edges, lowThresh, highThresh);

		vector<vector<Point>> contours;
		findContours(edges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());

		for(size_t i = 0; i < contours.size(); ++i)
		{
			approxPolyDP(contours[i], contours_poly[i],approxEpsilon, true);
			boundRect[i] = boundingRect(contours_poly[i]);
		}
		
		vector<Vec4f> lines;
		HoughLinesP(edges, lines, 1, CV_PI / 180, houghThresh, lineLength, lineGap);

		cvtColor(edges, edges, COLOR_GRAY2BGR);
		
		for (auto lin : lines)
		{
			Point2f pt1, pt2;
			pt1.x = lin[0];
			pt1.y = lin[1];
			pt2.x = lin[2];
			pt2.y = lin[3];
 			line(edges, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
		}

		Mat Drawing = edges.clone();
		
		for (size_t i = 0; i < contours.size(); i++)
		{
			if(boundRect[i].size().area() < 10000)
			{
				continue;
			}
			Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
			drawContours(Drawing, contours_poly, (int)i, color);
			rectangle(Drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
		}

		*/


		//FPS counter
		auto timeNow = hrc.now();
		activeFrameRate = floor((1000.0 / duration_cast<milliseconds>(timeNow - lastFrame).count()) * 100) / 100;
		lastFrame = timeNow;

		string fps = std::to_string(activeFrameRate) + "FPS";
		int font = FONT_HERSHEY_SCRIPT_SIMPLEX;
		Size textSize = getTextSize(fps, font, 1, 3, 0);
		Point textLoc(0, textSize.height);
		putText(imgOriginal, fps, textLoc, font, 1, Scalar::all(75), 2, 8);


		//Writing
		videoWriter.write(imgOriginal);
		imshow("Original", imgOriginal);
		imshow("Gray", imgGray);
		imshow("Corner", imgCorner);
		//imshow("Bounding", Drawing);
		//imshow("Edges", edges);
		//imshow("Laplacian", lappy);

		imgTmp = imgGray.clone();
		//p0 = good_new;
	}
}

void orbing(Mat& img,Ptr<ORB> orb, int scale = 25)
{
	vector<KeyPoint> kps;
	orb->detect(img, kps);
	for (auto kp : kps)
	{
		circle(img, kp.pt, scale, Scalar(0, 0, 200));
		//Draw angleLine
		Point2f endPoint(kp.pt.x + scale * sinf(kp.angle), kp.pt.y + scale * cosf(kp.angle));
		line(img, kp.pt, endPoint, Scalar(0, 150, 0));
	}
	
}