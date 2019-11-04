#include <iostream>
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui.hpp>  // OpenCV window I/O


using namespace std;
using namespace cv;


int main(int argc, char* argv[])
{
	VideoCapture cap(0);

	if(!cap.isOpened())
	{
		cout << "Cannot open webcam" << endl;
		return -1;
	}

	//cap.set(CAP_PROP_FRAME_HEIGHT, 320);
	//cap.set(CAP_PROP_FRAME_WIDTH, 320);

	int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH)); //get the width of frames of the video
	int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT)); //get the height of frames of the video
	int frames_per_second = 30;//cap.get(CAP_PROP_XI_FRAMERATE);

	Size frame_size(frame_width, frame_height);
	//int frames_per_second = 15;

	//Create and initialize the VideoWriter object 
	VideoWriter videoWriter("tracking.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),
		frames_per_second, frame_size, true);

	if(!videoWriter.isOpened())
	{
		cout << "Cannot save video to file" << endl;
		return -1;
	}
	
	namedWindow("Control", WINDOW_AUTOSIZE);

	int iLowH = 0;
	int iHighH = 9;

	int iLowS = 193;
	int iHighS = 255;

	int iLowV = 153;
	int iHighV = 255;

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);

	int LastX = -1;
	int LastY = -1;

	Mat imgTmp;
	cap.read(imgTmp);

	Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);
	
	while(true)
	{
		Mat imgOriginal;
		bool bSuccess = cap.read(imgOriginal);

		if(!bSuccess)
		{
			cout << "Stream ended" << endl;
			break;
		}
		if (waitKey(30) == 27) break;

		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
		GaussianBlur(imgHSV, imgHSV, Size(3, 3), 0);
		Mat imgThresholded;
		Mat imgEroded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgEroded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgEroded, imgEroded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgEroded, imgEroded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgEroded, imgEroded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		Moments moment = moments(imgEroded);

		double M01 = moment.m01;
		double M10 = moment.m10;
		double Area = moment.m00;

		if(Area > 10000)
		{
			int posX = M10 / Area;
			int posY = M01 / Area;

			if(LastX >= 0 && LastY >= 0 && posX >= 0 && posY >= 0)
			{
				line(imgLines, Point(posX, posY), Point(LastX, LastY), Scalar(0, 0, 255), 2);
			}

			LastX = posX;
			LastY = posY;
		}

		imgOriginal = imgOriginal + imgLines;
		
		videoWriter.write(imgOriginal);
		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Eroded", imgEroded);
		imshow("Original", imgOriginal); //show the original image
	}
	cap.release();
	videoWriter.release();
	return 0;
}