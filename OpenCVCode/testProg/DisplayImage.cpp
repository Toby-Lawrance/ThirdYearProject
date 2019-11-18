#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
int main(int argc, char** argv)
{
	VideoCapture cap(CAP_DSHOW);

	if (!cap.isOpened())
	{
		cout << "Cannot open webcam" << endl;
		cap.open(0);
		if (!cap.isOpened())
		{
			cout << "Really can't open it" << endl;
			return -1;
		}
	}
	while (true)
	{
		Mat image;
		bool bSuccess = cap.read(image);

		if (!bSuccess)
		{
			cout << "Stream ended" << endl;
			break;
		}
		if (waitKey(30) == 27) break;

		medianBlur(image, image, 5);
		GaussianBlur(image, image, Size(5, 5),3.0);
		namedWindow("Display Image", WINDOW_AUTOSIZE);
		imshow("Display Image", image);
	}
    return 0;
}
