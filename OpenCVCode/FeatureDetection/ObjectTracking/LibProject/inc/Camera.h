#pragma once
#include <string>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

bool Initialise_Camera(VideoCapture& cap, VideoWriter& videoWriter, Mat& baseFrame, int width = 640, int height = 360, int fps = 30, std::string fileName = "tracking.avi");
