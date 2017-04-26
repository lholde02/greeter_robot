#pragma once

#include <vector>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class FFFrameProcessor {
private:
	CascadeClassifier face_cascade;
public:
	FFFrameProcessor();

	vector<Rect> process(Mat frame);
};

double frameCompare(Mat frame1, Mat frame2);
