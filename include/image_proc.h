#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include "storage.h"
#include <vector>
#include <unordered_map>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class FFFrameProcessor {
private:
	CascadeClassifier face_cascade;
public:
	FFFrameProcessor();

	vector<pair<Rect, string>> process(Mat frame);
};

double frameCompare(Mat frame1, Mat frame2);

bool vector_contains(vector<pair<Rect, string>> vec, pair<Rect, string> elem);

#endif
