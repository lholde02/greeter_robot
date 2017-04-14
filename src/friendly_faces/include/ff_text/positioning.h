#ifndef FF_POSITIONING_H
#define FF_POSITIONING_H

#include "opencv2/opencv.hpp"

using namespace cv;

void text(Mat frame, const char* str, Point pt);

void frameText(Mat frame, const char* str, Rect rect);

#endif
