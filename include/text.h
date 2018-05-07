#ifndef TEXT_H
#define TEXT_H

#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sound_play/sound_play.h"
#include "text.h"
#include "opencv2/opencv.hpp"

using namespace cv;

void text(Mat frame, const char* str, Point pt);

void frameText(Mat frame, const char* str, Rect rect);

#endif
