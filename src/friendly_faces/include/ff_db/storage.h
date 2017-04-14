#ifndef STORAGE_H
#define STORAGE_H

#include <string>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

void loadPresets();

string findFriend(Mat frame);

void reset();


#endif
