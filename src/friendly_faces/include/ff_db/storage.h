#ifndef STORAGE_H
#define STORAGE_H

#include <string>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

void learn(string name, Mat face);

Mat getDB(string name);

string findFriend(Mat frame);

void reset();


#endif
