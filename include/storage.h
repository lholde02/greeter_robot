#ifndef STORAGE_H
#define STORAGE_H


#include <string>
#include <unordered_map>
#include <vector>
#include "image_proc.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

void learn(string name, Mat face);

unordered_map<string, Mat> getDB();

string findFriend(Mat frame);

void reset();

#endif
