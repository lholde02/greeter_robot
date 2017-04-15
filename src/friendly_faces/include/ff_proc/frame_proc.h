#ifndef FF_FRAME_PROC_H
#define FF_FRAME_PROC_H

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

#endif
