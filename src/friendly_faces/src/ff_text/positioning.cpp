#include "opencv2/opencv.hpp"

#include "ff_text/positioning.h"

using namespace cv;

void text(Mat frame, const char* str, Point pt) {
	putText(frame, str, pt, FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 0));
}

void frameText(Mat frame, const char* str, Rect rect) {
	int baseline = 0;
	Size sz = getTextSize(str, FONT_HERSHEY_SIMPLEX, 0.75, 1, &baseline);
	Point pt(rect.x + ((rect.width - sz.width) / 2), rect.y + rect.height - (sz.height / 2));
	text(frame, str, pt);
}
