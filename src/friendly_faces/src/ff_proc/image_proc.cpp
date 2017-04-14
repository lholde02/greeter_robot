#include "ff_proc/image_proc.h"

double frameCompare(Mat frame1, Mat frame2) {
	if (frame1.rows <= 0 || frame1.rows != frame2.rows || frame2.cols <= 0 || frame2.cols != frame2.cols) {
		return -1;
	}

	double error = norm(frame1, frame2, CV_L1);
	return error;
}
