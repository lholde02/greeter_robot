#include "ff_proc/frame_proc.h"

FFFrameProcessor::FFFrameProcessor() {
	if (!face_cascade.load("res/haarcascade_frontalface_alt.xml")) {
		printf("error loading face cascade\n");
		return;
	}
}

vector<Rect> FFFrameProcessor::process(Mat frame) {
	vector<Rect> faces;

	Mat process = frame.clone();
	resize(process, process, Size(160, 120));

	medianBlur(process, process, 3);

	Mat frame_gray;
	cvtColor(process, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	face_cascade.detectMultiScale(frame_gray, faces);

	return faces;
}
