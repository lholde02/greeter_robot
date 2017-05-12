#include "image_proc.h"
#include "storage.h"

FFFrameProcessor::FFFrameProcessor() {
	if (!face_cascade.load("res/haarcascade_frontalface_default.xml")) {
		printf("error loading face cascade\n");
		return;
	}
}

int upscale;

vector<pair<Rect, string>> FFFrameProcessor::process(Mat frame) {
	upscale = frame.cols / 160;
	//haar cascades
	vector<Rect> FR_faces;

	Mat process;
	resize(frame, process, Size(160, 120));

	Mat frame_gray;
	cvtColor(process, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	face_cascade.detectMultiScale(frame_gray, FR_faces, 1.1, 4, 0 | CV_HAAR_SCALE_IMAGE, Size(10, 10), Size(frame.cols / 3, frame.rows / 3));

	for_each(FR_faces.begin(), FR_faces.end(), [](Rect& faceRect){
		faceRect.x *= upscale;
		faceRect.y *= upscale;
		faceRect.width *= upscale;
		faceRect.height *= upscale;
	});

	vector<pair<Rect, string>> pairing;

	for (Rect rect : FR_faces) {
		pairing.push_back(pair<Rect, string>(rect, findFriend(frame(rect))));
	}
	reset();

	return pairing;
}

double frameCompare(Mat frame1, Mat frame2) {
	if (frame1.rows <= 0 || frame1.rows != frame2.rows || frame2.cols <= 0 || frame2.cols != frame2.cols) {
		return -1;
	}

	double error = norm(frame1, frame2, CV_L1);
	error /= frame1.rows * frame1.cols;
	return error;
}

bool vector_contains(vector<pair<Rect, string>> vec, pair<Rect, string> elem) {
	for (auto vec_elem : vec) {
		if (elem.second == vec_elem.second) {
			return true;
		}
	}
	return false;
}
