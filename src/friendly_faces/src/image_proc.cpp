#include "image_proc.h"
#include "storage.h"

FFFrameProcessor::FFFrameProcessor() {
	if (!face_cascade.load("res/haarcascade_frontalface_alt.xml")) {
		printf("error loading face cascade\n");
		return;
	}
}

vector<pair<Rect, string>> FFFrameProcessor::process(Mat frame) {
	//haar cascades
	vector<Rect> FR_faces;

	Mat process;
	resize(frame, process, Size(160, 120));
	medianBlur(process, process, 3);

	Mat frame_gray;
	cvtColor(process, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	face_cascade.detectMultiScale(frame_gray, FR_faces);

	for_each(FR_faces.begin(), FR_faces.end(), [](Rect& faceRect){
		faceRect.x *= 4;
		faceRect.y *= 4;
		faceRect.width *= 4;
		faceRect.height *= 4;
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
