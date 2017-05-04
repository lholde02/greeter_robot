#include "image_proc.h"

FFFrameProcessor::FFFrameProcessor() {
	if (!face_cascade.load("res/haarcascade_frontalface_alt.xml")) {
		printf("error loading face cascade\n");
		return;
	}
}

vector<Rect> prevFaces;

double minX;
double minY;
double maxH;
double maxW;

vector<Rect> FFFrameProcessor::process(Mat frame) {
	if (!prevFaces.empty()) {
		minX = frame.rows;
		minY = frame.cols;
		maxH = 0;
		maxW = 0;

		for_each(prevFaces.begin(), prevFaces.end(), [](Rect& faceRect) {
			faceRect.x *= 1;
			faceRect.y *= 1;
			faceRect.width *= 1;
			faceRect.height *= 1;

			if (faceRect.x < minX) {
				minX = faceRect.x;
			}
			if (faceRect.y < minY) {
				minY = faceRect.y;
			}
			if (faceRect.width > maxW) {
				maxW = faceRect.width;
			}
			if (faceRect.height > maxH) {
				maxH = faceRect.height;
			}
		});

		Rect cutRect(minX, minY, maxH, maxW);
		cout << cutRect << endl;
		frame = frame(cutRect);
	}

	vector<Rect> faces;

	Mat process = frame.clone();

	resize(process, process, Size(160, 120));

	Mat frame_gray;
	cvtColor(process, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	face_cascade.detectMultiScale(frame_gray, faces);

	for_each(faces.begin(), faces.end(), [](Rect& faceRect){
		faceRect.x *= 4;
		faceRect.y *= 4;
		faceRect.width *= 4;
		faceRect.height *= 4;
	});

	prevFaces = faces;

	return faces;
}

double frameCompare(Mat frame1, Mat frame2) {
	if (frame1.rows <= 0 || frame1.rows != frame2.rows || frame2.cols <= 0 || frame2.cols != frame2.cols) {
		return -1;
	}

	double error = norm(frame1, frame2, CV_L1);
	error /= frame1.rows * frame1.cols;
	return error;
}
