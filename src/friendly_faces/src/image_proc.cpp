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
/*
	//template matching
	for (auto map_elem : getDB()) {
		for (Rect faceRect : FR_faces) {
			Mat testFrame = frame(faceRect).clone();
			imshow("testFrame", testFrame);

			Mat templ = map_elem.second.clone();
			resize(templ, templ, testFrame.size());
			imshow("templ", templ);

			Mat result(testFrame.cols - templ.cols + 1, testFrame.rows - templ.rows + 1, CV_32FC1);

			matchTemplate(testFrame, templ, result, CV_TM_SQDIFF);
			normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

			double minVal, maxVal;
			Point minLoc, maxLoc;
			Point matchLoc;

			minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

			//SQDIFF & SQDIFF_NORMED
			matchLoc = minLoc;

			Rect drawRect = Rect(matchLoc.x + faceRect.x, matchLoc.y + faceRect.y, templ.cols, templ.rows);
			imshow("found", frame(drawRect));

			//rectangle(frame, drawRect, Scalar(0, 0, 255));
		}
	}
*/
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
