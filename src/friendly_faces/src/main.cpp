#include <string>
#include <unordered_map>
#include <algorithm>

#include <time.h>
#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "ros/ros.h"

#include "ff_text/positioning.h"
#include "ff_db/storage.h"
#include "ff_proc/image_proc.h"

using namespace cv;
using namespace std;
using namespace ros;

void run() {
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	VideoCapture stream(0);
	if (!stream.isOpened()) {
		printf("cannot open camera\n");
		return;
	}

	loadPresets();

	CascadeClassifier face_cascade;

	if (!face_cascade.load("res/haarcascade_frontalface_alt.xml")) {
		printf("error loading face cascade\n");
		return;
	}

	int fps;
	int frames;
	time_t start, end;
	time(&start);

	Mat frame;
	Mat process;
	Mat face(250, 250, CV_8UC3);
	while (stream.read(frame)) {
		process = frame.clone();
		resize(process, process, Size(160, 120));

		medianBlur(process, process, 3);

		vector<Rect> faces;
		Mat frame_gray;
		cvtColor(process, frame_gray, COLOR_BGR2GRAY);
		equalizeHist(frame_gray, frame_gray);

		face_cascade.detectMultiScale(frame_gray, faces);

		if (!faces.empty()) {
			for (int i = 0; i < faces.size(); i++) {
				Point p1(faces[i].x, faces[i].y);
				Point p2(faces[i].x + faces[i]. width, faces[i].y + faces[i].height);
				p1.x *= 4;
				p1.y *= 4;
				p2.x *= 4;
				p2.y *= 4;
				Rect faceRect(p1, p2);

				face = frame(faceRect).clone();
				resize(face, face, Size(250, 250));

				string name;
				if ((name = findFriend(face)).size() != 0) {
					printf("found %s\n", name.c_str());
				}

				frameText(frame, name.c_str(), faceRect);
				rectangle(frame, faceRect, Scalar(0, 255, 0));
			}
			reset();
		}

		frames++;
		time(&end);

		if (difftime(end, start) >= 1) {
			time(&start);
			fps = frames;
			frames = 0;
		}

		text(frame, to_string(fps).c_str(), Point(0, frame.size().height - 5));
		imshow("cam", frame);

		if (waitKey(10) >= 0) {
			break;
		}
		spinOnce();
	}

	destroyWindow("cam");
}

int main(int argc, char** argv) {
	init(argc, argv, "project_node");
	run();
	return 0;
}
