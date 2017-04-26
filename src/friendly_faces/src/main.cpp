#include <string>
#include <algorithm>

#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "ros/ros.h"

#include "text.h"
#include "storage.h"
#include "image_proc.h"
#include "fps.h"

using namespace cv;
using namespace std;
using namespace ros;

void run() {
	VideoCapture stream(0);
	if (!stream.isOpened()) {
		printf("cannot open camera\n");
		return;
	}

	FFFrameProcessor frame_proc;

	FPSCounter fps;

	int lastFaces = 0;
	bool action = false;

	Mat frame;
	Mat faceMat(250, 250, CV_8UC3);
	while (stream.read(frame)) {
		vector<Rect> faces = frame_proc.process(frame);

		if (lastFaces != faces.size()) {
			action = faces.size() > lastFaces;
			lastFaces = faces.size();
		} else {
			action = false;
		}
		if (action) {
			printf("action\n");
		}

		int strangers = 0;
		int friends = 0;
		for (auto face : faces) {
			Point p1(face.x, face.y);
			Point p2(face.x + face. width, face.y + face.height);
			p1.x *= 4;
			p1.y *= 4;
			p2.x *= 4;
			p2.y *= 4;
			Rect faceRect(p1, p2);

			faceMat = frame(faceRect).clone();

			string name = findFriend(faceMat);
			strangers += name.size() == 0;
			friends += name.size() != 0;
		}
		printf("friends: %d\nstrangers:%d\n\n", friends, strangers);
/*
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
					if (action) {
						printf("hi there %s, you look good today\n", name.c_str());
					}
				} else {
					imshow("stranger", face);
					waitKey(100);
					printf("What's your name stranger?\n");
					cin >> name;
					destroyWindow("stranger");
				}

				learn(name, face);
				face = getDB(name);

				frameText(frame, name.c_str(), faceRect);
				rectangle(frame, faceRect, Scalar(0, 255, 0));
			}
			reset();
		}
*/

		fps.frame();

		text(frame, to_string(fps.getFPS()).c_str(), Point(0, frame.size().height - 5));
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

	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	run();
	return 0;
}
