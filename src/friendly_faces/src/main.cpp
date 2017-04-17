#include <string>
#include <algorithm>

#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "ros/ros.h"

#include "ff_text/positioning.h"
#include "ff_db/storage.h"
#include "ff_proc/image_proc.h"
#include "ff_proc/frame_proc.h"
#include "ff_monitor/fps.h"

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
	Mat face(250, 250, CV_8UC3);
	while (stream.read(frame)) {
		vector<Rect> faces = frame_proc.process(frame);

		if (lastFaces != faces.size()) {
			lastFaces = faces.size();
			action = true;
		} else {
			action = false;
		}
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

		fps.frame();

		text(frame, to_string(fps.getFPS()).c_str(), Point(0, frame.size().height - 5));
		imshow("cam", frame);
		imshow("face", face);

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
