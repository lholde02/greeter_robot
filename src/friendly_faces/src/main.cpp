#include <string>
#include <algorithm>

#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "ros/ros.h"

#include "sound_play/sound_play.h"

#include "text.h"
#include "storage.h"
#include "image_proc.h"
#include "fps.h"

using namespace cv;
using namespace std;
using namespace ros;
using namespace sound_play;

void run() {
	//system("rosrun sound_play say.py 'hello world'");

	VideoCapture stream(0);
	if (!stream.isOpened()) {
		printf("cannot open camera\n");
		return;
	}

	FFFrameProcessor frame_proc;

	FPSCounter fps;

	Mat frame;
	Mat face;
	while (stream.read(frame)) {
		vector<pair<Rect, string>> faces = frame_proc.process(frame);

		for (auto elem : faces) {
			rectangle(frame, elem.first, Scalar(0, 255, 0));

			face = frame(elem.first).clone();
			string name = elem.second;
			if (name.size() == 0) {
				imshow("stranger", face);
				waitKey(25);
				printf("What's your name stranger?\n");
				cin >> name;
				destroyWindow("stranger");
			}
			learn(name, face);
			frameText(frame, name.c_str(), elem.first);
		}

		if (!getDB().empty()) {
			imshow("face", getDB().begin()->second);
		}

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
	init(argc, argv, "friendly_faces_runner");

	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	run();
	return 0;
}
