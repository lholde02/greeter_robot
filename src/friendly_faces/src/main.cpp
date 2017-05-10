#include <string>
#include <algorithm>
#include <stdio.h>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sound_play/sound_play.h"

#include "text.h"
#include "storage.h"
#include "image_proc.h"
#include "fps.h"

using namespace tf;
using namespace cv;
using namespace std;
using namespace ros;
using namespace cv_bridge;
using namespace actionlib;
using namespace sound_play;
using namespace move_base_msgs;

string greetings[] = {"hello", "hey there", "how are you", "nice to see you again"};
int greetings_sz = 4;

string goodbyes[] = {"good bye", "see you later", "bye", "I'll miss you"};
int goodbyes_sz = 4;

//init
SoundClient* client;

//loop stuff
bool idle = true;
FFFrameProcessor frame_proc;
FPSCounter fps;
vector<pair<Rect, string>> prevFaces;
Mat frame;

bool run(Mat frame) {
	vector<pair<Rect, string>> faces = frame_proc.process(frame);

	Mat face;
	bool stranger = false;
	for (auto elem : faces) {
		rectangle(frame, elem.first, Scalar(0, 255, 0));

		face = frame(elem.first).clone();
		string name = elem.second;
		if (name.size() == 0) {
			stranger = true;
			imshow("stranger", face);
			waitKey(25);
			client->say("Hello stranger, what is your name?");
			printf("What's your name stranger?\n");
			cin >> name;
			destroyWindow("stranger");
		}
		learn(name, face);
		frameText(frame, name.c_str(), elem.first);
	}

	fps.frame();

	text(frame, to_string(fps.getFPS()).c_str(), cv::Point(0, frame.size().height - 5));
	imshow("cam", frame);

	if (prevFaces.size() != faces.size() && !stranger) {
		for (auto elem : faces) {
			if (!vector_contains(prevFaces, elem)) {
	    		//elem in faces that's not in prevFaces
				//someone entered
				printf("entered frame: %s\n", elem.second.c_str());

				string cmd = greetings[rand() % greetings_sz] + " " + elem.second;
				client->say(cmd.c_str());
			}
		}
		for (auto elem : prevFaces) {
			if (!vector_contains(faces, elem)) {
	    			//elem in prevFaces that's not in faces
				//someone left
				printf("left frame: %s\n", elem.second.c_str());

				string cmd = goodbyes[rand() % goodbyes_sz] + " " + elem.second;
				client->say(cmd.c_str());
			}
		}
	}
	printf("hi\n");

	idle = prevFaces.size() == faces.size() && faces.size() == 0;

	prevFaces = faces;

	if (waitKey(10) >= 0) {
		return true;
	}
	//spinOnce();
	return false;
}

class SegbotProcessor {
private:
	bool processing;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	SimpleActionClient<MoveBaseAction>* ac;

	void callback(const sensor_msgs::ImageConstPtr& msg) {
		if (!processing) {
			return;
		}

		CvImagePtr cv_ptr;
		try {
			cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		if (run(cv_ptr->image.clone())) {
			processing = false;
		}

		if (idle) {
			printf("idling\n");
			go(0.5, 0, 0, 0);
		}
	}
public:
	SegbotProcessor(NodeHandle& nh) : it(nh) {
		processing = true;
		image_sub = it.subscribe("/nav_kinect/rgb/image_raw", 1, &SegbotProcessor::callback, this);

		ac = new SimpleActionClient<MoveBaseAction>("move_base", true);
	}
	
	~SegbotProcessor() {
		free(ac);
	}

	void go(float x, float y, float z, float yaw) {
		MoveBaseGoal goal;

		goal.target_pose.header.stamp = Time::now();
		goal.target_pose.header.frame_id = "/base_link";

		goal.target_pose.pose.position.x = x;
		goal.target_pose.pose.position.y = y;
		goal.target_pose.pose.position.z = z;
		goal.target_pose.pose.orientation = createQuaternionMsgFromYaw(yaw);

		ac->sendGoal(goal);
		ac->waitForResult();
	}
};

void launch() {
	VideoCapture stream(0);
	if (!stream.isOpened()) {
		printf("cannot open camera\n");
		return;
	}

	while (stream.read(frame)) {
		if (run(frame)) {
			break;
		}

		if (idle) {
			printf("idling\n");
		}
	}
}

int main(int argc, char** argv) {
	init(argc, argv, "friendly_faces_runner");

	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	NodeHandle nh;
	client = new SoundClient;

	//launch();
	SegbotProcessor sp(nh);
	spin();
	destroyWindow("cam");

	free(client);

	return 0;
}
