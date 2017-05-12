#include <string>
#include <algorithm>
#include <stdio.h>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/action_client.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"

#include "bwi_kr_execution/ExecutePlanAction.h"

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
using namespace bwi_kr_execution;

string greetings[] = {"hello", "hey there", "how are you", "nice to see you again"};
int greetings_sz = 4;

string goodbyes[] = {"good bye", "see you later", "bye", "I'll miss you"};
int goodbyes_sz = 4;

string doors[] = {"d3_414b2"};//"d3_414b1", "d3_414a1", "d3_414a2"
int doors_sz = 1;

//init
SoundClient* client;

//loop stuff
bool idle = true;
FFFrameProcessor frame_proc;
FPSCounter fps;
vector<pair<Rect, string>> prevFaces;
Mat frame;

class SegbotProcessor {
private:
	bool processing = true;
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	SimpleActionClient<ExecutePlanAction>* c;
	SimpleActionClient<MoveBaseAction>* cc;
	
	bool first = true;
	bool killed = false;
	double lastIdle = -1;

	void callback(const sensor_msgs::ImageConstPtr& msg) {
		if (!processing) {
			_kill_idle();
			return;
		}

		CvImagePtr cv_ptr;
		try {
			cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		run(cv_ptr->image.clone());

		if (idle) {
			printf("idling\n");
			_idle();
		} else {
			_kill_idle();
		}
	}
	
	void run(Mat frame) {
		vector<pair<Rect, string>> faces = frame_proc.process(frame);

		Mat face;
		bool stranger = false;
		for (auto elem : faces) {
			rectangle(frame, elem.first, Scalar(0, 255, 0));

			face = frame(elem.first).clone();
			string name = elem.second;
			if (name.size() == 0) {
				_kill_idle();
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
		reset();

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

		idle = prevFaces.size() == faces.size() && faces.size() == 0;

		prevFaces = faces;

		if (waitKey(10) >= 0) {
			processing = false;
		}
	}
public:
	SegbotProcessor(NodeHandle& nh) : it(nh) {
		processing = true;
		image_sub = it.subscribe("/nav_kinect/rgb/image_raw", 1, &SegbotProcessor::callback, this);

		c = new SimpleActionClient<ExecutePlanAction>("action_executor/execute_plan", true);
		c->waitForServer();
		
		cc = new SimpleActionClient<MoveBaseAction>("move_base", true);
		cc->waitForServer();
	}
	
	~SegbotProcessor() {
		free(c);
		free(cc);
	}

	void _idle() {
		if (lastIdle == -1 || Time::now().toSec() - lastIdle > 1) {
			lastIdle = Time::now().toSec();
		} else {
			return;
		}
		if (first || c->getState().isDone()) {
			first = false;
			killed = false;
			
			//create new action
			NodeHandle privateNode("~");
			string door;
			privateNode.param<string>("door", door, doors[rand() % doors_sz]);
			
			printf("new idle goal: %s\n", door.c_str());
			
			ExecutePlanGoal goal;
			AspRule rule;
			AspFluent fluent;
			fluent.name = "not facing";
			
			fluent.variables.push_back(door);
			rule.body.push_back(fluent);
			goal.aspGoal.push_back(rule);
			
			c->sendGoal(goal);
		}
	}
	
	void _kill_idle() {
		lastIdle = -1;
		first = true;
		
		printf("cancelling goals\n");
		c->cancelAllGoals();
		
		if (killed) {
			return;
		}
		
		MoveBaseGoal goal;
		
		goal.target_pose.header.stamp = Time::now();
		goal.target_pose.header.frame_id = "/base_link";

		goal.target_pose.pose.position.x = 0;
		goal.target_pose.pose.position.y = 0;
		goal.target_pose.pose.position.z = 0;
		goal.target_pose.pose.orientation = createQuaternionMsgFromYaw(0);

		cc->sendGoal(goal);
		cc->waitForResult();
		
		killed = true;
		c->cancelAllGoals();//insurance
	}
};

int main(int argc, char** argv) {
	init(argc, argv, "friendly_faces_runner");

	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	NodeHandle nh;
	client = new SoundClient;

	SegbotProcessor sp(nh);
	spin();
	destroyWindow("cam");

	free(client);

	return 0;
}
