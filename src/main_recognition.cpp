#include "face_detection.h"
#include "face_recognition.h"
#include <opencv2/contrib/contrib.hpp>
#include "std_msgs/String.h"
#include <sstream>


int main(int argc, char** argv) {
	init(argc, argv, "greeter_robot_recognition");
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);
	
	ros::NodeHandle n;

/*
	ros::Publisher robotsound_pub = n.advertise<std_msgs::String>("/sound_play/SoundRequest", 1);
	sound_play::SoundRequest str;
	str.arg = "Hello World";
	robotsound_pub.publish(str);
*/
	//SoundClient *soundhandle = new SoundClient;
	//soundhandle(soundhandle->wavepath + "/R2D2a.wav");
	//soundhandle->say("Hello World", soundhandle->voice);
/*
	SoundClient* soundclient = new sound_play::SoundClient(n, "/sound_play/SoundRequest");
	soundclient->say("Hello World", "voice_kal_diphone");
*/	
	ros::Publisher recognition_pub = n.advertise<std_msgs::String>("face_recognition", 1);
	ros:Rate loop_rate(10);
	int count = 0;

//	ROS_INFO("Making an instance of face recognition\n");
        Face_Recognition face_recognition = Face_Recognition(n);

	while (ros::ok()) {
//		ROS_INFO("Attempting to recognize a face\n");
                string name = face_recognition.recognize_faces();
                if (name == "unknown") {
                        ROS_INFO("Person unknown, learning them\n");
			std_msgs::String msg;
                        std::stringstream ss;
                        ss << "TODONameHere" << count;
                        msg.data = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
                        recognition_pub.publish(msg);
			ros::spinOnce();

                } else if (name != "noone") {
                        ROS_INFO("Hello %s\n!", name.c_str());
		}
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	return 0;
}
