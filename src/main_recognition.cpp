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
	ros::Publisher recognition_pub = n.advertise<std_msgs::String>("face_recognition", 1000);
	ros:Rate loop_rate(10);
	int count = 0;

	ROS_INFO("Making an instance of face recognition\n");
        Face_Recognition face_recognition = Face_Recognition(n);

	while (ros::ok()) {
		ROS_INFO("Attempting to recognize a face\n");
                int label = face_recognition.recognize_faces();
                if (label == -1) {
                        ROS_INFO("Person unknown, learning them\n");
			std_msgs::String msg;
                        std::stringstream ss;
                        ss << "TODONameHere" << count;
                        msg.data = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
                        recognition_pub.publish(msg);

                } else if (label >= 0) {
                        ROS_INFO("Hello person %i\n!", label);
		} else {
			ROS_INFO("No person seen\n");
		}
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	return 0;
}
