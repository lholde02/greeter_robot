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
        Face_Recognition face_recognition = Face_Recognition();

	while (ros::ok()) {
		ROS_INFO("Attempting to recognize a face\n");
                int label = face_recognition.recognize_faces();
                if (label < 0) {
                        ROS_DEBUG("No person seen, repeating again!\n");
                } else {
                        ROS_INFO("Hello person %i\n!", label);
			std_msgs::String msg;
			std::stringstream ss;
			ss << "TODONameHere" << count;
 			msg.data = ss.str();
			ROS_INFO("%s", msg.data.c_str());
			recognition_pub.publish(msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	return 0;
}
	//TODO: Publish to face_recognition
	/*
	//create a face_detection instance (sp)
	ROS_DEBUG("Making a sound client\n");
	SoundClient* client;
	ROS_DEBUG("Making a node handle\n");
	NodeHandle nh;
	ROS_DEBUG("Initizing the sound client\n");
        client = new SoundClient;
	ROS_DEBUG("Creating a test window");
        namedWindow( "test", WINDOW_AUTOSIZE );
	ROS_INFO("Createing an instance of the face recognition class\n");
        SegbotProcessor sp(nh);
	//ROS_INFO("Spinning...\n");
        //spin();
	*/
/*
	// create a face recognition instance
        ROS_INFO("Making an instance of face recognition\n");
	Face_Recognition face_recognition = Face_Recognition();
	
	while(true) {
		//ROS_INFO("Attempting to recognize a face\n");
		//detect_face(name);
		int label = face_recognition.recognize_faces();
		if (label < 0) {
			ROS_DEBUG("No person seen, repeat again!\n");
		} else {
			ROS_INFO("Hello person %i\n!", label);
			break;
		}
	}
	ROS_INFO("Destroying the window\n");
        destroyWindow("cam");
	free(client);
	return 0;
*/
