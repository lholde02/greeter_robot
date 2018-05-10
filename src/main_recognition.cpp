#include "face_recognition.h"

/*
* main_recognition.cpp
* Purpose: Creates a node called greeter_robot_recognition
* This node's functionality is implemented in the class Face_Recognition
* The header file for this class can be found at include/face_recognition.h
* The implementation can be found in face_recognition.cpp
*/
int main(int argc, char** argv) {
	init(argc, argv, "greeter_robot_recognition");
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	ros::NodeHandle n;
	// Train the classifiers for recognition
	system("/home/turtlebot/catkin_ws/src/greeter_robot/data/bootandtrain.sh");
  // Set up publisher, this will publish the names of NEW people it learns the face of
	ros::Publisher recognition_pub = n.advertise<std_msgs::String>("face_recognition", 1);
	ros:Rate loop_rate(10);
  Face_Recognition face_recognition = Face_Recognition(n); //Initialize face recognition class

	while (ros::ok()) {
	  string name = face_recognition.recognize_faces(); //Try to recognize a face
	  if (name == "unknown") { //Learning a new face
			system("/home/turtlebot/catkin_ws/src/greeter_robot/data/welcomemessages/unknownWelcomePart1.sh");
			string name;
			cout << "What's your name? ";
			cin >> name;
			std_msgs::String msg;
      msg.data = name;
      ROS_INFO("%s", msg.data.c_str());
      recognition_pub.publish(msg); // Publishes the new name
			ros::spinOnce();
			std::stringstream ss;
			ss << "/home/turtlebot/catkin_ws/src/greeter_robot/data/welcomemessages/unknownWelcomePart2.sh " << name;
			string nice_to_meet_you = ss.str();
			system(nice_to_meet_you.c_str()); // Play welcome message
			sleep(10);
			face_recognition.retrain();
		} else if (name != "noone") { // Recognized someone!!
      	ROS_INFO("Hello %s\n!", name.c_str());
				std::stringstream ss;
				ss << "/home/turtlebot/catkin_ws/src/greeter_robot/data/welcomemessages/welcome.sh " <<  name;
				string welcomepath = ss.str();
				system(welcomepath.c_str()); // Play welcome message
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
