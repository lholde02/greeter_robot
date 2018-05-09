#include "face_recognition.h"

int main(int argc, char** argv) {
	init(argc, argv, "greeter_robot_recognition");
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);
	
	ros::NodeHandle n;
	
	system("/home/turtlebot/catkin_ws/src/greeter_robot/data/bootandtrain.sh");

	ros::Publisher recognition_pub = n.advertise<std_msgs::String>("face_recognition", 1);
	ros:Rate loop_rate(10);
        Face_Recognition face_recognition = Face_Recognition(n);

	while (ros::ok()) {
                string name = face_recognition.recognize_faces();
                if (name == "unknown") {
			system("/home/turtlebot/catkin_ws/src/greeter_robot/data/welcomemessages/unknownWelcomePart1.sh");
			string name;
			cout << "What's your name? ";
			cin >> name;

			std_msgs::String msg;
                        msg.data = name;
                        ROS_INFO("%s", msg.data.c_str());
                        recognition_pub.publish(msg);
			ros::spinOnce();
			
			std::stringstream ss;
			ss << "/home/turtlebot/catkin_ws/src/greeter_robot/data/welcomemessages/unknownWelcomePart2.sh " << name;
			string nice_to_meet_you = ss.str();
			system(nice_to_meet_you.c_str());

			sleep(10);
			face_recognition.retrain();

                } else if (name != "noone") {
                        ROS_INFO("Hello %s\n!", name.c_str());
			std::stringstream ss;
			ss << "/home/turtlebot/catkin_ws/src/greeter_robot/data/welcomemessages/welcome" <<  name << ".sh";
			string welcomepath = ss.str();
			system(welcomepath.c_str());

		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
