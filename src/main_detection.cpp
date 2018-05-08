#include "face_detection.h"

/*
* main_detection.cpp
* Purpose: Creates a node called greeter_robot_detection
* This node's functionality is implemented in the class Face_Detection
* The header file for this class can be found at include/face_detection.h
* The implementation can be found in face_detection.cpp
*/
int main(int argc, char** argv) {
	init(argc, argv, "greeter_robot_detection");
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	SoundClient* client; // Used for the robot's speech
	NodeHandle nh;
	client = new SoundClient;
	namedWindow( "test", WINDOW_AUTOSIZE );
	Face_Detection sp(nh, argc, argv);
	spin();
  destroyWindow("cam");
	free(client);
	return 0;
}
