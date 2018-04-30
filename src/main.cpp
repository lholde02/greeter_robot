#include "face_detection.h"
#include "face_recognition.h"
#include <opencv2/contrib/contrib.hpp>

int main(int argc, char** argv) {
	init(argc, argv, "greeter_robot_runner");
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	//create a face_detection instance (sp)
	//TODO: RENAME FACE_DETECTION CLASS
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

	// create a face recognition instance
        ROS_INFO("Making an instance of face recognition\n");
	Face_Recognition face_recognition = Face_Recognition(&sp);
	
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
}
