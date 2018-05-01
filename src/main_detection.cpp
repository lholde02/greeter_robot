#include "face_detection.h"
#include "face_recognition.h"
#include <opencv2/contrib/contrib.hpp>

int main(int argc, char** argv) {
	init(argc, argv, "greeter_robot_detection");
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

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
	spin();

	ROS_INFO("Destroying the window\n");
        destroyWindow("cam");
	free(client);
	return 0;
}
