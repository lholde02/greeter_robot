#include "face_detection.h"

int main(int argc, char** argv) {
	init(argc, argv, "greeter_robot_runner");
	string name;
	if ( argc > 1) {
		name = string(argv[1]);
	} else {
		ROS_ERROR("Need a name for the new face\n");
		return(1);
	}
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	detect_face(name);

	return 0;
}
