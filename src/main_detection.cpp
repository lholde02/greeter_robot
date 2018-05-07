#include "face_detection.h"

int main(int argc, char** argv) {
	init(argc, argv, "greeter_robot_detection");
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	SoundClient* client;
	NodeHandle nh;
	client = new SoundClient;
        namedWindow( "test", WINDOW_AUTOSIZE );
        SegbotProcessor sp(nh, argc, argv);
	spin();
        destroyWindow("cam");
	free(client);
	return 0;
}
