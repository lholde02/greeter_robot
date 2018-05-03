#include "opencv2/core/core.hpp"
#include <opencv2/contrib/contrib.hpp>
//#include "opencv2/face/face.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

class Face_Recognition {
	private:
		const double IMAGE_WIDTH = 100;
 		const double IMAGE_HEIGHT = 100;
		const string fn_csv = "/home/turtlebot/catkin_ws/src/greeter_robot/data/faces.csv";
		const char csv_separator = ';';
		image_transport::ImageTransport it;
		Mat face_image;
		bool fresh_face;
		int height;
		vector<Mat> images;
		vector<int> labels;
		//vector<Mat> testSample; //TODO: REMOVE
		//vector<int> testLabel; //TODO: REMOVE
		Ptr<FaceRecognizer> model;
		void read_csv();
		ros::Subscriber detection_sub;
		void recognizer_callback(const sensor_msgs::Image::ConstPtr& msg);
	public:
		int recognize_faces();
		Face_Recognition(NodeHandle n);
};
