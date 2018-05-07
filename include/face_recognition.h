#ifndef FACE_RECOGNITION_H
#define FACE_RECOGNITION_H

#include "face_detection.h"
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
		const string label_csv = "/home/turtlebot/catkin_ws/src/greeter_robot/data/labels.csv";
		image_transport::ImageTransport it;
		Mat face_image;
		bool fresh_face;
		int height;
		vector<Mat> images;
		vector<int> labels;
		Ptr<FaceRecognizer> model;
		void read_csv();
		ros::Subscriber detection_sub;
		void recognizer_callback(const sensor_msgs::Image::ConstPtr& msg);
		void retrieve_labels();
		vector<string> label_to_name;
	public:
		string recognize_faces();
		Face_Recognition(NodeHandle n);
		void retrain();
};
#endif
