#ifndef FACE_RECOGNITION_H
#define FACE_RECOGNITION_H

#include "face_detection.h"
#include <fstream>
#include <sstream>
#include <time.h>

using namespace cv;
using namespace std;

/*
* face_recognition.h
* Class: Face_Recognition
* Purpose: Header file for the face recognition node, tries to identify
*					 whose face is in the image it receives OR tells Face_Detection
*					 to learn a new face
*/

class Face_Recognition {
	private:
		const double IMAGE_WIDTH = 100;  // In pixels
 		const double IMAGE_HEIGHT = 100; // In pixels
		const string fn_csv = "/home/turtlebot/catkin_ws/src/greeter_robot/data/faces.csv";
		const char csv_separator = ';';
		const string label_csv = "/home/turtlebot/catkin_ws/src/greeter_robot/data/labels.csv";
		image_transport::ImageTransport it; // Used for publish/subscribe ops
		Mat face_image;
		vector<string> label_to_name;
		vector<ros::Time> label_last_time; // Mapping parallel to label_to_name
																			 // holding the time stamp of the last
																			 // time we saw that face. When we learn
																			 // a new person, all timestamps reset
		const ros::Duration welcome_wait_time = ros::Duration(30 * 60);
		bool fresh_face; // Bool that indicates if we received a new face from face
										 // detection or are looking at an old face
		int height;
		vector<Mat> images;
		vector<int> labels;
		Ptr<FaceRecognizer> model;     // Contains the loaded classifier model
		ros::Subscriber detection_sub; // Subscribes to the face detection's msgs
		void read_csv();         // Retrieves images for training the classifier
		void retrieve_labels(); // Creates a mapping of person index to their name
		void recognizer_callback(const sensor_msgs::Image::ConstPtr& msg);
	public:
		string recognize_faces(); // Returns a name or 'unknown', depending on
														  // if the image contains a face it knows
		Face_Recognition(NodeHandle n);
		void retrain();
};
#endif
