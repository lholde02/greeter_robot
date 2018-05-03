#ifndef FACE_DETECTION_H
#define FACE_DETECTION_H
//#pragma once

#include <string>
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <sstream>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/action_client.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"

#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sound_play/sound_play.h"

#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "text.h"
#include "storage.h"
#include "image_proc.h"
#include "fps.h"

#include "std_msgs/String.h"

using namespace tf;
using namespace std;
using namespace ros;
using namespace cv_bridge;
using namespace actionlib;
using namespace sound_play;
using namespace move_base_msgs;

class SegbotProcessor {
   	private:
		const String window_name = "Capture - Face Detection";
		const string data_folder = "/home/turtlebot/catkin_ws/src/greeter_robot/data/";
		const String face_cascade_name = "/home/turtlebot/catkin_ws/src/greeter_robot/classifiers/haarcascade_frontalface_default.xml";
		const String eyes_cascade_name = "/home/turtlebot/catkin_ws/src/greeter_robot/classifiers/haarcascade_eye.xml";
		const int MAX_COUNT = 25;
		int count;

		int num_training_images;
		vector<Mat> visible_faces;
		
		//Subscriber to face recognition
		ros::Subscriber recognition_sub;

		//Subscriber to camera data
      		bool processing;
      		image_transport::ImageTransport it;
      		image_transport::Subscriber image_sub;
      		cv::CascadeClassifier face_cascade;
      		cv::CascadeClassifier eyes_cascade;

      		bool first = true;
      		bool killed = false;
      		double lastIdle = -1;
  		int face_pic_num = 0;
      		string face_name;

      		void retrieveFaces ();
		IplImage *cropImage(const IplImage *img, const CvRect region);
		Mat preProcessImage(Mat frame);
		void detectAndDisplay( Mat frame );
		void callback(const sensor_msgs::ImageConstPtr& msg);
		void recognitionCallback(const std_msgs::String::ConstPtr& msg);
		ros::Publisher detection_pub;
	public:
		SegbotProcessor(NodeHandle& nh, int argc, char** argv);
		~SegbotProcessor();
		void _idle();
		void _kill_idle();
		void collect_training_faces(string name);
		vector<Mat> get_visible_faces();

};

void detect_face(string name);
#endif
