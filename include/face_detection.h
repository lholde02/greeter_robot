#ifndef FACE_DETECTION_H
#define FACE_DETECTION_H

#include <sys/stat.h>
#include <dirent.h>
#include "tf/tf.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/action_client.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "opencv2/opencv.hpp"
#include "sound_play/sound_play.h"
#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "image_proc.h"
#include "std_msgs/String.h"

using namespace tf;
using namespace std;
using namespace ros;
using namespace cv_bridge;
using namespace actionlib;
using namespace sound_play;
using namespace move_base_msgs;

/*
* face_detection.h
* Class: Face_Detection
* Purpose: Header file for the facial detection node, which searches for a face
*          in a video stream and when instructed by facial recognition, will
*          take a series of photos of a NEW face for the classifier to train on
*/
class Face_Detection {
    private:
    /* Variables */
  		const String window_name = "Capture - Face Detection";
  		const string data_folder = "/home/turtlebot/catkin_ws/src/greeter_robot/data/"; // Where image folders for each person are stored
  		const String face_cascade_name = "/home/turtlebot/catkin_ws/src/greeter_robot/classifiers/haarcascade_frontalface_default.xml"; //Chosen classifier for faces, can be changed to other options
  		const String eyes_cascade_name = "/home/turtlebot/catkin_ws/src/greeter_robot/classifiers/haarcascade_eye.xml"; //Chosen classifier for eyes
      cv::CascadeClassifier face_cascade;
  		cv::CascadeClassifier eyes_cascade;
  		const int MAX_COUNT = 25; // The maximum number of face images to take at once
  		int count; // The number of training images in the folder
  		int num_training_images; // The desired number of training images, we count down from the desired number of images as we take photos
      bool processing;
      bool first = true;
      bool killed = false;
      double lastIdle = -1;
      int face_pic_num = 0; //The number of images in a folder
      string face_name;
  		vector<Mat> visible_faces;
  		ros::Subscriber recognition_sub; //Subscriber to face recognition
  		image_transport::Subscriber image_sub; //Subscriber to camera data
  		ros::Publisher detection_pub; // Publishes the detected faces
  		image_transport::ImageTransport it;
    /* Functions */
  		void retrieveFaces ();
  		IplImage *cropImage(const IplImage *img, const CvRect region);
  		Mat preProcessImage(Mat frame);
  		void detectAndDisplay( Mat frame );
  		void callback(const sensor_msgs::ImageConstPtr& msg);
  		void recognitionCallback(const std_msgs::String::ConstPtr& msg);
  	public:
  		Face_Detection(NodeHandle& nh, int argc, char** argv);
  		~Face_Detection();
  		void _idle();
  		void _kill_idle();
  		int num_training_images_to_collect();
  		vector<Mat> get_visible_faces();
};

#endif
