#pragma once

#include <string>
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

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

using namespace tf;
using namespace std;
using namespace ros;
using namespace cv_bridge;
using namespace actionlib;
using namespace sound_play;
using namespace move_base_msgs;

class SegbotProcessor {
   	private:
      		bool processing = true;
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
		IplImage* cropImage(const IplImage *img, const CvRect region);
		void detectAndDisplay( Mat frame );
		void callback(const sensor_msgs::ImageConstPtr& msg);
	public:
		SegbotProcessor(NodeHandle& nh, string name);
		~SegbotProcessor();
};

void detect_face(string name);
