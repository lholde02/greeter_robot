#include <string>
#include <algorithm>
#include <stdio.h>

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

string greetings[] = {"hello", "hey there", "how are you", "nice to see you again"};
int greetings_sz = 4;

string goodbyes[] = {"good bye", "see you later", "bye", "I'll miss you"};
int goodbyes_sz = 4;

string doors[] = {"d3_414b1","d3_414b2", "d3_414a1", "d3_414a2"};
int doors_sz = 4;

String window_name = "Capture - Face detection";

//init
SoundClient* client;

//loop stuff
bool idle = true;
vector<pair<Rect, string>> prevFaces;
cv::Mat frame;

class SegbotProcessor {
   private:
      bool processing = true;
      image_transport::ImageTransport it;
      image_transport::Subscriber image_sub;
      cv::CascadeClassifier face_cascade;

      bool first = true;
      bool killed = false;
      double lastIdle = -1;

      void retrieveFaces () {
         int numberOfImages = 0;
         int goalNumberOfImages = 50;
         
         
      }

      void cropAndSave ( Mat img ) {
         //Shrink camera image to 320 pixels
         const int DETECTION_WIDTH = 320;
         // Possibly shirink the image, to run much faster
         Mat smallImg;
         float scale = (img.cols / ( (float) DETECTION_WIDTH));
         if ( img.cols > DETECTION_WIDTH) {
            //Shrink the image while keeping the same aspect ratio.
            int scaledHeight = cvRound(img.rows / scale);
            resize(img, smallImg, Size(DETECTION_WIDTH, scaledHeight));
         } else {
            //Acess the input directly since it is already small.
            smallImg = img;
         }

         //Improve contrast and brightness
         Mat equalizedImg;
         cv::equalizeHist(smallImg, equalizedImg);

         //Find Faces
  
      }

	void detectAndDisplay( Mat frame ) {
		// Preprocess
 		std::vector<cv::Rect> faces;
		cv::Mat frame_gray;
 		cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
 		//cropAndSave(frame_gray);
  		cv::equalizeHist( frame_gray, frame_gray );

  		// Detect Faces
  		face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
		
		//For Each Face
  		for( size_t i = 0; i < faces.size(); i++ ) {
			//Find the center
			cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
			//Draw a rectangle around it
			cv::rectangle(frame, cvPoint(faces[i].x, faces[i].y), cvPoint(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(255, 0, 255), 1,8,0);
			//Crop to just an image of that face?
			
			//Save croped image to faces data?
		
  		}
  		
		// Display Results
  		imshow( "test", frame );
  		waitKey(10);    
 	}

	void callback(const sensor_msgs::ImageConstPtr& msg) {
		if (!processing) {
			_kill_idle();
			return;
		}
		CvImagePtr cv_ptr;
		try {
			cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		detectAndDisplay(cv_ptr->image.clone());
	}

public:
	SegbotProcessor(NodeHandle& nh) : it(nh) {
		processing = true;
		image_sub = it.subscribe("/camera/rgb/image_raw", 1, &SegbotProcessor::callback, this);
		String face_cascade_name = "haarcascade_frontalface_alt.xml";
		face_cascade.load( face_cascade_name );
	}
	
	~SegbotProcessor() {
	}

	void _idle() {
		if (lastIdle == -1 || Time::now().toSec() - lastIdle > 1) {
			lastIdle = Time::now().toSec();
		} else {
			return;
		}
	}
	
	void _kill_idle() {
		lastIdle = -1;
		first = true;
		
		printf("cancelling goals\n");
		
		if (killed) {
			return;
		}
	}
};

int main(int argc, char** argv) {
	init(argc, argv, "friendly_faces_runner");

	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	NodeHandle nh;
	client = new SoundClient;
	namedWindow( "test", WINDOW_AUTOSIZE );

	SegbotProcessor sp(nh);
	spin();
	destroyWindow("cam");

	free(client);

	return 0;
}
