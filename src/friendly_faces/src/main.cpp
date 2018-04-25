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

string greetings[] = {"hello", "hey there", "how are you", "nice to see you again"};
int greetings_sz = 4;

string goodbyes[] = {"good bye", "see you later", "bye", "I'll miss you"};
int goodbyes_sz = 4;

string doors[] = {"d3_414b1","d3_414b2", "d3_414a1", "d3_414a2"};
int doors_sz = 4;

String window_name = "Capture - Face detection";

string data_folder = "/home/turtlebot/catkin_ws/src/greeter_robot/src/friendly_faces/data/";
String face_cascade_name = "/home/turtlebot/catkin_ws/src/greeter_robot/src/friendly_faces/classifiers/haarcascade_frontalface_default.xml";
String eyes_cascade_name = "/home/turtlebot/catkin_ws/src/greeter_robot/src/friendly_faces/classifiers/haarcascade_eye.xml";

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
      cv::CascadeClassifier eyes_cascade;

      bool first = true;
      bool killed = false;
      double lastIdle = -1;
      int face_pic_num = 0;
      string face_name;

      void retrieveFaces () {
         int numberOfImages = 0;
         int goalNumberOfImages = 50;
         
         
      }
	// Returns a new image that is a cropped version of the original image.
	IplImage* cropImage(const IplImage *img, const CvRect region)
	{
	    IplImage *imageTmp;
	    IplImage *imageRGB;
	    CvSize size;
	    size.height = img->height;
	    size.width = img->width;

	    if (img->depth != IPL_DEPTH_8U) {
		ROS_INFO("ERROR in cropImage: Unknown image depth of %d given in cropImage() instead of 8 bits per pixel.", img->depth);
		exit(1);
	    }

	    // First create a new (color or greyscale) IPL Image and copy contents of img into it.
	    imageTmp = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	    cvCopy(img, imageTmp, NULL);

	    // Create a new image of the detected region
	    // Set region of interest to that surrounding the face
	    cvSetImageROI(imageTmp, region);
	    // Copy region of interest (i.e. face) into a new iplImage (imageRGB) and return it
	    size.width = region.width;
	    size.height = region.height;
	    imageRGB = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	    cvCopy(imageTmp, imageRGB, NULL);	// Copy just the region.

            cvReleaseImage( &imageTmp );
	    return imageRGB;
        }

	void detectAndDisplay( Mat frame ) {
		// Preprocess
 		std::vector<cv::Rect> faces;
		cv::Mat frame_gray;
 		cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
  		cv::equalizeHist( frame_gray, frame_gray );

  		// Detect Faces
  		face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
		
		//For Each Face
  		for( size_t i = 0; i < faces.size(); i++ ) {
			//Find the center
			cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
			//Draw a rectangle around it
			cv::rectangle(frame, cvPoint(faces[i].x, faces[i].y), cvPoint(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(255, 0, 255), 1,8,0);
			//FIND EYES, ONLY USE FACES WITH EYES...
			Mat faceROI = frame_gray(faces[i]);
			std::vector<Rect> eyes;
			eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );
			for(size_t j = 0; j < eyes.size(); j++) {
				cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
       				int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
       				circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
			//Crop to just an image of that face
			IplImage temp_frame_gray = frame_gray;
			IplImage *frame_gray_ptr = &temp_frame_gray;
			IplImage *face_img_ptr = cropImage(frame_gray_ptr, faces[i]);
			Mat face_img = Mat(face_img_ptr, true);

			//Resize image
         		const int DETECTION_WIDTH = 320;
        		Mat small_img;
         		float scale = (face_img.cols / ( (float) DETECTION_WIDTH));
         		if (face_img.cols > DETECTION_WIDTH) {
            			//Shrink the image while keeping the same aspect ratio.
            			int scaledHeight = cvRound(face_img.rows / scale);
           			resize(face_img, small_img, Size(DETECTION_WIDTH, scaledHeight));
         		} else {
            			//Acess the input directly since it is already small.
            			small_img = face_img;
         		}
	
			//Check if the person's folder exists already
			struct stat sb;
			if ( !(stat((data_folder+face_name).c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) {
				//Folder does not exist already
				//Make a folder
				ROS_INFO("Createing a folder for images %s \n", face_name.c_str());
				mkdir((data_folder + face_name).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			} else if (face_pic_num == 0) {
				//Folder exists, so it might have files in it we dont want to overwite, so we must count the files in it
				DIR *dp;
				struct dirent *ep;
				dp = opendir( (data_folder + face_name).c_str());
				while (ep = readdir (dp))
					face_pic_num++;
				(void) closedir(dp);
			}
			//Save croped image to faces data in their folder
			//ensure sequential naming of photos,
			//even if there are already photos in the folder
			ROS_INFO("Saveing face %i \n", face_pic_num);
			String image_name = data_folder + face_name + "/" + std::to_string(face_pic_num) + ".pgm";
			face_pic_num = face_pic_num + 1;
			imwrite(image_name, small_img);//save image
			}
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
	SegbotProcessor(NodeHandle& nh, string name) : it(nh) {
		processing = true;
		image_sub = it.subscribe("/camera/rgb/image_raw", 1, &SegbotProcessor::callback, this);

		face_name = name;
		face_cascade.load( face_cascade_name );
		eyes_cascade.load( eyes_cascade_name );
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
	string name;
	if ( argc > 1) {
		name = string(argv[1]);
	} else {
		ROS_ERROR("Need a name for the face\n");
		return(1);
	}
	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	NodeHandle nh;
	client = new SoundClient;
	namedWindow( "test", WINDOW_AUTOSIZE );

	SegbotProcessor sp(nh, name);
	spin();
	destroyWindow("cam");

	free(client);

	return 0;
}
