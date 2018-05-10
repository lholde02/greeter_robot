#include "face_detection.h"

/* Globals needed for the code from the friendly faces repository */
bool idle = true;
vector<pair<Rect, string>> prevFaces;
cv::Mat frame;

/*
* face_detection.cpp
* Class: Face_Detection
* Purpose: Implementation for the facial detection node
*/

/*
* cropImage takes a pointer to an IplImage, an older form of Mat, and crops
* the image to only include the face featured in an image. This is important
* for learning this face later.
*/
IplImage* Face_Detection::cropImage(const IplImage *img, const CvRect region) {
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
	  cvReleaseImage( &imageTmp ); // Free up memory
		return imageRGB;
}

/*
* preProcessImage takes a Mat image and converts it to gray-scale, which
* is required before we can train our classifier on this image
*/
Mat Face_Detection::preProcessImage(Mat frame) {
	  cv::Mat frame_gray;
	  cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
	  cv::equalizeHist( frame_gray, frame_gray );
		return frame_gray;
}

/*
* Handles a large portion of functionality for facial detection, makes sure
* that each face detected also has at least one eye, does preprocessing,
* calls other functions to do more preprocessing, and displays the results
*/
void Face_Detection::detectAndDisplay( Mat frame ) {
		frame = preProcessImage(frame);

  	// Detect Faces
  	std::vector<cv::Rect> faces;
  	face_cascade.detectMultiScale( frame, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

		//For Each Face
  	for( size_t i = 0; i < faces.size(); i++ ) {
			//Find the center
			cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
			//Draw a rectangle around it
			cv::rectangle(frame, cvPoint(faces[i].x, faces[i].y), cvPoint(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(255, 0, 255), 1,8,0);
			Mat faceROI = frame(faces[i]);
			//Find Eyes
			std::vector<Rect> eyes;
			eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

			//For Each Face with Eyes
			//By only working with faces with eyes we ensure fewer false positives
			for(size_t j = 0; j < eyes.size(); j++) {
				//  Crop to just an image of that face
				IplImage temp_frame_gray = frame;
				IplImage *frame_gray_ptr = &temp_frame_gray;
				IplImage *face_img_ptr = cropImage(frame_gray_ptr, faces[i]);
				Mat face_img = Mat(face_img_ptr, true);

				//Resize image to be 100x100 pixels
				Mat small_img;
				resize(face_img, small_img, Size(100, 100));
				// If num_training_images > 0, save the faces under the name, and decrement the counter
				ROS_INFO("num_training_images to collect %i\n", num_training_images);
				if (num_training_images > 0) {
					num_training_images = num_training_images - 1;
					//Check if the person's folder exists already
					struct stat sb;
					if ( !(stat((data_folder+face_name).c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) {
						//If the folder does not exist already, make a folder
						mkdir((data_folder + face_name).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
					}
						
					//Folder exists, so it might have files in it we dont want to overwite, so we must count the files in it
					face_pic_num = 0;
					DIR *dp;
					struct dirent *ep;
					dp = opendir( (data_folder + face_name).c_str());
					while (ep = readdir (dp))
						face_pic_num++;
					(void) closedir(dp);
					//Save croped image to faces data in their folder
					//ensure sequential naming of photos,
					//even if there are already photos in the folder
					String image_name = data_folder + face_name + "/" + std::to_string(face_pic_num) + ".pgm";
					imwrite(image_name, small_img);//save image
					ROS_INFO("wrote %s", image_name.c_str());				
				}

				//Publisher: will publish Mat images in a sensor_msg::Image format, must be converted back on recieving end
				sensor_msgs::Image msg;
				cv_bridge::CvImage cvi_mat;
				cvi_mat.encoding = sensor_msgs::image_encodings::MONO8;
				cvi_mat.image = small_img;
				cvi_mat.toImageMsg(msg);
				detection_pub.publish(msg);
				ros::spinOnce();
				count++;
  			}
		}
		// Display Results
  		imshow( "test", frame );
 		waitKey(10);
 }

/*
* callback is the function that gets called every time the facial_detection
* node receives an image on the topic usb_cam
* This function initializes all of the preprocessing that must be done
* before the image can be published to the topic face_detection
*/
void Face_Detection::callback(const sensor_msgs::ImageConstPtr& msg) {
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

/*
* Callback function that gets called when this class receives a message
* from the face recognition node indicating that we need to take a certain
* number of photos for the classifier to be trained on for a NEW person
*/
void Face_Detection::recognitionCallback(const std_msgs::String::ConstPtr& msg) {
	face_name = msg->data.c_str();
	num_training_images = MAX_COUNT;
}

/*
* Constructor for the Face_Detection class
* Handles subscribing and publishing to the necessary topics
*/
Face_Detection::Face_Detection(NodeHandle& nh, int argc, char** argv) : it(nh) {
	face_name = "";
	num_training_images = 0;

	//Subscribe to face recognition data
	ros::init(argc, argv, "recognition_listener");
	recognition_sub = nh.subscribe("face_recognition", 1, &Face_Detection::recognitionCallback, this);

	//Subscribe to camera data
	processing = true;
	image_sub = it.subscribe("/camera/rgb/image_raw", 1, &Face_Detection::callback, this);
	count = 0;

	face_cascade.load( face_cascade_name );
	eyes_cascade.load( eyes_cascade_name );

	detection_pub = nh.advertise<sensor_msgs::Image>("face_detection", 1);
}

/*
* Deconstructor - currently unnecessary
*/
Face_Detection::~Face_Detection() {
}

/*
* Extracted from friendly faces' code
*/
void Face_Detection::_idle() {
	if (lastIdle == -1 || Time::now().toSec() - lastIdle > 1) {
		lastIdle = Time::now().toSec();
	} else {
		return;
	}
}

/*
* Extracted from friendly faces' code
*/
void Face_Detection::_kill_idle() {
		lastIdle = -1;
		first = true;

		printf("cancelling goals\n");

		if (killed) {
			return;
		}
}

// Returns the number of training images we need to collect
int Face_Detection::num_training_images_to_collect() {
	return num_training_images;
}
