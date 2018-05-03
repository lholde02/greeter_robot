#include "face_detection.h"

bool idle = true;
vector<pair<Rect, string>> prevFaces;
cv::Mat frame;

IplImage* SegbotProcessor::cropImage(const IplImage *img, const CvRect region) {
	ROS_DEBUG("In cropImage");
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

Mat SegbotProcessor::preProcessImage(Mat frame) {
	ROS_DEBUG("In preProcessImage\n");
        cv::Mat frame_gray;
	ROS_DEBUG("grayscaleing the image\n");
        cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
	ROS_DEBUG("equalizing the image\n");
        cv::equalizeHist( frame_gray, frame_gray );
	return frame_gray;
}

void SegbotProcessor::detectAndDisplay( Mat frame ) {
        ROS_INFO("In detect and Display\n");
	frame = preProcessImage(frame);

  	// Detect Faces
      	std::vector<cv::Rect> faces;
  	face_cascade.detectMultiScale( frame/*_gray*/, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
		
	//For Each Face
  	for( size_t i = 0; i < faces.size(); i++ ) {
		//Find the center
		cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
		//Draw a rectangle around it
		cv::rectangle(frame, cvPoint(faces[i].x, faces[i].y), cvPoint(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(255, 0, 255), 1,8,0);
		Mat faceROI = frame/*_gray*/(faces[i]);

		//Find Eyes
		std::vector<Rect> eyes;
		eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );
		//For Each Face with Eyes
		visible_faces.erase(visible_faces.begin(), visible_faces.end());
		//By only working with faces with eyes we ensure fewer false positives
		for(size_t j = 0; j < eyes.size(); j++) {
			ROS_INFO("I SEE A FACE!\n");
			// Find the center
			// NOTE: Only neccisary for displaying
			cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
			// Draw a circle around the eyes
			// NOTE: (Only neccisary for displaying
       			int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
       			circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
				
			//  Crop to just an image of that face
			// TODO: Move conversion into crop
			IplImage temp_frame_gray = frame/*_gray*/;
			IplImage *frame_gray_ptr = &temp_frame_gray;
			IplImage *face_img_ptr = cropImage(frame_gray_ptr, faces[i]);
			Mat face_img = Mat(face_img_ptr, true);

			//Resize image
			Mat small_img;
			resize(face_img, small_img, Size(100, 100));
			ROS_DEBUG("Checking if the faces should be saved\n");
			// If num_training_images > 0, save the faces under the name, and decrement the counter
			if (num_training_images > 0) {
				num_training_images = num_training_images - 1;
				//Check if the person's folder exists already
				struct stat sb;
				if ( !(stat((data_folder+face_name).c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) {
					//Folder does not exist already
					//Make a folder
					ROS_INFO("Createing a folder for images %s \n", face_name.c_str());
					mkdir((data_folder + face_name).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
				}  else if (face_pic_num == 0) {
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
			ROS_DEBUG("Saving images for face_recognition\n");

			//Publisher: will publish mat images in a sensor_msg::Image format, must be converted back on recieving end
			ROS_INFO("Publishing images for recognition\n");
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
	ROS_INFO("Displaying Results");
  	imshow( "test", frame );
  	waitKey(10);    
 }

void SegbotProcessor::callback(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("In segbot processor callback\n");
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
	ROS_INFO("calling detect and display");
	detectAndDisplay(cv_ptr->image.clone());
}

void SegbotProcessor::recognitionCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("Face Detection heard the name %s", msg->data.c_str());
	face_name = msg->data.c_str();
	count = MAX_COUNT;	
}

SegbotProcessor::SegbotProcessor(NodeHandle& nh, int argc, char** argv) : it(nh) {
	ROS_DEBUG("In face detection constructor\n");
	//Subscribe to face recognition data
	ros::init(argc, argv, "recognition_listener");
	recognition_sub = nh.subscribe("face_recognition", 1, &SegbotProcessor::recognitionCallback, this);
	//TODO: DO I REALLY NEED TO spin() HERE?

	//Subscribe to camera data
	processing = true;
	image_sub = it.subscribe("/camera/rgb/image_raw", 1, &SegbotProcessor::callback, this);
	count = 0;

	face_name = "";
	face_cascade.load( face_cascade_name );
	eyes_cascade.load( eyes_cascade_name );

	detection_pub = nh.advertise<sensor_msgs::Image>("face_detection", 1);
}
	
SegbotProcessor::~SegbotProcessor() {
}

void SegbotProcessor::_idle() {
	if (lastIdle == -1 || Time::now().toSec() - lastIdle > 1) {
		lastIdle = Time::now().toSec();
	} else {
		return;
	}
}
	
void SegbotProcessor::_kill_idle() {
		lastIdle = -1;
		first = true;
		
		printf("cancelling goals\n");
		
		if (killed) {
			return;
		}
}

void SegbotProcessor::collect_training_faces(string name) {
	ROS_INFO("In collect_training_faces\n");
	face_name = name;
	count = MAX_COUNT;
}

