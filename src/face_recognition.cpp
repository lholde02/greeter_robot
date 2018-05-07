#include "face_recognition.h"

/*
 * Some code taken from  <http://www.opensource.org/licenses/bsd-license>
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 */

using namespace cv;
using namespace std;

void Face_Recognition::read_csv() {
	ROS_DEBUG("In read csv\n");
  	std::ifstream file(fn_csv.c_str(), ifstream::in);
    	if (!file) {
    	    string error_message = "No valid input file was given, please check the given filename.";
    	    CV_Error(CV_StsBadArg, error_message);
    	}
    	string line, path, classlabel;
    	while (getline(file, line)) {
 		stringstream liness(line);
 		getline(liness, path, csv_separator);
 		getline(liness, classlabel);
 		if(!path.empty() && !classlabel.empty()) {
			Mat img = imread(path, IMREAD_GRAYSCALE);
			if (img.empty()) {
				cout << path << "could not be read!" << endl;
				exit(-1);
		    	}	
	    		images.push_back(img);		
            		labels.push_back(atoi(classlabel.c_str()));
       		}	
	}
}

string Face_Recognition::recognize_faces() {
//	ROS_INFO("In recognize faces\n");
	// predict the face in an image with the confidence
    	int predictedLabel = -10; //-10 == no face, -1 == unknown, >= 0 is a person label
    	double confidence = 0.0;
	if (fresh_face == true) { //We have gotten a new face to analyze
		model->predict(face_image, predictedLabel, confidence);
		ROS_INFO("The predicted label is %i, which corresponds to the name %s in the csv \n", predictedLabel, label_to_name[predictedLabel].c_str());
		ROS_INFO("The following was predicted with a confidence level of %lf :\n", confidence);
		fresh_face = false;
	}
	
	if (predictedLabel >= 0 && confidence > 3500) { //TODO: What confidence value is too low?
		return "unknown";
	} else if (predictedLabel >= 0) {
    		//TODO: Write code to get name given label    
		ROS_INFO("Label %i translates to %s\n", predictedLabel, label_to_name[predictedLabel].c_str());
		return label_to_name[predictedLabel].c_str();
	}
	return "noone";
}

void Face_Recognition::recognizer_callback(const sensor_msgs::Image::ConstPtr& msg) {
//	ROS_INFO("In recognizer_callback\n");
    	cv_bridge::CvImagePtr img;
    	img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // container of type sensor_msgs/Image
    cv::Mat mat = img->image;
	face_image = mat;
	fresh_face = true;
}
void Face_Recognition::retrieve_labels() {
	ROS_INFO("Lable file: %s\n", label_csv.c_str());
    std::ifstream file(label_csv.c_str(), ifstream::in);
    if (!file) {
        string error_message = "No valid labels.csv file found";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, label, name;
    label_to_name.clear();
    while (getline(file, line)) {
      stringstream liness(line);
      getline(liness, label, csv_separator);
      getline(liness, name);
      if (!label.empty() && !name.empty()) {
	ROS_INFO("Pushing back name %s", name.c_str());
        label_to_name.push_back(name); //Pushes the names in order
      }
    }
}

Face_Recognition::Face_Recognition(NodeHandle n) : it(n) {
//	ROS_INFO("In face recognition constructor\n");
//	ROS_INFO("setting face detection instance\n");
	detection_sub = n.subscribe("face_detection", 1, &Face_Recognition::recognizer_callback, this);
 	// Read in the data.
    	try {
		ROS_INFO("Reading both csv files\n");
        	read_csv();
		retrieve_labels(); 
    	} catch (cv::Exception& e) {
        	cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        	// nothing more we can do
        	exit(1);
    	}
	ROS_INFO("Ensuring there are enough images\n");
 	// Quit if there are not enough images.
   	if(images.size() <= 1) {
        	string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
        	CV_Error(CV_StsError, error_message);
    	}
    	// Get the height from the first image
    	height = images[0].rows;

	fresh_face = false;

    	ROS_INFO("creating eigenfaces recognizer\n");
    	//Create a Faceial Recognizer
    	model = createEigenFaceRecognizer();
    	ROS_INFO("training the recognizer\n");
    	//Train the Faceial Recognizer
    	model->train(images, labels);
    	ROS_INFO("finished training the recognizer\n");
}

void Face_Recognition::retrain() {
	ROS_INFO("Attempting to retrain\n");
	system("/home/turtlebot/catkin_ws/src/greeter_robot/data/retrain.sh");
	read_csv();
	retrieve_labels();
	model->train(images, labels);
}

