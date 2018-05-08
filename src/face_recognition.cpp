#include "face_recognition.h"

/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 */

using namespace cv;
using namespace std;

/*
* read_csv uses the faces csv file to retrieve all of the images
* it will need to train the classifier for facial recognition
*/
void Face_Recognition::read_csv() {
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

/*
* recognize_faces when called will take the received face image and run
* the classifier on it, if it finds a match with certainty above a reasonable
* threshold it will report the name associated with the face it believes
* it has seen
*/
string Face_Recognition::recognize_faces() {
		// Predict the face in an image with the confidence
  	int predictedLabel = -10; //-10 == no face, -1 == unknown, >= 0 is a person label
  	double confidence = 0.0;
		if (fresh_face == true) { //We have gotten a new face to analyze
			model->predict(face_image, predictedLabel, confidence);
			ROS_INFO("The predicted label is %i, which corresponds to the name %s in the csv \n", predictedLabel, label_to_name[predictedLabel].c_str());
			ROS_INFO("The following was predicted with a confidence level of %lf :\n", confidence);
			fresh_face = false;
		}

		if (predictedLabel >= 0 && confidence > 3500) {
			return "unknown"; // Confidence values above 3500 indicate that we do not recognize this face
		} else if (predictedLabel >= 0) {
			return label_to_name[predictedLabel].c_str();
		}
		return "noone";
}

void Face_Recognition::recognizer_callback(const sensor_msgs::Image::ConstPtr& msg) {
		cv_bridge::CvImagePtr img;
		img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // container of type sensor_msgs/Image
	  cv::Mat mat = img->image;
		face_image = mat;
		fresh_face = true;
}

/*
* retrieve_labels() uses the label csv file, a file that lays out which
* index is related to which person, in order to create an array that maps
* from face index to the name associated with that face
*/
void Face_Recognition::retrieve_labels() {
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
        label_to_name.push_back(name); //Pushes the names in order
      }
    }
}

/*
* Constructor for the Face_Recognition class
* Handles subscribing to the face_detection node, retrieving data,
* and training the classifier for identifying faces upon initialization
*/
Face_Recognition::Face_Recognition(NodeHandle n) : it(n) {
		detection_sub = n.subscribe("face_detection", 1, &Face_Recognition::recognizer_callback, this);
 		// Read in the data.
    try {
			read_csv();
			retrieve_labels();
    } catch (cv::Exception& e) {
    	cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
    	// nothing more we can do
    	exit(1);
    }

 		// Quit if there are not enough images.
   	if(images.size() <= 1) {
        	string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
        	CV_Error(CV_StsError, error_message);
    }
  	// Get the height from the first image
  	height = images[0].rows;
		fresh_face = false;
  	model = createEigenFaceRecognizer(); //Create a Face Recognizer
  	model->train(images, labels); //Train the Face Recognizer
}

/*
* Trains the classifier with all of the faces with folders
* in the data directory and listed in the faces.csv file
*/
void Face_Recognition::retrain() {
	system("/home/turtlebot/catkin_ws/src/greeter_robot/data/retrain.sh");
	read_csv();
	retrieve_labels();
	model->train(images, labels);
}
