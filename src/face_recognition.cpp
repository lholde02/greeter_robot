#include "face_detection.h"
#include "face_recognition.h"
#include <opencv2/contrib/contrib.hpp>

/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http://www.opensource.org/licenses/bsd-license>
 */
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

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

int Face_Recognition::recognize_faces() {
	ROS_INFO("In recognize faces\n");
	// predict the face in an image with the confidence
    	int predictedLabel = -10; //-10 == no face, -1 == unknown, >= 0 is a person label
    	double confidence = 0.0;
	if (fresh_face == true) { //We have gotten a new face to analyze
		model->predict(face_image, predictedLabel, confidence);
		ROS_INFO("The predicted label is %i, which corresponds to a name in the csv \n", predictedLabel);
		ROS_INFO("This was predicted with a confidence level of %lf \n", confidence);
		fresh_face = false;
	}
	
	if (predictedLabel >= 0 && confidence < 100) { //TODO: What confidence value is too low?
		predictedLabel = -1;
	}

    	//TODO: Write code to get name given label    
    	return predictedLabel;
}

void Face_Recognition::recognizer_callback(const sensor_msgs::Image::ConstPtr& msg) {
	ROS_INFO("In recognizer_callback\n");
    	cv_bridge::CvImagePtr img;
    	img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // container of type sensor_msgs/Image
    cv::Mat mat = img->image;
	face_image = mat;
	fresh_face = true;
}
void Face_Recognition::retrieve_labels() {
    std::ifstream file(label_csv.c_str(), ifstream::in);
    if (!file) {
        string error_message = "No valid labels.csv file found";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, label, name;
    while (getline(file, line)) {
      stringstream liness(line);
      getline(liness, label, csv_separator);
      getline(liness, name);
      if (!label.empty() && !name.empty()) {
        label_to_name.push_back(name); //Pushes the names in order
      }
    }
}

Face_Recognition::Face_Recognition(NodeHandle n) : it(n) {
	ROS_INFO("In face recognition constructor\n");
	ROS_INFO("setting face detection instance\n");
	detection_sub = n.subscribe("face_detection", 1000, &Face_Recognition::recognizer_callback, this);
 	// Read in the data.
    	try {
        	read_csv();
		retrieve_labels(); 
    	} catch (cv::Exception& e) {
        	cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        	// nothing more we can do
        	exit(1);
    	}
	ROS_DEBUG("Ensuring there are enough images\n");
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
