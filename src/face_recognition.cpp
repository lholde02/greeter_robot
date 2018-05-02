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
	ROS_DEBUG("In recognize faces\n");
	// predict the face in an image with the confidence
    	int predictedLabel = -10; //-10 == no face, -1 == unknown, >= 0 is a person label
    	double confidence = 0.0;
    	ROS_DEBUG("Getting visible faces\n");
	fresh_face = false;
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

Face_Recognition::Face_Recognition() {
	ROS_DEBUG("In face recognition constructor\n");
	ROS_DEBUG("setting face detection instance\n");
 	//TODO: SUBSCRIBE TO FACIAL DETECTION!!
 	// Read in the data.
    	try {
        	read_csv();
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

    	//TODO: remove the following when facial recognition gets data from facial detection
/*
	for (int i = 0; i < 3; i++) {
		testSample.push_back(images[images.size() - 1]);
		testLabel.push_back(labels[labels.size() - 1]);
		images.pop_back();
		labels.pop_back();
	}
*/
  	// The following lines create an Eigenfaces model for
    	// face recognition and train it with the images and
    	// labels read from the given CSV file.
    	// This here is a full PCA, if you just want to keep
    	// 10 principal components (read Eigenfaces), then call
    	// the factory method like this:
    	//
    	//      cv::createEigenFaceRecognizer(10);
    	//
    	// If you want to create a FaceRecognizer with a
    	// confidence threshold (e.g. 123.0), call it with:
    	//
    	//      cv::createEigenFaceRecognizer(10, 123.0);
    	//
    	// If you want to use _all_ Eigenfaces and have a threshold,
    	// then call the method like this:
    	//
    	//      cv::createEigenFaceRecognizer(0, 123.0);
    	//
    	ROS_INFO("creating eigenfaces recognizer\n");
    	//Create a Faceial Recognizer
    	model = createEigenFaceRecognizer();
    	ROS_INFO("training the recognizer\n");
    	//Train the Faceial Recognizer
    	model->train(images, labels);
    	ROS_INFO("finished training the recognizer\n");
}
