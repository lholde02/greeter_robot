#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include "face_recognition.h"

using namespace cv;
using namespace cv::face;
using namespace std;

static void Face_Recognition::read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file) {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(Error::StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}

Face_Recognition::Face_Recognition(vector<Mat> imgs, vector<int> labels) {
    model = LCPHFaceRecognizer::create();
    model->train(imgs, labels);
    model->save(model_path); 
}

void Face_Recognition::update_model(vector<Mat> imgs, vector<int> labels) {
    model->update(imgs, labels);
    model->save(model_path);
}

int Face_Recognition::identify(Mat img) {
    return model->predict(img);
}
