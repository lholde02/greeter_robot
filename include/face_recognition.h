#include "opencv2/core/core.hpp"
#include <opencv2/contrib/contrib.hpp>
//#include "opencv2/face/face.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
//using namespace cv::face;
using namespace std;

class Face_Recognition {
	private:
	   cv::Ptr<cv::FaceRecognizer> model;//Ptr<LBPHFaceRecognizer> model;
	   string model_path = "/home/turtlebot/catkin_ws/src/greeter_robot/data/recognizer_model.xml";
	   void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator);
	public:
	   Face_Recognition(vector<Mat> imgs, vector<int> labels);
	   void update_model(vector<Mat> imgs, vector<int> labels);
	   int identify(Mat img);
};
