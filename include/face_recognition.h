#include "opencv2/core/core.hpp"
#include <opencv2/contrib/contrib.hpp>
//#include "opencv2/face/face.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

class Face_Recognition {
	private:
	   Mat norm_0_255(InputArray _src);
           void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator);
	public:
	   int recognize_faces();
};
