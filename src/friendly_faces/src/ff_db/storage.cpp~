#include "ff_db/storage.h"

#include <vector>
#include <unordered_map>

#include "ff_proc/image_proc.h"

vector<string> presets;
vector<string> used;
unordered_map<string, Mat> db;

void loadPresets() {
	presets.push_back("ailyn");
	presets.push_back("anjuli");
	presets.push_back("matthew");

	for (int i = 0; i < presets.size(); i++) {
		db[presets.at(i)] = imread(("res/" + presets.at(i) + ".png").c_str(), CV_LOAD_IMAGE_COLOR);
		resize(db[presets.at(i)], db[presets.at(i)], Size(250, 250));
	}
}

string findFriend(Mat frame) {
	double lowestCompare = 1000000000000000000;
	string lowestName = "";
	for (string name : presets) {
		if (find(used.begin(), used.end(), name) != used.end()) {
			continue;
		}
		double compare = frameCompare(db[name], frame);
		printf("compare %s: %f\n", name.c_str(), compare);
		if (compare < lowestCompare) {
			lowestCompare = compare;
			lowestName = name;
		}
		if (compare < 100) {
			//return name;
		}
	}
	printf("\n");
	used.push_back(lowestName);
	return lowestName;
}

void reset() {
	used.clear();
}
