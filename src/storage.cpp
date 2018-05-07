#include "storage.h"

//#include <vector>

//#include "image_proc.h"

vector<string> used;
unordered_map<string, Mat> db;

void learn(string name, Mat _face) {
	Mat face = _face.clone();
	resize(face, face, Size(160, 120));
	if (db[name].empty()) {
		db[name] = face;
	} else {
		addWeighted(face, .05, db[name], .95, 0.0, db[name]);
	}
}

unordered_map<string, Mat> getDB() {
	return db;
}

string findFriend(Mat _face) {
	Mat face = _face.clone();
	resize(face, face, Size(160, 120));
	for (auto item : db) {
		if (find(used.begin(), used.end(), item.first) != used.end()) {
			continue;
		}
		double compare = frameCompare(item.second, face);
		printf("compare to %s: %f\n", item.first.c_str(), compare);
		if (compare < 100) {
			used.push_back(item.first);
			return item.first;
		}
	}
	return "";
}

void reset() {
	used.clear();
}
