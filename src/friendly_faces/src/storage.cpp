#include "storage.h"

#include <vector>
#include <unordered_map>

#include "image_proc.h"

vector<string> used;
unordered_map<string, Mat> db;

void learn(string name, Mat face) {
	if (db[name].empty()) {
		db[name] = face;
	} else {
		addWeighted(face, .10, db[name], .90, 0.0, db[name]);
	}
}

Mat getDB(string name) {
	return db[name];
}

string findFriend(Mat frame) {
	for (auto item : db) {
		if (find(used.begin(), used.end(), item.first) != used.end()) {
			continue;
		}
		double compare = frameCompare(db[item.first], frame);
		if (compare < 60) {
			return item.first;
		}
	}
	return "";
}

void reset() {
	used.clear();
}
