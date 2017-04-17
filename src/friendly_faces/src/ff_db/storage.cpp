#include "ff_db/storage.h"

#include <vector>
#include <unordered_map>

#include "ff_proc/image_proc.h"

vector<string> used;
unordered_map<string, Mat> db;

void learn(string name, Mat face) {
	if (db[name].empty()) {
		db[name] = face;
	} else {
		addWeighted(face, .01, db[name], .99, 0.0, db[name]);
	}
}

Mat getDB(string name) {
	return db[name];
}

string findFriend(Mat frame) {
	double lowestCompare = 1000000000000000000;
	string lowestName = "";
	for (auto item : db) {
		if (find(used.begin(), used.end(), item.first) != used.end()) {
			continue;
		}
		double compare = frameCompare(db[item.first], frame);
		if (compare < lowestCompare) {
			lowestCompare = compare;
			lowestName = item.first;
		}
		if (compare < 100) {
			//return name;
		}
	}
	used.push_back(lowestName);
	return lowestName;
}

void reset() {
	used.clear();
}
