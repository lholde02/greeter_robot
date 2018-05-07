#!/usr/bin/env python

import sys
import os.path
import cv2

# This script is designed to help
# keep track of which names are associated with which labels

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print "usage: python labels.py <base_path>"
        sys.exit(1)

    BASE_PATH=sys.argv[1]
    SEPARATOR=";"

    label = 0
    file = open('/home/turtlebot/catkin_ws/src/greeter_robot/data/labels.csv', 'w')
    for dirname, dirnames, filenames in os.walk(BASE_PATH):
        for subdirname in dirnames:
            file.write("%d%s%s" % (label, SEPARATOR, subdirname))
            file.write("\n")
            label = label + 1
    file.close()

