#!/bin/bash
python create_csv.py "/home/turtlebot/catkin_ws/src/greeter_robot/data/" > faces.csv
python labels.py "/home/turtlebot/catkin_ws/src/greeter_robot/data/"
