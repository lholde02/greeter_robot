#!/bin/bash
echo In boot and train
rosrun sound_play say.py "Welcome to Halligan! Please give me a second to boot and train my classifiers."
/home/turtlebot/catkin_ws/src/greeter_robot/data/retrain.sh

