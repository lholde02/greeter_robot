#!/bin/bash
echo In Retrain.py
python /home/turtlebot/catkin_ws/src/greeter_robot/data/create_csv.py "/home/turtlebot/catkin_ws/src/greeter_robot/data/" > /home/turtlebot/catkin_ws/src/greeter_robot/data/faces.csv
python /home/turtlebot/catkin_ws/src/greeter_robot/data/labels.py "/home/turtlebot/catkin_ws/src/greeter_robot/data/" > /home/turtlebot/catkin_ws/src/greeter_robot/data/labels.csv
