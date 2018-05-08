MEET & GREET
----------------------------------
This work explores creating a robot that will recognize and greet people it has met before as well as learn the faces of new people. The purpose of this project was to create a friendly face in a space, specifically Halligan Hall at Tufts University, in order to provide a small comfort to visitors. Our group used a Turtlebot running ROS, an open-source meta-operating system used widely in robotics, as a robot sociable partner. We applied facial detection and recognition to our robot so that it could keep track of the people it 'knows'. Throughout this project, we also referred to work in human-robot interaction to inform design decisions about our robot in order to make it more appealing in the role of greeter.


Getting Started
----------------------------------

Dependencies
-----------------

Installing
-----------------
cd ~/catkin_ws/src

git clone https://github.com/lholde02/greeter-robot

cd ~/catkin_ws

catkin_make

rospack_profile

source devel/setup.bash

Running
-----------------
Run the following command:

roslaunch greeter_robot greeter_robot_runner

This launches roscore, the tbot2 launch file created for the Tufts Turtlebots, and our two ROS nodes: facial detection and recognition.


Authors
----------------------------------
Anne Oursler

Leah Holden

Lauren Dierker


Licence
----------------------------------
Copyright (c) 2018. Anne Oursler, Leah Holden, and Lauren Dierker
Released to public domain under terms of the BSD Simplified license.
<http://www.opensource.org/licenses/bsd-license>


Acknolegements
----------------------------------
Thank you to Jivko's former students: Noviv, nj1407, and
Marcus-Zhu for their friendly faces code
(https://github.com/Noviv/friendly_faces) which we used as a base
to get started with facial detection using OpenCV.

A special thank you to Jivko Sinapov for his help with facial
detection and facial recognition, his understanding, and
his support.
