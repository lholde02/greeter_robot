MEET & GREET
----------------------------------
A robot to greet people when they enter Halligan,
to make people feel more welcome.


Getting Started
----------------------------------

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
roslaunch greeter_robot greeter_robot_runner


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
Marcus-Zhu for their friendly faces github repo
(https://github.com/Noviv/friendly_faces) which we used as a base 
to get started with facial detection using opencv.
A special thank you to Jivko Sinapov for his help with facial 
detection and facial recognition, his understanding, and 
his support. 
