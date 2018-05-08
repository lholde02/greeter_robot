MEET & GREET
----------------------------------
This work explores creating a robot that will recognize and greet people it has met before as well as learn the faces of new people. The purpose of this project was to create a friendly face in a space, specifically Halligan Hall at Tufts University, in order to provide a small comfort to visitors. Our group used a Turtlebot running ROS, an open-source meta-operating system used widely in robotics, as a robot sociable partner. We applied facial detection and recognition to our robot so that it could keep track of the people it 'knows'. Throughout this project, we also referred to work in human-robot interaction to inform design decisions about our robot in order to make it more appealing in the role of greeter.


Getting Started
----------------------------------

Dependencies
-----------------
- Built for a Turtlebot running Ubuntu 14.04 and ROS Indigo
- OpenCV must be installed, typically already is installed. This package is used to run facial detection and recognition
- Need to have tufts_service_robots repo installed, https://github.com/jsinapov/tufts_service_robots.git
- Other packages that are required:
  * usb_cam for accessing camera images
    - Wiki: http://wiki.ros.org/usb_cam
    - Git repo: https://github.com/ros-drivers/usb_cam.git
  * sound_play for robot speech
    - Wiki: http://wiki.ros.org/sound_play
    - Git repo: this repo audio_common contains multiple folders including sound_play, which is the folder we need https://github.com/ros-drivers/audio_common.git
  * tf is a transform library
    - Wiki: http://wiki.ros.org/tf
    - Git repo: this repo geometry contains multiple folders including tf https://github.com/ros/geometry.git


Installing
-----------------
**cd ~/catkin_ws/src**

**git clone https://github.com/lholde02/greeter-robot**

**cd ~/catkin_ws**

**catkin_make**

**rospack_profile**

**source devel/setup.bash**

Running
-----------------
Run the following command:

**roslaunch greeter_robot greeter_robot_runner**

This launches roscore, the tbot2 launch file created for the Tufts Turtlebots, and our two ROS nodes: facial detection and recognition. The Turtlebot laptop and base must be on before running this command.

Navigating the File Structure
----------------------------------
- **classifiers** - A folder containing different types of classifiers that can be used for facial recognition, stored as .xml files. These files come from the OpenCV library
- **data** - This folder contains two python scripts for adjusting the metadata for all the faces the robot can recognize, two csv files containing that metadata, .sh files that produce the greeting sound clips for each person, and subfolders for each person that the robot recognizes. Each folder contains all of the facial images that are stored for that person.
- **include** - This folder contains some .h files that match the .cpp files containing our code
- **launch** - This folder contains a launch file for running all of the nodes required for our robot to detect, recognize, greet, and learn faces.
- **src** - This folder contains the .cpp files that implement the majority of this project. The two files, main_recognition.cpp and main_detection.cpp, respectively create and handle the face_recognition and face_detection nodes. The core functionality for these nodes are stored in face_recognition.cpp and face_detection.cpp respectively. fps.cpp, storage.cpp, text.cpp, and image_proc.cpp are all files that we got from the authors of the Friendly Faces GitHub repo (https://github.com/Noviv/friendly_faces) and implement key subfunctions for our process.
- **docs** - This folder contains documentation, such as our final project writeup, our final presentation, and our demo video

Authors
----------------------------------
Anne Oursler, Leah Holden, Lauren Dierker


Licence
----------------------------------
Copyright (c) 2018. Anne Oursler, Leah Holden, and Lauren Dierker
Released to public domain under terms of the BSD Simplified license.
<http://www.opensource.org/licenses/bsd-license>


Acknoledgements
----------------------------------
Thank you to Jivko's former students: Noviv, nj1407, and
Marcus-Zhu for their friendly faces code
(https://github.com/Noviv/friendly_faces) which we used as a base
to get started with facial detection using OpenCV.

A special thank you to Jivko Sinapov for his help with facial
detection and facial recognition, his understanding, and
his support.
