cmake_minimum_required(VERSION 2.8.3)
project(greeter_robot)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	sound_play
	cv_bridge
	image_transport
	sensor_msgs
	std_msgs
	turtlesim
	message_generation
	geometry_msgs
	actionlib
	actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include/ ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
	CATKIN_DEPENDS actionlib std_msgs
)

# BUILD

include_directories(include/ ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(greeter_robot_runner
	src/storage.cpp
	src/fps.cpp
	src/image_proc.cpp
	src/text.cpp
	src/face_detection.cpp
	src/main.cpp
	src/face_detection.cpp
	src/face_recognition.cpp
)
add_dependencies(greeter_robot_runner ${catkin_EXPORTED_TARGETS})
target_link_libraries(greeter_robot_runner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})