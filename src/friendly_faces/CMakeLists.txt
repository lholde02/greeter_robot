cmake_minimum_required(VERSION 2.8.3)
project(friendly_faces)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	OpenCV
	sound_play
	cv_bridge
	image_transport
	sensor_msgs
	std_msgs
	turtlesim
	message_generation
	geometry_msgs
	actionlib)

catkin_package(INCLUDE_DIRS include/ ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# BUILD

include_directories(include/ ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(friendly_faces_runner
	src/storage.cpp
	src/fps.cpp
	src/image_proc.cpp
	src/text.cpp
	src/main.cpp
)
target_link_libraries(friendly_faces_runner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
