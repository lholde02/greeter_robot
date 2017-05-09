cmake_minimum_required(VERSION 2.8.3)
project(friendly_faces)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(OpenCV 3 REQUIRED)
find_package(sound_play REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(message_generation REQUIRED)
find_package(geometry_msgs REQUIRED)

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
