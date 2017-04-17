cmake_minimum_required(VERSION 2.8.3)
project(friendly_faces)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(OpenCV 3 REQUIRED)

catkin_package(INCLUDE_DIRS include/)

# BUILD

include_directories(include/ ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(friendly_face_runner
	src/ff_db/storage.cpp
	src/ff_monitor/fps.cpp
	src/ff_proc/image_proc.cpp
	src/ff_proc/frame_proc.cpp
	src/ff_text/positioning.cpp
	src/main.cpp
)
target_link_libraries(friendly_face_runner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
