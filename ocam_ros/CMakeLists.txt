cmake_minimum_required(VERSION 2.8.3)
project(ocam_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  image_transport
  camera_info_manager
  cv_bridge
)
find_package(Threads)
find_package(OpenCV REQUIRED)
find_package(udev)
find_package(v4l2)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -O3 -Wl,--no-as-needed")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -O3 -Wl,--no-as-needed")

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs camera_info_manager image_transport cv_bridge
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(ocam_ros_node
        src/ocam_ros.cpp
        src/withrobot_camera.cpp
        src/withrobot_utility.cpp
        include/withrobot_camera.hpp
        include/withrobot_utility.hpp
)
target_link_libraries(ocam_ros_node ${catkin_LIBRARIES} ${OpenCV_LIBS} udev v4l2 pthread)






