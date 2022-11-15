cmake_minimum_required(VERSION 3.0.2)
project(kinect_proc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED geometry_msgs roscpp rospy sensor_msgs std_msgs pcl_ros cv_bridge)
find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
add_executable(kinect_proc_node src/kinect_proc_node.cpp)
add_dependencies(kinect_proc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinect_proc_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})


