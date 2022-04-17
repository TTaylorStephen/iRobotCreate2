cmake_minimum_required(VERSION 2.8.3)
project(master)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package(
	CATKIN_DEPENDS message_runtime ir_odom
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/master_main.cpp src/create_config.cpp src/create_comm.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



