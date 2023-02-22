cmake_minimum_required(VERSION 3.0.2)
project(StereoSynch)

find_package(catkin REQUIRED COMPONENTS rosbag message_filters roscpp sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${rosbag_LIBRARIES})

