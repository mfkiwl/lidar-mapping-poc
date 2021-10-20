cmake_minimum_required(VERSION 3.0.2)
project(gnss_parser)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/
)

link_directories(
  /usr/local/lib
)

add_executable(
  ${PROJECT_NAME}_node 
    src/gnss_parser.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  rt
  pthread
  serial
)
