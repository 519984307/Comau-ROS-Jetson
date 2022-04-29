cmake_minimum_required(VERSION 3.0.2)
project(comau_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  tf
  tf2_ros
  geometry_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
   cfg/SnsTrkPlot.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_sin_pub src/twist_sin_pub.cpp)
add_dependencies(twist_sin_pub ${PROJECT_NAME}_gencfg)
target_link_libraries(twist_sin_pub ${catkin_LIBRARIES})
