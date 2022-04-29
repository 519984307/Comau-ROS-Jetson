cmake_minimum_required(VERSION 3.0.2)
project(comau_gripper_handler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -Wextra -g3 -O0 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -Wextra -g0 -Ofast")
# Set the build type
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  comau_msgs
  pluginlib
  comau_tcp_interface
  rosparam_shortcuts
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES comau_gripper_handler
  CATKIN_DEPENDS
    roscpp
    pluginlib
    comau_tcp_interface
    std_msgs
    comau_msgs
    rosparam_shortcuts
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Comau Gripper Library
add_library(comau_gripper_handler src/comau_gripper_handler.cpp)
add_dependencies(comau_gripper_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_gripper_handler ${catkin_LIBRARIES})

# comau_gripper_handler_node executable
add_executable(comau_gripper_handler_node nodes/comau_gripper_handler_node.cpp)
add_dependencies(comau_gripper_handler_node ${PROJECT_NAME})
target_link_libraries(comau_gripper_handler_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})