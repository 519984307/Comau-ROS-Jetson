cmake_minimum_required(VERSION 3.0.2)
project(comau_action_handlers)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

# check c++17 / c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
  message(STATUS "Compile with C++17 support")
  add_compile_options(-std=c++17)
elseif(COMPILER_SUPPORTS_CXX11)
message(STATUS "Compile with C++11 support")
  add_compile_options(-std=c++11)  
elseif(COMPILER_SUPPORTS_CXX0X)
  message(STATUS "Compile with C++11 support")
  add_compile_options(-std=c++0x)
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++ support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

find_package(catkin 
  REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  comau_msgs
  trajectory_msgs
  control_msgs
  pcl_conversions 
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    moveit_handlers
  CATKIN_DEPENDS 
    actionlib
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    roscpp
    comau_msgs
    trajectory_msgs
    control_msgs
    pcl_conversions 
    pcl_ros
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(moveit_handlers
      
       src/moveit_handlers/comau_plan_action_server.cpp
       src/moveit_handlers/comau_perception_action_server.cpp
    
)
add_dependencies(moveit_handlers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_handlers
  ${catkin_LIBRARIES}
)

add_executable(comau_plan_action_server_node src/comau_plan_action_server_node.cpp)
add_dependencies(comau_plan_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_plan_action_server_node   
moveit_handlers
  ${catkin_LIBRARIES}
)

add_executable(comau_perception_action_server_node src/comau_perception_action_server_node.cpp)
add_dependencies(comau_perception_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_perception_action_server_node   
moveit_handlers
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(comau_plan_action_client.cpp src/comau_plan_action_client.cpp)
add_dependencies(comau_plan_action_client.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_plan_action_client.cpp   
moveit_handlers
  ${catkin_LIBRARIES}
)

add_executable(comau_perception_action_client.cpp src/comau_perception_action_client.cpp)
add_dependencies(comau_perception_action_client.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_perception_action_client.cpp   
moveit_handlers
  ${catkin_LIBRARIES}
)

install(TARGETS moveit_handlers 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
