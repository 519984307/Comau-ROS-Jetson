cmake_minimum_required(VERSION 2.8.3)
project(comau_driver)

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
  roscpp
  actionlib
  hardware_interface
  controller_manager
  pluginlib
  joint_limits_interface
  comau_tcp_interface
  tf2_ros
  comau_controllers
  industrial_robot_status_interface
  rosparam_shortcuts
  kdl_parser
  std_msgs
  std_srvs
  geometry_msgs
  comau_msgs
  trajectory_msgs
  control_msgs
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    comau_driver
    comau_hardware_control_loop
    comau_hardware_interface
    comau_handlers
  CATKIN_DEPENDS 
    roscpp
    actionlib
    hardware_interface
    controller_manager
    pluginlib
    joint_limits_interface
    comau_tcp_interface
    tf2_ros
    comau_controllers
    industrial_robot_status_interface
    rosparam_shortcuts
    kdl_parser
    std_msgs
    std_srvs
    geometry_msgs
    comau_msgs
    trajectory_msgs
    control_msgs
  DEPENDS 
    Boost
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 ${Eigen_INCLUDE_DIRS}
)

# Control loop
add_library(comau_hardware_control_loop src/comau_hardware_interface/comau_hw_control_loop.cpp)
target_link_libraries(comau_hardware_control_loop
  ${catkin_LIBRARIES}
)

# Comau Driver
add_library(comau_driver
  src/comau_driver/comau_driver.cpp
)
add_dependencies(comau_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_driver
  ${catkin_LIBRARIES}
)

## Comau Action Handlers
add_library(comau_handlers
  src/comau_handlers/execute_joint_trajectory_handler.cpp
  src/comau_handlers/execute_cartesian_trajectory_handler.cpp
)
add_dependencies(comau_handlers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_handlers
  comau_driver
  ${catkin_LIBRARIES}
)

# Comau Hardware Interface
add_library(comau_hardware_interface
  src/comau_hardware_interface/comau_hw_interface.cpp
)
add_dependencies(comau_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_hardware_interface  
  comau_driver
  comau_handlers
  ${catkin_LIBRARIES}
)

# Comau Hardware Interface node
add_executable(comau_hardware_interface_node src/comau_hw_interface_node.cpp)
add_dependencies(comau_hardware_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_hardware_interface_node
  comau_driver
  comau_hardware_interface
  comau_handlers
  comau_hardware_control_loop
  ${catkin_LIBRARIES}
)

# Switch Controllers
add_executable(switch_controllers src/switch_controllers_client.cpp)
add_dependencies(switch_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(switch_controllers
  ${catkin_LIBRARIES}
)

install(TARGETS comau_driver comau_hardware_interface comau_handlers comau_hardware_control_loop comau_hardware_interface_node
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

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
