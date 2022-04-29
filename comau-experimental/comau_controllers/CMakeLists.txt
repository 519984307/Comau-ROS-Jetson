cmake_minimum_required(VERSION 3.0.2)
project(comau_controllers)

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

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_toolbox
  kdl_parser
  geometry_msgs
  hardware_interface
  joint_trajectory_controller
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  comau_msgs
  tf
)

#find_package(Eigen3 REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    comau_controllers
  CATKIN_DEPENDS 
    control_toolbox
    controller_interface 
    geometry_msgs 
    hardware_interface 
    joint_trajectory_controller 
    pluginlib 
    realtime_tools 
    roscpp 
    std_msgs
    comau_msgs
    tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(comau_controllers
   src/comau_controllers/sensor_tracking_controller.cpp
)

add_dependencies(comau_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_controllers
  ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN ".svn"
        EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
