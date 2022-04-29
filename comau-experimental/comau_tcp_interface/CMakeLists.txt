cmake_minimum_required(VERSION 2.8.3)
project(comau_tcp_interface)

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
  roscpp
  comau_msgs
  pluginlib 
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES comau_tcp_interface_plugins
  CATKIN_DEPENDS 
    roscpp 
    comau_msgs
  DEPENDS 
    Boost 
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(comau_tcp_interface_plugins
  src/${PROJECT_NAME}/utils/message_package.cpp
  src/${PROJECT_NAME}/comau_tcp_interface.cpp
  src/${PROJECT_NAME}/comau_state_client.cpp
  src/${PROJECT_NAME}/comau_motion_client.cpp
)
add_dependencies(comau_tcp_interface_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_tcp_interface_plugins ${catkin_LIBRARIES})
target_compile_features(comau_tcp_interface_plugins PRIVATE cxx_std_17)


add_executable(comau_state_client_node src/comau_state_client_node.cpp)
add_dependencies(comau_state_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_state_client_node 
  ${catkin_LIBRARIES}
  comau_tcp_interface_plugins
)
add_executable(comau_motion_client_node src/comau_motion_client_node.cpp)
add_dependencies(comau_motion_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_motion_client_node
  ${catkin_LIBRARIES}
  comau_tcp_interface_plugins
)

