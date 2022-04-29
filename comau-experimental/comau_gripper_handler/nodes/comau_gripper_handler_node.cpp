/**
 * @file comau_gripper_handler_node.cpp
 * @author lms
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <comau_gripper_handler/comau_gripper_handler.h>
#include <csignal>
#include <ros/ros.h>

// unique smart pointer
std::unique_ptr<comauGripperHandler> comau_gripper_handler_ptr;

int main(int argc, char **argv) {

  const std::string node_name = "comau_gripper_handler_node";
  ros::init(argc, argv, node_name, ros::init_options::NoRosout);

  ros::NodeHandle nh("");

  ROS_INFO_STREAM("[" << node_name << "]"
                      << "Initializing node with ns :" << nh.getNamespace());

  // instansiate the smart pointer
  comau_gripper_handler_ptr.reset(new comauGripperHandler());
  comau_gripper_handler_ptr->initialize(true);

  // spin the node
  ros::spin();

  return 0;
}
