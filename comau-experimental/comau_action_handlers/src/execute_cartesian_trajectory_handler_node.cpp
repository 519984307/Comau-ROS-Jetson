/**
 * @file execute_cartesian_trajectory_handler_node.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <csignal>
#include <ros/ros.h>

#include <comau_handlers/execute_cartesian_trajectory_handler.h>

std::unique_ptr<comau_driver::handler::ExecuteCartesianTrajectoryHandler> move_cartesian_handler_ptr;

void signalHandler(int signum) {

  ROS_WARN_STREAM("[comau_joint_motion_handler] Interrupt signal (" << signum << ") received.\n");

  move_cartesian_handler_ptr.reset();

  exit(signum);
}

int main(int argc, char **argv) {
  const std::string server_name = "execute_cartesian_trajectory_handler";
  const std::string server_node_name = server_name + SERVER_NODE_NAME_POSTFIX;
  const std::string server_action_name = server_name + SERVER_ACTION_NAME_POSTFIX;

  ros::init(argc, argv, server_node_name, ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  ROS_INFO_STREAM("[" << server_node_name << "] "
                      << "Initializing node with ns :" << nh.getNamespace());
  move_cartesian_handler_ptr.reset(
      new comau_driver::handler::ExecuteCartesianTrajectoryHandler(nh, nh_local, server_name));

  if (!move_cartesian_handler_ptr->initialize()) {
    ROS_ERROR_STREAM("[" << server_node_name << "] "
                         << "Handler initialization error");
    exit(1);
  }

  ros::spin();
  return 1;
}
