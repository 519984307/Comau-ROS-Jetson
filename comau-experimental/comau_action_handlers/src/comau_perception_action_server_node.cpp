#include <csignal>
#include <ros/ros.h>

#include "moveit_handlers/comau_perception_action_server.h"

std::unique_ptr<PerceptionHandlerServer> perception_handler_ptr;

void signalHandler(int signum) {
  ROS_WARN_STREAM("[perception_handler] Interrupt signal (" << signum << ") received.\n");
  perception_handler_ptr.reset();


  exit(signum);
}

int main(int argc, char **argv) {
  const std::string server_name = "perception_handler_server";
  const std::string server_node_name = server_name + SERVER_NODE_NAME_POSTFIX;
  const std::string server_action_name = server_name + SERVER_ACTION_NAME_POSTFIX;

  ros::init(argc, argv, server_node_name, ros::init_options::NoRosout);

  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  ROS_INFO_STREAM("[" << server_node_name << "]"
                      << "Initializing node with ns :" << nh.getNamespace());
  perception_handler_ptr.reset(new PerceptionHandlerServer(nh, nh_local, server_action_name));

  if (!perception_handler_ptr->initialize()) {
    ROS_ERROR_STREAM("[" << server_node_name << "]"
                         << "Perception Handler initialization error");
    exit(1);
  }
  ros::spin();
  return 1;
}
