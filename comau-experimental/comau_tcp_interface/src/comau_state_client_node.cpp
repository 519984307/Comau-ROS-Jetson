/**
 * @file comau_state_client_node.cpp
 * @author LMS ()
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <csignal>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "comau_tcp_interface/comau_client_base.h"

using namespace comau_tcp_interface;

[[noreturn]] void signalHandler(int signum) {

  ROS_WARN_STREAM("[comau_state_client_node] Interrupt signal (" << signum << ") received.\n");

  exit(signum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "comau_state_client_node", ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  ComauTcpInterfaceParameters params;

  sleep(3);
  // Read parameters through ros parameter server

  nh.param<std::string>("robot_ip", params.server_ip_address, "192.168.56.2");
  nh.param<std::string>("state_server_port", params.server_port, "1104");
  params.log_tag = "[comau_state_client_node] ";

  pluginlib::ClassLoader<ComauClientBase> client_loader("comau_tcp_interface", "comau_tcp_interface::ComauClientBase");

  try {
    boost::shared_ptr<ComauClientBase> state_client = client_loader.createInstance("comau_tcp_interface::StateClient");

    if (state_client->initialize(params)) {
      utils::MessagePackage msg(state_client->getRecvRecipe());
      while (ros::ok()) {
        if (state_client->getLastMessage(msg)) {
          ros::Duration(0.1).sleep();
          ROS_INFO_STREAM(msg.toString());
        }
      }

      state_client->close();
    } else {
      ROS_ERROR_STREAM("[comau_state_client_node] Error at state client initialize");
    }
  } catch (pluginlib::PluginlibException &e) {
    ROS_ERROR_STREAM("[comau_state_client_node]" << e.what());
  }

  ros::shutdown();

  return 0;
}
