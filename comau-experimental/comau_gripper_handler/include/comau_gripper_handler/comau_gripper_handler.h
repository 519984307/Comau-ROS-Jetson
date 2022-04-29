/**
 * @file comau_point_follower.h
 * @author lms
 * @brief
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

#include <ros/ros.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>
// boost
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
// comau tcp
#include <comau_tcp_interface/comau_motion_client.h>
#include <comau_tcp_interface/comau_tcp_interface.h>
// #include "comau_tcp_interface/comau_state_client.h"
// msgs
#include <comau_msgs/GripperCommand.h>
#include <std_msgs/Bool.h>
// pluginlib
#include <pluginlib/class_loader.h>
/**
 * @brief This class contains the functionality to communicate with pdl gripper through tcp
 */
class comauGripperHandler {
public:
  /**
   * @brief Construct a new comauGripperHandler object
   */
  comauGripperHandler();
  /**
   * @brief Destroy the comauGripperHandler object
   */
  ~comauGripperHandler();
  /**
   * @brief Initializes the gripper handler
   * @param use_motion_server if we are using the motion server
   * @return true If comau robot initialized correctly
   */
  bool initialize(bool use_motion_server);
  /**
   * @brief Service Function to communicate with gripper
   */
  bool gripper_routine(comau_msgs::GripperCommand::Request &req, comau_msgs::GripperCommand::Response &res);

private:
  std::string name_;        /** Name of this class (for parameter loading) */
  ros::NodeHandle nh_priv_; /** node handle */
  boost::shared_ptr<comau_tcp_interface::MotionClient> gripper_client_ptr_; /**< GripperClient object */
  comau_tcp_interface::ComauTcpInterfaceParameters gripper_params_;
  ros::ServiceServer gripper_service_;
};
