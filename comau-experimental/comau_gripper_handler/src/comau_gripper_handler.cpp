/**
 * @file comau_point_follower.cpp
 * @author lms
 * @brief
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <comau_gripper_handler/comau_gripper_handler.h>

// constructor
comauGripperHandler::comauGripperHandler() : name_("comau_gripper_handler") {

  // Load parameters
  ros::NodeHandle nh_priv(name_);
  nh_priv_ = nh_priv;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_priv_, "robot_ip", gripper_params_.server_ip_address);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "gripper_server_port", gripper_params_.server_port);
  rosparam_shortcuts::shutdownIfError(name_, error);
  gripper_params_.log_tag = "gripper_tcp_client";

  // Instantiate gripper service server
  gripper_service_ = nh_priv_.advertiseService("/gripper_command", &comauGripperHandler::gripper_routine, this);
}

bool comauGripperHandler::initialize(bool use_motion_server) {
  pluginlib::ClassLoader<comau_tcp_interface::ComauClientBase> client_loader("comau_tcp_interface",
                                                                             "comau_tcp_interface::ComauClientBase");
  if (use_motion_server) {
    // tcp interface gripper client ptr
    try {
      gripper_client_ptr_.reset(new comau_tcp_interface::MotionClient());
      if (!gripper_client_ptr_->initialize(gripper_params_)) {
        ROS_ERROR_STREAM("[comau_robot] Gripper Command Client could not initialized ");
        return false;
      }
    } catch (pluginlib::PluginlibException &e) {
      ROS_ERROR_STREAM("[comau_robot] " << e.what());
      return false;
    }
  }
  return true;
}

bool comauGripperHandler::gripper_routine(comau_msgs::GripperCommand::Request &req,
                                          comau_msgs::GripperCommand::Response &res) {
  bool gripper_command = req.gripper_command;
   
  if (req.gripper_command) {
    return gripper_client_ptr_->sendGripperMessage(gripper_command);
  } else {
    return gripper_client_ptr_->sendGripperMessage(gripper_command);
  }
  res.success = true;
  return true;
}

// destructor
comauGripperHandler::~comauGripperHandler() {}
