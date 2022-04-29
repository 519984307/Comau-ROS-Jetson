/**
 * @file comau_driver.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 21-03-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <pluginlib/class_loader.h>

#include <comau_driver/comau_driver.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_driver {
ComauRobot::ComauRobot(ros::NodeHandle &nh, ros::NodeHandle &nh_local)
    : name_("comau_driver"), nh_(nh), nh_local_(nh_local) {
  // Read parameters through ros parameter server
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_, "comau_driver/robot_ip", state_params_.server_ip_address);
  error += !rosparam_shortcuts::get(name_, nh_, "comau_driver/state_server_port", state_params_.server_port);
  error += !rosparam_shortcuts::get(name_, nh_, "comau_driver/motion_server_port", motion_params_.server_port);
  rosparam_shortcuts::shutdownIfError(name_, error);
  motion_params_.server_ip_address = state_params_.server_ip_address;
  state_params_.log_tag = "state_tcp_client";
  motion_params_.log_tag = "motion_tcp_client";
}

ComauRobot::~ComauRobot() {
  terminateMotion();
  state_client_ptr_->close();
  state_client_ptr_.reset();
  motion_client_ptr_->close();
  motion_client_ptr_.reset();
}

bool ComauRobot::initialize(bool use_state_server, bool use_motion_server) {

  pluginlib::ClassLoader<comau_tcp_interface::ComauClientBase> client_loader("comau_tcp_interface",
                                                                             "comau_tcp_interface::ComauClientBase");
  if (use_state_server) {
    try {
      state_client_ptr_.reset(new comau_tcp_interface::StateClient());
      if (!state_client_ptr_->initialize(state_params_)) {
        ROS_ERROR_STREAM("[comau_robot] State Client could not initialized ");
        return false;
      }
    } catch (pluginlib::PluginlibException &e) {
      ROS_ERROR_STREAM("[comau_robot] " << e.what());
      return false;
    }
  }
  if (use_motion_server) {
    // tcp interface motion client ptr
    try {
      motion_client_ptr_.reset(new comau_tcp_interface::MotionClient());
      if (!motion_client_ptr_->initialize(motion_params_)) {
        ROS_ERROR_STREAM("[comau_robot] Motion Client could not initialized ");
        return false;
      }
    } catch (pluginlib::PluginlibException &e) {
      ROS_ERROR_STREAM("[comau_robot] " << e.what());
      return false;
    }
  }

  dual_fix_ = nh_.param("use_dual", false);
  parallel_link_fix_ = nh_.param("parallel_joint_fix", false);

  return true;
} // namespace comau_driver

void ComauRobot::getJointPosition(std::vector<double> &joint_position) {
  msg->getData("joint_position", joints_float_);
  if (dual_fix_) {
    joints_float_[2] += 90.;
    joints_float_[4] -= 90.;
  }
  if (parallel_link_fix_)
    joints_float_[2] += joints_float_[1];
  joints_float_ = comau_tcp_interface::utils::toRad(joints_float_);
  joint_position.assign(joints_float_.begin(), joints_float_.end());

  return;
}

void ComauRobot::getEePosition(std::vector<double> &ee_position) {
  msg->getData("ee_position", ee_float_);
  ee_position.assign(ee_float_.begin(), ee_float_.end());
  return;
  // convert ee vector from float to double
}
void ComauRobot::getTimeStamp(int32_t &timestamp) {
  msg->getData("timestamp", timestamp);
  return;
}
void ComauRobot::getStatus(char &status) {
  msg->getData("robot_status", status);
  return;
}

bool ComauRobot::readMessagePackage() {
  msg = dynamic_cast<comau_tcp_interface::utils::MessagePackage *>(
      new comau_tcp_interface::utils::MessagePackage(state_client_ptr_->getRecvRecipe()));
  if (state_client_ptr_->getLastMessage(*msg)) {
    return true;
  }
  ROS_ERROR_STREAM("[comau_robot] Could not get Last Message Package");
  return false;
}

bool ComauRobot::terminateMotion() {
  return motion_client_ptr_->sendTerminateMessage();
}

bool ComauRobot::resetState() {
  // ROS_WARN("From driver reset");
  return motion_client_ptr_->sendResetMessage();
}

bool ComauRobot::writeTrajectoryCommand(trajectoryf_t &trajectory, ControlMode ControlMode) {
  if (ControlMode == ControlMode::MODE_JOINT_TRAJECTORY) {

    if (dual_fix_) {
      trajectoryf_t traj_transf;
      for (uint8_t i = 0; i < trajectory.size(); i++) {
        vector6f_t traj_point;
        for (uint8_t j = 0; j < 6; j++) {
          traj_point.at(j) = trajectory.at(i).at(j);
        }
        traj_point.at(2) = traj_point.at(2) - 90.0;
        traj_point.at(4) = traj_point.at(4) + 90.0;
        traj_transf.push_back(traj_point);
      }
      return motion_client_ptr_->sendJointTrajectoryMessage(traj_transf);
    }

    if (parallel_link_fix_) {
      trajectoryf_t traj_transf;
      for (uint8_t i = 0; i < trajectory.size(); i++) {
        vector6f_t traj_point;
        for (uint8_t j = 0; j < 6; j++) {
          traj_point.at(j) = trajectory.at(i).at(j);
        }
        traj_point.at(2) = traj_point.at(2) - traj_point.at(1);
        traj_transf.push_back(traj_point);
      }
      return motion_client_ptr_->sendJointTrajectoryMessage(traj_transf);
    }

    return motion_client_ptr_->sendJointTrajectoryMessage(trajectory);
  }
  if (ControlMode == ControlMode::MODE_CARTESIAN_TRAJECTORY) {
    return motion_client_ptr_->sendCartTrajectoryMessage(trajectory);
  }
  return true;
}

bool ComauRobot::writeCommand(const std::vector<double> &joint_command, ControlMode ControlMode) {
  if (ControlMode == ControlMode::MODE_SENSOR_TRACKING) {
    for (uint8_t i = 0; i < 3; i++) {
      sns_trk_cmd_.at(i) = joint_command.at(i) * 1000.0; //da m a mm
    }
    for (uint8_t i = 3; i < 6; i++) {
      sns_trk_cmd_.at(i) = joint_command.at(i) * 57.2957795; //da rad a grd
    }
    return motion_client_ptr_->sendSensorTrackingMessage(sns_trk_cmd_);
  }
  return true;
}

void ComauRobot::enableAllowAsync() {
  allow_async_ = true;
}
void ComauRobot::desableAllowAsync() {
  allow_async_ = false;
}
bool ComauRobot::checkAllowAsync() {
  return allow_async_;
}

} // namespace comau_driver
