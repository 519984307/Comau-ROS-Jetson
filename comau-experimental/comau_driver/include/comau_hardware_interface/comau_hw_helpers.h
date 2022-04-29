#pragma once
#include <comau_hardware_interface/comau_hw_interface.h>
#include <sstream>

namespace comau_hardware_interface {

void ComauHardwareInterface::publishRobotStatus() {
  if (robot_status_pub_) {
    if (robot_status == comau_tcp_interface::RobotStatus::TERMINATE) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::ComauRobotStatus::TERMINATE;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status == comau_tcp_interface::RobotStatus::READY) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::ComauRobotStatus::READY;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status == comau_tcp_interface::RobotStatus::MOVING) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::ComauRobotStatus::MOVING;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status == comau_tcp_interface::RobotStatus::PAUSED) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::ComauRobotStatus::PAUSED;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status == comau_tcp_interface::RobotStatus::RESUMING) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::ComauRobotStatus::RESUMING;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status == comau_tcp_interface::RobotStatus::SUCCEEDED) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::ComauRobotStatus::SUCCEEDED;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status == comau_tcp_interface::RobotStatus::ERROR) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::ComauRobotStatus::ERROR;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status == comau_tcp_interface::RobotStatus::CANCELING) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::ComauRobotStatus::CANCELING;
        robot_status_pub_->unlockAndPublish();
      }
    } else {
      ROS_ERROR_STREAM("[comau_robot] "
                       << "Unknow status type.");
      ROS_ERROR_STREAM("[comau_robot] " << robot_status_pub_->msg_.status);
    }
  }
}

void ComauHardwareInterface::extractEndEffectorPose() {
  ee_transform_.header.stamp = ros::Time::now();
  ee_transform_.header.frame_id = "base_link";
  ee_transform_.child_frame_id = "tool_controller";
  ee_transform_.transform.translation.x = ee_position_[0] / 1000.;
  ee_transform_.transform.translation.y = ee_position_[1] / 1000.;
  ee_transform_.transform.translation.z = ee_position_[2] / 1000.;
  q.setRPY(ee_position_[3] / 180. * M_PI, ee_position_[4] / 180. * M_PI, ee_position_[5] / 180. * M_PI);
  ee_transform_.transform.rotation.x = q.x();
  ee_transform_.transform.rotation.y = q.y();
  ee_transform_.transform.rotation.z = q.z();
  ee_transform_.transform.rotation.w = q.w();
}
void ComauHardwareInterface::publishEndEffectorPose() {
  if (ee_pose_pub_) {
    if (ee_pose_pub_->trylock()) {
      ee_pose_pub_->msg_.transforms.clear();
      ee_pose_pub_->msg_.transforms.push_back(ee_transform_);
      ee_pose_pub_->unlockAndPublish();
    }
  }
}
bool ComauHardwareInterface::shouldResetControllers() {
  return false;
}

bool ComauHardwareInterface::holdConnection() {
  // TODO add the implementation
  return true;
}

void ComauHardwareInterface::copyVector(const std::vector<double> &src, std::vector<double> &dest) {
  for (size_t i = 0; i < src.size(); i++)
    dest.at(i) = src.at(i);
}

bool ComauHardwareInterface::ifZero(const std::vector<double> &vec) {
  for (double val : vec)
    if (val != 0.0)
      return false;
  return true;
}

void ComauHardwareInterface::printVector(const std::vector<double> &vec) {
  ROS_INFO("Vector : %f %f %f %f %f %f", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

bool ComauHardwareInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                           const std::list<hardware_interface::ControllerInfo> &stop_list) {
  bool ret_val = true;
  if (controllers_initialized_ && !isRobotProgramRunning() && !start_list.empty()) {
    for (auto &controller : start_list) {
      if (!controller.claimed_resources.empty()) {
        ROS_ERROR_STREAM("Robot control is currently inactive. Starting controllers that claim resources is currently "
                         "not possible. Not starting controller '"
                         << controller.name << "'");
        ret_val = false;
      }
    }
  }

  controllers_initialized_ = true;
  return ret_val;
}

void ComauHardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                      const std::list<hardware_interface::ControllerInfo> &stop_list) {
  for (auto &controller_it : stop_list) {
    for (auto &resource_it : controller_it.claimed_resources) {
      if (checkControllerClaims(resource_it.resources)) {
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface") {
          position_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::EffortJointInterface") {
          sensor_tracking_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "comau_controllers::SensorTrackingController") {
          sensor_tracking_controller_running_ = false;
        }
      }
    }
  }
  for (auto &controller_it : start_list) {
    for (auto &resource_it : controller_it.claimed_resources) {
      if (checkControllerClaims(resource_it.resources)) {
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface") {
          position_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::EffortJointInterface") {
          sensor_tracking_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "comau_controllers::SensorTrackingController") {
          sensor_tracking_controller_running_ = true;
        }
      }
    }
  }
  if (async_enable_pub_) {
    if (async_enable_pub_->trylock()) {
      if (position_controller_running_ || sensor_tracking_controller_running_) {
        async_enable_pub_->msg_.data = false;
        robot_ptr_->desableAllowAsync();
      } else {
        async_enable_pub_->msg_.data = true;
        robot_ptr_->enableAllowAsync();
      }
      async_enable_pub_->unlockAndPublish();
    }
  }
  if (use_motion_server_)
    robot_ptr_->resetState();
}

bool ComauHardwareInterface::checkControllerClaims(const std::set<std::string> &claimed_resources) {
  for (const std::string &it : joint_names_) {
    for (const std::string &jt : claimed_resources) {
      if (it == jt) {
        return true;
      }
    }
  }
  return false;
}

bool ComauHardwareInterface::isRobotProgramRunning() const {
  return robot_program_running_;
}

} // namespace comau_hardware_interface