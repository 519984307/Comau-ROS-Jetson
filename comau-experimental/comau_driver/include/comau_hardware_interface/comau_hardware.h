/**
 * @file comau_hardware.h
 * @author LMS
 * @brief
 * @version 0.1
 * @date 17-12-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef ROS_CONTROL__COMAU_HARDWARE_H
#define ROS_CONTROL__COMAU_HARDWARE_H

#include <boost/scoped_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>

namespace comau_hardware_interface {
/**
 * \brief The ComauHardware class
 */
class ComauHardware : public hardware_interface::RobotHW {
protected:
  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  // joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
  // joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;

  // Custom or available transmissions
  // transmission_interface::RRBOTTransmission rrbot_trans_;
  // std::vector<transmission_interface::SimpleTransmission> simple_trans_;

  // Shared memory
  size_t num_joints_;
  int joint_mode_; // position, velocity, or effort
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
};

} // namespace comau_hardware_interface

#endif // ROS_CONTROL__COMAU_HARDWARE_H
