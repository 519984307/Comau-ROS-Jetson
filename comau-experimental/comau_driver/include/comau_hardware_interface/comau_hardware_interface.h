/**
 * @file comau_hardware_interface.h
 * @author LMS
 * @brief
 * @version 0.1
 * @date 17-12-2019
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef ROS_CONTROL__COMAU_HARDWARE_INTERFACE_H
#define ROS_CONTROL__COMAU_HARDWARE_INTERFACE_H

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

#include <comau_hardware_interface/comau_hardware.h>

#include <comau_driver/comau_driver.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace comau_hardware_interface {

/**
 * @brief The ComauHardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class ComauHardwareInterface : public comau_hardware_interface::ComauHardware {
public:
  /**
   * @brief Construct a new Comau Hardware Interface object
   *
   */
  ComauHardwareInterface()= default;
  ~ComauHardwareInterface() override = default;
  /**
   * @brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * @param nh Root level ROS node handle
   * @param nh_local ROS node handle for the robot namespace
   *
   * @returns True, if the setup was performed successfully
   */
  bool init(ros::NodeHandle& nh, ros::NodeHandle& nh_local) override;
  /**
   * @brief Read method of the control loop. Reads a messages from the robot and handles and
   * publishes the information as needed.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  void read(const ros::Time& time, const ros::Duration& period) override;
  /**
   * @brief Write method of the control loop. Writes target joint positions to the robot to be read
   * by its PDL programs.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  void write(const ros::Time& time, const ros::Duration& period) override;
  
  /**
   * @brief Get the Control Frequency object
   * 
   * @return int The used control frequency
   */
  int getControlFrequency() const;

  /**
   * @brief Checks if a reset of the ROS controllers is necessary
   * 
   * @return Necessity of ROS controller reset 
   */
  bool shouldResetControllers();

protected:
  boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_;

  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  PositionJointInterface positionJointInterface;
  PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
  int loop_hz_;
  bool parallel_joint_fix_;
  double p_error_, v_error_, e_error_;
  bool controller_reset_necessary_;

  // boost::shared_ptr<controller_manager::ControllerManager> controller_manager_ptr_;

};

} // namespace comau_hardware_interface

#endif
