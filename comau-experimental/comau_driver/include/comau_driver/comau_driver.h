/**
 * @file comau_driver.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 21-03-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef COMAU_DRIVER_H
#define COMAU_DRIVER_H

#include "comau_tcp_interface/comau_motion_client.h"
#include "comau_tcp_interface/comau_state_client.h"
#include "comau_tcp_interface/comau_tcp_interface.h"
#include <boost/scoped_ptr.hpp>
// For publishing End Effector TF
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_driver {
/*!
 * \brief Possible states for robot control mode
 */
enum class ControlMode {
  MODE_POSITION,
  MODE_VELOCITY,
  MODE_SENSOR_TRACKING,
  MODE_JOINT_TRAJECTORY,
  MODE_CARTESIAN_TRAJECTORY
};
/**
 * @brief This is the main class for interfacing the Robot controller.
 *
 * It sets up all the necessary socket connections and handles the data exchange with the robot.
 */
class ComauRobot {
public:
  /**
   * @brief Construct a new Comau Robot object
   *
   * @param nh global ROS NodeHandle
   * @param nh_local local ROS NodeHandle
   */
  ComauRobot(ros::NodeHandle &nh, ros::NodeHandle &nh_local);
  /**
   * @brief Destroy the Comau Robot object
   *
   */
  ~ComauRobot();
  /**
   * @brief
   *
   * @return true If comau robot initialized correctly
   */
  bool initialize(bool use_state_server, bool use_motion_server);
  /**
   * @brief Getter for the robot Message Package
   *
   */
  bool readMessagePackage();
  /**
   * @brief Write a joint command using the Motion Client
   *
   * @param joint_command desired joint positions
   * @return true on successful write
   * @return false on failed write
   */
  bool terminateMotion();
  bool writeCommand(const std::vector<double> &joint_command, ControlMode ControlMode);
  bool writeTrajectoryCommand(trajectoryf_t &trajectory, ControlMode ControlMode);
  void getJointPosition(std::vector<double> &joint_position);
  void getEePosition(std::vector<double> &joint_position);
  void getTimeStamp(int32_t &timestamp);
  void getStatus(char &status);
  void enableAllowAsync();
  void desableAllowAsync();
  bool checkAllowAsync();
  bool resetState();

private:
  std::string name_;
  ros::NodeHandle nh_, nh_local_; /**< ROS NodeHandle objects required for parameters reading */

  boost::shared_ptr<comau_tcp_interface::StateClient> state_client_ptr_;   /**< StateClient object */
  boost::shared_ptr<comau_tcp_interface::MotionClient> motion_client_ptr_; /**< StateClient object */
  comau_tcp_interface::utils::MessagePackage *msg;
  comau_tcp_interface::ComauTcpInterfaceParameters state_params_, motion_params_;
  bool dual_fix_ = false, parallel_link_fix_ = false;
  bool allow_async_ = false;
  comau_tcp_interface::utils::vector6f_t joints_float_;
  comau_tcp_interface::utils::vector6f_t ee_float_;
  comau_tcp_interface::utils::vector6f_t joint_command_float_;
  comau_tcp_interface::utils::vector6f_t sns_trk_cmd_;
};

} // namespace comau_driver

#endif // COMAU_DRIVER_H
