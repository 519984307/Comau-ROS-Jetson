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
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <comau_controllers/sensor_tracking_controller.h>
#include <comau_driver/comau_driver.h>
#include <comau_handlers/execute_cartesian_trajectory_handler.h>
#include <comau_handlers/execute_joint_trajectory_handler.h>
#include <comau_msgs/ComauRobotStatus.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>

namespace comau_hardware_interface {

/**
 * @brief The ComauHardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class ComauHardwareInterface : public hardware_interface::RobotHW {
public:
  /**
   * @brief Construct a new Comau Hardware Interface object
   */
  ComauHardwareInterface();
  virtual ~ComauHardwareInterface() = default;
  /**
   * @brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * @param nh Root level ROS node handle
   * @param nh_local ROS node handle for the robot namespace
   * @returns True, if the setup was performed successfully
   *
   */
  virtual bool init(ros::NodeHandle &nh, ros::NodeHandle &nh_local) override;
  /**
   * @brief Read method of the control loop. Reads a messages from the robot and handles and
   * publishes the information as needed.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time &time, const ros::Duration &period) override;
  /**
   * @brief Write method of the control loop. Writes target joint positions to the robot to be read
   * by its PDL programs.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time &time, const ros::Duration &period) override;
  /**
   * @brief extract data end effector position from robot - pdl
   *
   * @return
   */
  void extractEndEffectorPose();
  /**
   * @brief end effector position from robot - pdl and publish TF
   *
   * @return
   */
  void publishEndEffectorPose();
  /**
   * @brief Publishes the Robot status
   * @return
   */
  void publishRobotStatus();
  /**
   * @brief Keeps the connection alive by sending a standard message (NOT USED)
   *
   * @return true on successful write
   * @return false on failed write
   */
  bool holdConnection();
  /**
   * @brief Prints a double vector in the console for debugging
   *
   * @param vec The vector to be printed in the console
   */
  void printVector(const std::vector<double> &vec);
  /**
   * @brief Checks if the command is zero
   *
   * @param vec The vector to be checked if it is 0
   */
  bool ifZero(const std::vector<double> &vec);
  /**
   * @brief Assigns the source vector into the destination vector
   *
   * @param src The source vector
   * @param dest The destination vector
   */
  void copyVector(const std::vector<double> &src, std::vector<double> &dest);
  /**
   * @brief Checks if a reset of the controllers is necessary
   */
  bool shouldResetControllers();
  /*!
   * \brief Preparation to start and stop loaded controllers.
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   *
   * \returns True, if the controllers can be switched
   */
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                             const std::list<hardware_interface::ControllerInfo> &stop_list) override;
  /*!
   * \brief Starts and stops controllers.
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                        const std::list<hardware_interface::ControllerInfo> &stop_list) override;

  /*!
   * \brief Checks whether a resource list contains joints from this hardware interface
   *
   * True is returned as soon as one joint name from claimed_resources matches a joint from this
   * hardware interface.
   */
  bool checkControllerClaims(const std::set<std::string> &claimed_resources);
  /*!
   * \brief Checks if the URCaps program is running on the robot.
   *
   * \returns True, if the program is currently running, false otherwise.
   */
  bool isRobotProgramRunning() const;

protected:
  // Name of this class
  std::string name_ = "comau_hardware_interface";
  ros::NodeHandle nh_, nh_local_;
  // Robot Driver Pointer
  boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_;
  // Configuration
  bool position_controller_running_;
  bool velocity_controller_running_;
  bool sensor_tracking_controller_running_;
  bool controllers_initialized_;
  bool robot_program_running_ = true; /* TODO */
  // Hardware Interfaces
  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::EffortJointInterface ej_interface_;
  comau_controllers::SensorTrackingController sensor_tracking_interface_;
  // industrial_robot_status_interface::IndustrialRobotStatusInterface robot_status_interface_{};
  // States
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> ee_position_;
  // industrial_robot_status_interface::RobotStatus robot_status_resource_{};
  // Commands
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> sensor_tracking_command_;
  // private
  size_t num_joints_;
  std::vector<std::string> joint_names_;
  uint64_t loop_hz_;
  bool use_state_server_, use_motion_server_;
  int32_t data_timestamp;
  char robot_status;
  bool packet_read_;
  geometry_msgs::TransformStamped ee_transform_; // TODO : change timestamp to ros::Time
  tf2::Quaternion q;
  // publishers
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>> async_enable_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::ComauRobotStatus>>
      robot_status_pub_; /**< ROS status publisher see ComauRobotStatus.msg */
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> ee_pose_pub_;
  // comau action handlers
  const std::string execute_joint_server_name_ = "execute_joint_trajectory_handler";
  std::unique_ptr<comau_action_handlers::ExecuteJointTrajectoryHandler> execute_joints_handler_ptr;
  const std::string execute_cartesian_server_name_ = "execute_cartesian_trajectory_handler";
  std::unique_ptr<comau_action_handlers::ExecuteCartesianTrajectoryHandler> execute_cartesian_handler_ptr;
};

} // namespace comau_hardware_interface

#endif
