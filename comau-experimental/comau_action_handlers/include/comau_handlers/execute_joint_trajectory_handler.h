/**
 * @file execute_joint_trajectory_handler.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 15-05-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

#include <limits>
#include <ros/ros.h>
#include <utility>

#include <actionlib/server/simple_action_server.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

#include "comau_tcp_interface/comau_motion_client.h"
#include "comau_tcp_interface/comau_tcp_interface.h"
#include "comau_tcp_interface/utils/message_package.h"

// msgs
#include "comau_msgs/ActionResultStatusConstants.h"
#include "comau_msgs/ComauRobotStatus.h"
#include "comau_msgs/ExecuteJointTrajectoryAction.h"

#include "naming_constants.h"

using namespace comau_tcp_interface;

namespace comau_driver {
namespace handler {
typedef actionlib::SimpleActionServer<comau_msgs::ExecuteJointTrajectoryAction> ExecuteJointTrajectoryActionServer;

/**
 * @brief This handler accepts as goal a joint space trajectory and send it to
 * Robot with MotionClient
 *
 */
class ExecuteJointTrajectoryHandler {
public:
  /**
   * @brief Construct a new MoveJointsServer object
   *
   * @param nh
   * @param nh_local
   * @param name
   */
  ExecuteJointTrajectoryHandler(ros::NodeHandle &nh, ros::NodeHandle &nh_local, std::string name);

  /**
   * @brief Destroy the Move Joints Handler object
   *
   */
  ~ExecuteJointTrajectoryHandler();
  /**
   * @brief Initialization function
   *
   * @return true
   * @return false if something went wrong at the initialization
   */
  bool initialize();

private:
  /**
   * @brief Get the URDF XML from the parameter server
   *
   * @param nh
   * @param robot_descr_param_name
   * @return true
   * @return false
   */
  bool loadJointLimits(const ros::NodeHandle &nh, std::string robot_descr_param_name);

  /**
   * @brief Robot status subscribers's callback function.
   *
   * @param msg
   */
  void statusCallback(const comau_msgs::ComauRobotStatusConstPtr &msg);
  /**
   * @brief Conversion function ROS trajectory_msgs/JointTrajectoryPoint[]
   * message to custom utils::trajectoryf_t
   *
   * @param trajectory
   */
  comau_tcp_interface::utils::trajectoryf_t
  parseJointTrajectoryGoal(const comau_msgs::ExecuteJointTrajectoryGoalConstPtr &goal);
  /**
   * @brief Conversion function ROS geometry_msgs/PoseStamped[]
   * message to custom utils::trajectoryf_t
   *
   * @param cartesian_trajectory
   */
  comau_tcp_interface::utils::trajectoryf_t
  parseCartesianTrajectoryGoal(const comau_msgs::ExecuteJointTrajectoryGoalConstPtr &goal);
  /**
   * @brief A callback function that handles the incoming goal
   *
   * @param goal see comau_msgs::ExecuteJointTrajectoryGoal
   */
  void executeCallback(const comau_msgs::ExecuteJointTrajectoryGoalConstPtr &goal);
  /**
   * @brief A function that check whether the goal command is complete or out of limits
   *
   * @return true if goal is valid
   * @return false if invalid
   */
  bool validateJointLimits(utils::vector6f_t joint_values_array);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  std::string action_name_;

  bool action_active_;
  bool parallel_joint_fix_;
  bool error_;
  bool valid_goal_;

  comau_msgs::ExecuteJointTrajectoryFeedback feedback_;
  comau_msgs::ExecuteJointTrajectoryResult result_;
  comau_msgs::ComauRobotStatus robot_status_;

  boost::shared_ptr<ExecuteJointTrajectoryActionServer> as_ptr_;
  boost::shared_ptr<comau_tcp_interface::MotionClient> motion_client_;
  boost::shared_ptr<comau_msgs::ComauRobotStatus const> status_ptr_;

  boost::shared_ptr<urdf::Model> urdf_model_ptr_;
  size_t total_number_of_joints_; // NOTE not needed to be class variable
  std::vector<double> joint_position_lower_limits_;
  std::vector<double> joint_position_upper_limits_;

  ros::Subscriber robot_status_sub_;
};

} // namespace handler
} // namespace comau_driver
