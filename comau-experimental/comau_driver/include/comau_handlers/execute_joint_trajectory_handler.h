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

#include <boost/scoped_ptr.hpp>
#include <limits>
#include <memory>
#include <ros/ros.h>
#include <utility>

#include <actionlib/server/simple_action_server.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

// msgs
#include <comau_msgs/ActionResultStatusConstants.h>
#include <comau_msgs/ExecuteJointTrajectoryAction.h>
#include <std_msgs/Bool.h>

#include <comau_driver/comau_driver.h>

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_action_handlers {

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
   * @param robot_ptr
   * @param name
   */
  ExecuteJointTrajectoryHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_local, const std::string &name,
                                const boost::shared_ptr<comau_driver::ComauRobot> &robot_ptr);

  /**
   * @brief Destroy the Move Joints Handler object
   *
   */
  ~ExecuteJointTrajectoryHandler();
  /**
   * @brief Initialization function
   *
   * @param use_state_server
   * @param use_motion_server
   *
   * @return true
   * @return false if something went wrong at the initialization
   */
  bool initialize(bool use_state_server, bool use_motion_server);
  void set_status(char &status);
  void set_allow_async(const bool &allow_async);

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
   * @brief Conversion function ROS trajectory_msgs/JointTrajectoryPoint[]
   * message to custom utils::trajectoryf_t
   *
   * @param trajectory
   */
  trajectoryf_t parseJointTrajectoryGoal(const comau_msgs::ExecuteJointTrajectoryGoalConstPtr &goal);
  /**
   * @brief Conversion function ROS geometry_msgs/PoseStamped[]
   * message to custom utils::trajectoryf_t
   *
   * @param cartesian_trajectory
   */
  trajectoryf_t parseCartesianTrajectoryGoal(const comau_msgs::ExecuteJointTrajectoryGoalConstPtr &goal);
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
  bool validateJointLimits(vector6f_t joint_values_array);
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  std::string action_name_;

  bool action_active_;
  bool error_;
  bool valid_goal_;
  char robot_status_ = 'R'; // init only for dummy
  trajectoryf_t goal_joint_trajectory_;

  comau_msgs::ExecuteJointTrajectoryFeedback feedback_;
  comau_msgs::ExecuteJointTrajectoryResult result_;

  boost::shared_ptr<ExecuteJointTrajectoryActionServer> as_ptr_;

  boost::shared_ptr<urdf::Model> urdf_model_ptr_;
  size_t total_number_of_joints_; // NOTE not needed to be class variable
  std::vector<double> joint_position_lower_limits_;
  std::vector<double> joint_position_upper_limits_;
  bool use_state_server_, use_motion_server_;
  bool allow_async_ = false;
  // Robot Driver Pointer
  boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_;
};

} // namespace comau_action_handlers
