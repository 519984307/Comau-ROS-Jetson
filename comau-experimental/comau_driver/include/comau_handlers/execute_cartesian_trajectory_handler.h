/**
 * @file execute_cartesian_trajectory_handler.h
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
// msgs
#include "comau_msgs/ActionResultStatusConstants.h"
#include "comau_msgs/ExecuteCartesianTrajectoryAction.h"
#include <comau_msgs/CartesianPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
// tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "comau_driver/comau_driver.h"

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_action_handlers {

typedef actionlib::SimpleActionServer<comau_msgs::ExecuteCartesianTrajectoryAction>
    ExecuteCartesianTrajectoryActionServer;

/**
 * @brief This handler accepts as goal a cartesian space trajectory and send it to
 * Robot with MotionClient
 *
 */
class ExecuteCartesianTrajectoryHandler {
public:
  /**
   * @brief Construct a new MoveCartesianServer object
   *
   * @param nh
   * @param nh_local
   * @param robot_ptr
   * @param name
   */
  ExecuteCartesianTrajectoryHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_local, const std::string &name,
                                    const boost::shared_ptr<comau_driver::ComauRobot> &robot_ptr);
  /**
   * @brief Destroy the Move Cartesian Handler object
   *
   */
  ~ExecuteCartesianTrajectoryHandler();
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
   * @brief Conversion function ROS geometry_msgs/PoseStamped[]
   * message to custom utils::trajectoryf_t
   *
   * @param cartesian_trajectory
   */
  trajectoryf_t parseCartesianTrajectoryGoal(const comau_msgs::ExecuteCartesianTrajectoryGoalConstPtr &goal);
  /**
   * @brief A callback function that handles the incoming goal
   *
   * @param goal see comau_msgs::ExecuteCartesianTrajectoryGoal
   */
  void executeCallback(const comau_msgs::ExecuteCartesianTrajectoryGoalConstPtr &goal);
  /**
   * @brief A function that transforms the pose goal relative to another frame
   *
   */
  geometry_msgs::PoseStamped changePoseFrame(const std::string &target_frame,
                                             const geometry_msgs::PoseStamped &goal_pose);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  std::string action_name_;

  bool action_active_;
  bool error_;

  comau_msgs::ExecuteCartesianTrajectoryFeedback feedback_;
  comau_msgs::ExecuteCartesianTrajectoryResult result_;
  char robot_status_ = 'R'; // only initialized for dummy

  boost::shared_ptr<ExecuteCartesianTrajectoryActionServer> as_ptr_;
  trajectoryf_t goal_cartesian_trajectory_;
  bool use_state_server_, use_motion_server_;
  bool allow_async_ = false;
  // Robot Driver Pointer
  boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_;
};

} // namespace comau_action_handlers
