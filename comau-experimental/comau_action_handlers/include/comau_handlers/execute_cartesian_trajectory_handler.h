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

#include <limits>
#include <ros/ros.h>
#include <utility>

#include <actionlib/server/simple_action_server.h>

#include "comau_tcp_interface/comau_motion_client.h"
#include "comau_tcp_interface/comau_tcp_interface.h"
#include "comau_tcp_interface/utils/message_package.h"

// msgs
#include "comau_msgs/ActionResultStatusConstants.h"
#include "comau_msgs/ComauRobotStatus.h"
#include "comau_msgs/ExecuteCartesianTrajectoryAction.h"
#include <comau_msgs/CartesianPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
// tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "naming_constants.h"

using namespace comau_tcp_interface;

namespace comau_driver {
namespace handler {
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
   * @param name
   */
  ExecuteCartesianTrajectoryHandler(ros::NodeHandle &nh, ros::NodeHandle &nh_local, std::string name);

  /**
   * @brief Destroy the Move Cartesian Handler object
   *
   */
  ~ExecuteCartesianTrajectoryHandler();
  /**
   * @brief Initialization function
   *
   * @return true
   * @return false if something went wrong at the initialization
   */
  bool initialize();

private:
  /**
   * @brief Robot status subscribers's callback function.
   *
   * @param msg
   */
  void statusCallback(const comau_msgs::ComauRobotStatusConstPtr &msg);
  /**
   * @brief Conversion function ROS geometry_msgs/PoseStamped[]
   * message to custom utils::trajectoryf_t
   *
   * @param cartesian_trajectory
   */
  comau_tcp_interface::utils::trajectoryf_t
  parseCartesianTrajectoryGoal(const comau_msgs::ExecuteCartesianTrajectoryGoalConstPtr &goal);
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
  comau_msgs::ComauRobotStatus robot_status_;

  boost::shared_ptr<ExecuteCartesianTrajectoryActionServer> as_ptr_;
  boost::shared_ptr<comau_tcp_interface::MotionClient> motion_client_;
  boost::shared_ptr<comau_msgs::ComauRobotStatus const> status_ptr_;

  ros::Subscriber robot_status_sub_;
};

} // namespace handler
} // namespace comau_driver
