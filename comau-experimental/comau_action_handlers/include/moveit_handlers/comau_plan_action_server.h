/**
 * @file move_topose_handler_server.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 15-05-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

#include <ros/ros.h>
#include <sys/ioctl.h> // For ioctl, TIOCGWINSZ
#include <unistd.h>    // For STDOUT_FILENO
#include <utility>

#include "naming_constants.h"

// action libs
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <comau_msgs/ActionResultStatusConstants.h>
#include <comau_msgs/ExecuteJointTrajectoryAction.h>
#include <comau_msgs/MoveToPoseMoveItAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// moveit libs
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>

#include <math.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// For Visualization
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Read/Write bag files
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>


// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

typedef actionlib::SimpleActionServer<comau_msgs::MoveToPoseMoveItAction>
    MoveToPoseMoveItActionServer;

/**
 * @brief This handler accepts as goal a cartesian point relative to any tf
 * frame and will plan and execute a trajectory
 *
 */
class MoveToPoseHandlerServer {
public:
  /**
   * @brief Construct a new MoveToPoseHandlerServer object
   *
   * @param nh
   * @param nh_local
   * @param name
   */
  MoveToPoseHandlerServer(ros::NodeHandle &nh, ros::NodeHandle &nh_local,
                          std::string name);
  /**
   * @brief Destroy the Move To Pose Handler Server object
   *
   */
  ~MoveToPoseHandlerServer(void);
  /**
   * @brief Initialization function
   *
   * @return true
   * @return false if something went wrong at the initialization
   */
  bool initialize();

  // For Sending Trajectory Goal
  comau_msgs::ExecuteJointTrajectoryGoal traj_goal;
  typedef actionlib::SimpleActionClient<comau_msgs::ExecuteJointTrajectoryAction>
      arm_control_client;
  typedef boost::shared_ptr<arm_control_client> arm_control_client_Ptr;
  arm_control_client_Ptr ArmClient;

  /**
   * @brief Creates a client to ExecuteJointTrajectoryhanlder server. It sends 
   * the computed trajectory in order to be executed by the real robot.
   *
   */
  void createArmClient(arm_control_client_Ptr &actionClient);

private:
  /**
   * @brief A callback function that handles the incoming goal
   *
   * @param goal see comau_msgs::MoveToPoseMoveItGoal
   */
  void executeCallback(const comau_msgs::MoveToPoseMoveItGoalConstPtr &goal);
  /**
   * @brief thread function for MoveIt planning. It plans a point-to-point
   * from the current robot state to goal state
   */
  void moveit_planning();
  /**
   * @brief thread function for MoveIt execution. It execute the trajectory only iin simulation
   * by using a fake controller
   */
  void moveit_execution();
  
  /**
   * @brief thread function for Comau controller execution
   *
   */
  void controller_execution();
  
  /**
   * @brief A function that transforms the pose goal relative to another frame
   *
   */
  geometry_msgs::PoseStamped changePoseFrame(const std::string &target_frame,
                                             const geometry_msgs::PoseStamped &goal_pose);
  
  /**
   * @brief prints message when a goal has ended
   *
   */
  void printLine();

  /**
   * @brief check if target pose is reachable
   * 
   */
  bool validityCheck(geometry_msgs::PoseStamped target);

  /**
   * @brief check if current robot state is equal to planned state
   * for safe execution
   * 
   */
  bool validateExecution();

  /**
   * @brief show target pose
   *
   */
  void showTargetPose(geometry_msgs::PoseStamped target);

  /**
   * @brief show planned path
   *
   */
  void showPlannedPath(moveit::planning_interface::MoveGroupInterface::Plan plan_);

  /**
   * @brief save planned path to bag file
   *
   */
  void savePlan(moveit::planning_interface::MoveGroupInterface::Plan plan_, std::string file_path, std::string plan_id);

  /**
   * @brief load planned path from bag file
   *
   */
  moveit::planning_interface::MoveGroupInterface::Plan loadPlan(std::string file_path, std::string plan_id);

  

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  std::string action_name_;
  comau_msgs::MoveToPoseMoveItFeedback feedback_;
  comau_msgs::MoveToPoseMoveItResult result_;

  std::string PLANNING_GROUP_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
  double eps_ = 1e-03;
  
  std::string file_path = "/home/antonio/Lib/RosService/Battery";

  bool moveit_running = true, planning_succeeded_, execution_succeeded_,
       plannig_thread_done_, execution_thread_done_;

  ros::WallTime _start;
  boost::shared_ptr<MoveToPoseMoveItActionServer> as_ptr_;
  // Visualization
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr;
  const robot_state::JointModelGroup *joint_model_group;

  // Test path length
  double l_ = 0;
  Eigen::Isometry3d fk(std::vector<double> joint_values);
  double pathLength(moveit_msgs::MotionPlanResponse plan);
};
