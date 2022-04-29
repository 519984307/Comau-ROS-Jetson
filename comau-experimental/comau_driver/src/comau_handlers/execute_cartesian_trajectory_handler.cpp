/**
 * @file execute_cartesian_trajectory_handler.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 15-05-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "comau_handlers/execute_cartesian_trajectory_handler.h"

namespace comau_action_handlers {

ExecuteCartesianTrajectoryHandler::ExecuteCartesianTrajectoryHandler(
    const ros::NodeHandle &nh, const ros::NodeHandle &nh_local, const std::string &name,
    const boost::shared_ptr<comau_driver::ComauRobot> &robot_ptr)
    : nh_(nh), nh_local_(nh_local), action_name_(std::move(name)), robot_ptr_(robot_ptr) {}

ExecuteCartesianTrajectoryHandler::~ExecuteCartesianTrajectoryHandler() {
  as_ptr_.reset();
  if (action_active_) {
    result_.action_result.success = false;
    result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
    result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;
    as_ptr_->setPreempted(result_);
  }
}

bool ExecuteCartesianTrajectoryHandler::initialize(bool use_state_server, bool use_motion_server) {
  use_state_server_ = use_state_server;
  use_motion_server_ = use_motion_server;
  ROS_INFO_STREAM("[" << action_name_ << "] Starting up the ExecuteCartesianTrajectoryActionServer ...  ");
  try {
    as_ptr_.reset(new ExecuteCartesianTrajectoryActionServer(
        nh_, action_name_, boost::bind(&ExecuteCartesianTrajectoryHandler::executeCallback, this, _1), false));
    as_ptr_->start();
  } catch (...) {
    ROS_ERROR_STREAM("[" << action_name_ << "]"
                         << "ExecuteCartesianTrajectoryActionServer cannot not start.");
    return false;
  }

  ROS_INFO_STREAM("[" << action_name_ << "] Ready to receive goals!  ");
  return true;
}

geometry_msgs::PoseStamped
ExecuteCartesianTrajectoryHandler::changePoseFrame(const std::string &target_frame,
                                                   const geometry_msgs::PoseStamped &goal_pose) {
  tf2_ros::Buffer br;
  br.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf2_listener(br);
  geometry_msgs::TransformStamped transform;
  geometry_msgs::PoseStamped transformed_pose;

  try {
    transform = br.lookupTransform(target_frame, goal_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(goal_pose, transformed_pose, transform);
    ROS_INFO_STREAM("Change transform pose to..   " << transformed_pose);
    return transformed_pose;
  } catch (tf2::LookupException &e) {
    ROS_ERROR("[%s] %s", action_name_.c_str(), e.what());
    transformed_pose.header.frame_id = "tool_controller";
    return transformed_pose;
  }
}

trajectoryf_t ExecuteCartesianTrajectoryHandler::parseCartesianTrajectoryGoal(
    const comau_msgs::ExecuteCartesianTrajectoryGoalConstPtr &goal) {
  trajectoryf_t pose_traj;

  for (comau_msgs::CartesianPoseStamped cart_pose : goal->trajectory) {
    if (cart_pose.header.frame_id != "") {
      // construct tf pose from cart pose
      geometry_msgs::PoseStamped pose;
      tf2::Quaternion q;
      q.setRPY(cart_pose.roll, cart_pose.pitch, cart_pose.yaw);
      q.normalize();
      pose.header = cart_pose.header;
      pose.pose.position.x = cart_pose.x;
      pose.pose.position.y = cart_pose.y;
      pose.pose.position.z = cart_pose.z;
      pose.pose.orientation.x = q[0];
      pose.pose.orientation.y = q[1];
      pose.pose.orientation.z = q[2];
      pose.pose.orientation.w = q[3];
      // Transform the pose relative on existing TF frame (if frame not equal to pass)
      geometry_msgs::PoseStamped transformed_pose;
      transformed_pose = changePoseFrame("base_link", pose);
      vector6f_t pose_values_array;
      // first 3 points correspond to position - PDL wants millimiters
      pose_values_array.at(0) = static_cast<float>(transformed_pose.pose.position.x * 1000.);
      pose_values_array.at(1) = static_cast<float>(transformed_pose.pose.position.y * 1000.);
      pose_values_array.at(2) = static_cast<float>(transformed_pose.pose.position.z * 1000.);
      // Revert back to euler
      // POS_SET_RPY IN PDL
      tf2::Quaternion q_transformed;
      q_transformed[0] = transformed_pose.pose.orientation.x;
      q_transformed[1] = transformed_pose.pose.orientation.y;
      q_transformed[2] = transformed_pose.pose.orientation.z;
      q_transformed[3] = transformed_pose.pose.orientation.w;
      q_transformed.normalize();
      double roll, pitch, yaw;
      tf2::Matrix3x3 m(q_transformed);
      m.getRPY(roll, pitch, yaw);
      pose_values_array.at(3) = static_cast<float>(roll * 180. / M_PI);
      pose_values_array.at(4) = static_cast<float>(pitch * 180. / M_PI);
      pose_values_array.at(5) = static_cast<float>(yaw * 180. / M_PI);
      pose_traj.push_back(pose_values_array);
    } else {
      vector6f_t pose_values_array;
      // first 3 points correspond to position - PDL wants millimiters
      pose_values_array.at(0) = static_cast<float>(cart_pose.x * 1000.);
      pose_values_array.at(1) = static_cast<float>(cart_pose.y * 1000.);
      pose_values_array.at(2) = static_cast<float>(cart_pose.z * 1000.);
      // euler angles
      pose_values_array.at(3) = static_cast<float>(cart_pose.roll * 180. / M_PI);
      pose_values_array.at(4) = static_cast<float>(cart_pose.pitch * 180. / M_PI);
      pose_values_array.at(5) = static_cast<float>(cart_pose.yaw * 180. / M_PI);
      pose_traj.push_back(pose_values_array);
    }
  }
  return pose_traj;
}

void ExecuteCartesianTrajectoryHandler::executeCallback(
    const comau_msgs::ExecuteCartesianTrajectoryGoalConstPtr &goal) {
  double start_time = ros::Time::now().toNSec() / 1e-6; // to convert nanoseconds to milliseconds
  action_active_ = false;
  if (robot_status_ == RobotStatus::READY && allow_async_) {
    ROS_INFO("[%s]: Parsing Cartesian trajectory", action_name_.c_str());
    goal_cartesian_trajectory_ = parseCartesianTrajectoryGoal(goal);
    if (use_motion_server_)
      robot_ptr_->writeTrajectoryCommand(goal_cartesian_trajectory_,
                                         comau_driver::ControlMode::MODE_CARTESIAN_TRAJECTORY);
    action_active_ = true;
    ROS_INFO("[%s]: Received trajectory sended for execution", action_name_.c_str());
  } else {
    ROS_WARN("[%s]: Robot is not in READY status. We are stopping - resetting", action_name_.c_str());
    if (use_motion_server_ && allow_async_)
      robot_ptr_->resetState();
    result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
    result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
    result_.action_result.success = false;

    as_ptr_->setAborted(result_);
    return;
  }

  while (action_active_) {

    if (as_ptr_->isPreemptRequested() || !ros::ok()) { // CANCELLED
      ROS_INFO("[%s]: Trajectory execution Preempted", action_name_.c_str());
      if (use_state_server_ || use_motion_server_)
        result_.action_result.success = false;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;
      if (use_motion_server_)
        robot_ptr_->resetState();
      as_ptr_->setPreempted(result_);

      return;
    } else if (robot_status_ == RobotStatus::SUCCEEDED) { // SUCCEEDED
      ROS_INFO("[%s]: Trajectory execution Succeeded", action_name_.c_str());

      result_.action_result.status = comau_msgs::ActionResultStatusConstants::SUCCESS;
      result_.action_result.success = true;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      if (use_motion_server_)
        robot_ptr_->resetState();
      as_ptr_->setSucceeded(result_);

      return;
    } else if (robot_status_ == RobotStatus::ERROR) { // ERROR
      ROS_ERROR("[%s]: Unexpected error, closing action server", action_name_.c_str());

      result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      result_.action_result.success = false;
      if (use_motion_server_)
        robot_ptr_->resetState();
      as_ptr_->setAborted(result_);

      return;
    } else if (robot_status_ == RobotStatus::TERMINATE) { // TERMINATE
      ROS_INFO("[%s]: Action terminated, canceling Trajectory execution", action_name_.c_str());

      result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      result_.action_result.success = false;
      if (use_motion_server_)
        robot_ptr_->terminateMotion();
      as_ptr_->setAborted(result_);

      return;
    } else if (robot_status_ == RobotStatus::MOVING) { // MOVING

      ROS_DEBUG("[%s]: Trajectory execution is active", action_name_.c_str());
      feedback_.action_feedback.millis_passed = uint((ros::Time::now().toNSec() / 1e-6) - start_time);

      as_ptr_->publishFeedback(feedback_);
    }

    ros::Duration(0.001).sleep();
  }
}

void ExecuteCartesianTrajectoryHandler::set_status(char &status) {
  robot_status_ = status;
}
void ExecuteCartesianTrajectoryHandler::set_allow_async(const bool &allow_async) {
  allow_async_ = allow_async;
}

} // namespace comau_action_handlers