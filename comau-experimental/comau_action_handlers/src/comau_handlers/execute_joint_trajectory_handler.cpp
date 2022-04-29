/**
 * @file execute_joint_trajectory_handler.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 15-05-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "comau_handlers/execute_joint_trajectory_handler.h"

#include "comau_hardware_interface/comau_hardware_interface.h"

using namespace comau_tcp_interface;

namespace comau_driver {
namespace handler {

ExecuteJointTrajectoryHandler::ExecuteJointTrajectoryHandler(ros::NodeHandle &nh, ros::NodeHandle &nh_local,
                                                             std::string name)
    : nh_(nh), nh_local_(nh_local), action_name_(std::move(name)) {}
    
ExecuteJointTrajectoryHandler::~ExecuteJointTrajectoryHandler() {
  motion_client_.reset();
  if (action_active_) {
    result_.action_result.success = false;
    result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
    result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;
    as_ptr_->setPreempted(result_);
  }
  as_ptr_.reset();
  urdf_model_ptr_.reset();
}

bool ExecuteJointTrajectoryHandler::initialize() {
  const std::string server_name = "execute_joint_trajectory_handler";

  ComauTcpInterfaceParameters params;
  params.log_tag = "motion_tcp_client";
  // Read parameters through ros parameter server
  if (!nh_.getParam("robot_ip", params.server_ip_address)) {
    ROS_ERROR_STREAM("[" << action_name_ << "] Required parameter " << nh_.resolveName("robot_ip") << " not given.");
    return false;
  }
  ROS_DEBUG_STREAM("[" << action_name_ << "] robot_ip : " << params.server_ip_address);

  if (!nh_.getParam("motion_server_port", params.server_port)) {
    ROS_ERROR_STREAM("[" << action_name_ << "] Required parameter " << nh_.resolveName("state_server_port")
                         << " not given.");
    return false;
  }
  ROS_DEBUG_STREAM("[" << action_name_ << "] motion_server_port : " << params.server_port);

  parallel_joint_fix_ = nh_.param("parallel_joint_fix", false);

  if (!loadJointLimits(nh_, "robot_description")) {
    ROS_ERROR_STREAM("[" << action_name_ << "] Error at loadJointLimits function");
    return false;
  }

  robot_status_sub_ = nh_.subscribe("status", 1000, &ExecuteJointTrajectoryHandler::statusCallback, this);
  ROS_INFO_STREAM("[" << action_name_
                      << "] Subsrcibing at robot status topic with name : " << robot_status_sub_.getTopic());
  status_ptr_ = ros::topic::waitForMessage<comau_msgs::ComauRobotStatus>("status", ros::Duration(1));
  while (!status_ptr_) {
    ROS_WARN_STREAM("[" << action_name_
                        << "] Robot status topic not published "
                           "yet. Trying again ...");
    status_ptr_ = ros::topic::waitForMessage<comau_msgs::ComauRobotStatus>("status", ros::Duration(1));
  }

  ROS_INFO_STREAM("[" << action_name_ << "] Initializing Motion client ... ");
  motion_client_.reset(new MotionClient());
  if (!motion_client_->initialize(params)) {
    ROS_ERROR_STREAM("[" << action_name_ << "] Motion client cannot be initialized");
    return false;
  }

  ROS_INFO_STREAM("[" << action_name_ << "] Starting up the ExecuteJointTrajectoryActionServer ...  ");
  try {
    as_ptr_.reset(new ExecuteJointTrajectoryActionServer(
        nh_, action_name_, boost::bind(&ExecuteJointTrajectoryHandler::executeCallback, this, _1), false));
    as_ptr_->start();
  } catch (...) {
    ROS_ERROR_STREAM("[" << action_name_ << "]"
                         << "ExecuteJointTrajectoryActionServer cannot not start.");
    return false;
  }

  ROS_INFO_STREAM("[" << action_name_ << "] Ready to receive goals!  ");
  return true;
}

bool ExecuteJointTrajectoryHandler::loadJointLimits(const ros::NodeHandle &nh, std::string param_name) {
  std::string urdf_string;
  urdf_model_ptr_.reset(new urdf::Model());

  // Search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok()) {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name)) {
      ROS_INFO_STREAM("[" << action_name_
                          << "] Found URDF model on the ROS "
                             "param server at location: "
                          << nh.getNamespace() << search_param_name);
      nh.getParam(search_param_name, urdf_string);
    } else {
      ROS_INFO_STREAM("[" << action_name_
                          << "] Waiting for model URDF on the ROS "
                             "param server at location: "
                          << nh.getNamespace() << param_name);
    }
    usleep(100000);
  }

  if (!urdf_model_ptr_->initString(urdf_string)) {
    ROS_ERROR_STREAM("[" << action_name_ << "] Unable to load URDF model with param string : " << urdf_string);
    return false;
  } else {
    ROS_DEBUG_STREAM("[" << action_name_ << "] Received URDF from param server");
  }

  // Get limits from URDF
  if (!urdf_model_ptr_) {
    ROS_ERROR_STREAM("[" << action_name_ << "] No URDF model loaded, unable to get joint limits");
    return false;
  }
  total_number_of_joints_ = urdf_model_ptr_->joints_.size();
  joint_position_lower_limits_.resize(0, 0.0);
  joint_position_upper_limits_.resize(0, 0.0);

  // size_t joint_id = 0;
  for (auto joint : urdf_model_ptr_->joints_) {
    // joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
    // joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();

    // Limits datastructures
    joint_limits_interface::JointLimits joint_limits; // Position

    if (!joint.second) {
      ROS_ERROR_STREAM("[" << action_name_ << "] URDF joint not found " << joint.first);
      return false;
    }

    if (joint_limits_interface::getJointLimits(joint.second, joint_limits)) {
      ROS_INFO_STREAM("[" << action_name_ << "] " << joint.first << " has URDF position limits ["
                          << joint_limits.min_position << ", " << joint_limits.max_position << "]");
      joint_limits.min_position += std::numeric_limits<double>::epsilon();
      joint_limits.max_position -= std::numeric_limits<double>::epsilon();

      // joint_position_lower_limits_[joint_id] = joint_limits.min_position;
      // joint_position_upper_limits_[joint_id] = joint_limits.max_position;
      joint_position_lower_limits_.push_back(joint_limits.min_position);
      joint_position_upper_limits_.push_back(joint_limits.max_position);
    } else {
      ROS_WARN_STREAM("[" << action_name_ << "] " << joint.first
                          << " has invalid joint limits. Maybe is not revolute joint.");
    }

    // joint_id++;
  }
  ROS_INFO_STREAM("hellloooooo!!!" << joint_position_lower_limits_.size());

  return true;
}

void ExecuteJointTrajectoryHandler::statusCallback(const comau_msgs::ComauRobotStatusConstPtr &msg) {
  robot_status_ = *msg;
}

utils::trajectoryf_t
ExecuteJointTrajectoryHandler::parseJointTrajectoryGoal(const comau_msgs::ExecuteJointTrajectoryGoalConstPtr &goal) {
  utils::trajectoryf_t joint_traj;

  for (trajectory_msgs::JointTrajectoryPoint joints_goal : goal->trajectory) {
    utils::vector6f_t joint_values_array;
    for (size_t i = 0; i < joints_goal.positions.size(); i++) {
      // Convert RADS to DEGREES
      joint_values_array.at(i) = static_cast<float>(joints_goal.positions.at(i) * 180 / M_PI);
      if (parallel_joint_fix_) {
        joint_values_array.at(2) -= joint_values_array.at(1);
      }
    }

    // Check for joint limits
    if (!validateJointLimits(joint_values_array))
      return joint_traj;

    joint_traj.push_back(joint_values_array);
  }
  return joint_traj;
}

bool ExecuteJointTrajectoryHandler::validateJointLimits(utils::vector6f_t joint_values_array) {

  for (size_t joint_id = 0; joint_id < joint_values_array.size(); joint_id++) {
    valid_goal_ = true;
    if (joint_values_array[joint_id] * (M_PI / 180) > joint_position_upper_limits_[joint_id] ||
        joint_values_array[joint_id] * (M_PI / 180) < joint_position_lower_limits_[joint_id]) {
      valid_goal_ = false;
      ROS_WARN_STREAM("[" << action_name_ << "] Trajectory contains an out of limits goal at joint " << joint_id + 1
                          << ". Please check robot limits.");
      return valid_goal_;
    }
  }
  return valid_goal_;
}

void ExecuteJointTrajectoryHandler::executeCallback(const comau_msgs::ExecuteJointTrajectoryGoalConstPtr &goal) {
  double start_time = ros::Time::now().toNSec() / 1e-6; // to convert nanoseconds to milliseconds
  action_active_ = false;
  utils::trajectoryf_t goal_joint_trajectory = parseJointTrajectoryGoal(goal);

  if (robot_status_.status == comau_msgs::ComauRobotStatus::READY) {
    if (valid_goal_) {
      motion_client_->sendJointTrajectoryMessage(goal_joint_trajectory);
      action_active_ = true;
      ROS_INFO("[%s]: Received trajectory sended for execution", action_name_.c_str());
    } else {
      result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      result_.action_result.success = false;
      as_ptr_->setAborted(result_);
      return;
    }
  } else {
    ROS_WARN("[%s]: Robot is not in READY status. Try to send a "
             "goal later.",
             action_name_.c_str());

    result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
    result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
    result_.action_result.success = false;

    as_ptr_->setAborted(result_);
    return;
  }

  while (action_active_) {
    if (as_ptr_->isPreemptRequested() || !ros::ok()) { // CANCELLED
      ROS_INFO("[%s]: Trajectory execution Preempted", action_name_.c_str());

      motion_client_->sendCancelTrajectoryMessage();

      result_.action_result.success = false;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;

      as_ptr_->setPreempted(result_);

      return;
    } else if (robot_status_.status == comau_msgs::ComauRobotStatus::SUCCEEDED) { // SUCCEEDED
      ROS_INFO("[%s]: Trajectory execution Succeeded", action_name_.c_str());

      result_.action_result.status = comau_msgs::ActionResultStatusConstants::SUCCESS;
      result_.action_result.success = true;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;

      as_ptr_->setSucceeded(result_);

      return;
    } else if (robot_status_.status == comau_msgs::ComauRobotStatus::ERROR) { // ERROR
      ROS_ERROR("[%s]: Unexpected error, closing action server", action_name_.c_str());

      result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      result_.action_result.success = false;

      as_ptr_->setAborted(result_);

      return;
    } else if (robot_status_.status == comau_msgs::ComauRobotStatus::TERMINATE) { // TERMINATE
      ROS_INFO("[%s]: Action terminated, canceling Trajectory execution", action_name_.c_str());

      result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      result_.action_result.success = false;

      as_ptr_->setAborted(result_);

      return;
    }
    // MOVING
    ROS_DEBUG("[%s]: Trajectory execution is active", action_name_.c_str());
    feedback_.action_feedback.millis_passed = uint((ros::Time::now().toNSec() / 1e-6) - start_time);

    as_ptr_->publishFeedback(feedback_);
    ros::Duration(0.001).sleep();
  }
}

} // namespace handler
} // namespace comau_driver
