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

#include "comau_hardware_interface/comau_hardware_interface.h"

using namespace comau_tcp_interface;

namespace comau_driver {
namespace handler {

ExecuteCartesianTrajectoryHandler::ExecuteCartesianTrajectoryHandler(ros::NodeHandle &nh, ros::NodeHandle &nh_local,
                                                                     std::string name)
    : nh_(nh), nh_local_(nh_local), action_name_(std::move(name)) {}
ExecuteCartesianTrajectoryHandler::~ExecuteCartesianTrajectoryHandler() {
  motion_client_.reset();
  if (action_active_) {
    result_.action_result.success = false;
    result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
    result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;
    as_ptr_->setPreempted(result_);
  }
  as_ptr_.reset();
}

bool ExecuteCartesianTrajectoryHandler::initialize() {
  const std::string server_name = "execute_cartesian_trajectory_handler";

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

  robot_status_sub_ = nh_.subscribe("status", 1000, &ExecuteCartesianTrajectoryHandler::statusCallback, this);
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

void ExecuteCartesianTrajectoryHandler::statusCallback(const comau_msgs::ComauRobotStatusConstPtr &msg) {
  robot_status_ = *msg;
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

utils::trajectoryf_t ExecuteCartesianTrajectoryHandler::parseCartesianTrajectoryGoal(
    const comau_msgs::ExecuteCartesianTrajectoryGoalConstPtr &goal) {
  utils::trajectoryf_t pose_traj;

  for (comau_msgs::CartesianPose cart_pose : goal->trajectory) {
    if (cart_pose.frame != "pass") {
      // construct tf pose from cart pose
      geometry_msgs::PoseStamped pose;
      tf2::Quaternion q;
      q.setRPY(cart_pose.roll, cart_pose.pitch, cart_pose.yaw);
      pose.header.frame_id = cart_pose.frame;
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
      utils::vector6f_t pose_values_array;
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
      double roll, pitch, yaw;
      tf2::Matrix3x3 m(q_transformed);
      m.getRPY(roll, pitch, yaw);
      pose_values_array.at(3) = static_cast<float>(roll * 180. / M_PI);
      pose_values_array.at(4) = static_cast<float>(pitch * 180. / M_PI);
      pose_values_array.at(5) = static_cast<float>(yaw * 180. / M_PI);
      pose_traj.push_back(pose_values_array);
    } else {
      utils::vector6f_t pose_values_array;
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
  ROS_INFO("[%s]: Parsing Cartesian trajectory", action_name_.c_str());
  utils::trajectoryf_t goal_cartesian_trajectory = parseCartesianTrajectoryGoal(goal);
  if (robot_status_.status == comau_msgs::ComauRobotStatus::READY) {
    motion_client_->sendCartTrajectoryMessage(goal_cartesian_trajectory);
    action_active_ = true;
    ROS_INFO("[%s]: Received trajectory sended for execution", action_name_.c_str());
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
