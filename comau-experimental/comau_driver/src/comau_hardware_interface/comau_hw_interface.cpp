#include <sstream>

#include <comau_hardware_interface/comau_hw_helpers.h>
#include <comau_hardware_interface/comau_hw_interface.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace comau_hardware_interface {

ComauHardwareInterface::ComauHardwareInterface()
    : position_controller_running_(false), sensor_tracking_controller_running_(false), controllers_initialized_(false) {
}

bool ComauHardwareInterface::init(ros::NodeHandle &nh, ros::NodeHandle &nh_local) {

  // Get Comau Hardware Interface parameters
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_local, "use_state_server", use_state_server_);
  error += !rosparam_shortcuts::get(name_, nh_local, "use_motion_server", use_motion_server_);
  error += !rosparam_shortcuts::get(name_, nh_local, "joints", joint_names_);
  rosparam_shortcuts::shutdownIfError(name_, error);
  // Resize vectors
  num_joints_ = joint_names_.size();
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);
  sensor_tracking_command_.resize(num_joints_);
  ee_position_.resize(num_joints_);

  // Initialize Robot driver

  robot_ptr_.reset(new comau_driver::ComauRobot(nh_, nh_local_));
  if (!robot_ptr_->initialize(use_state_server_, use_motion_server_)) {
    ROS_ERROR_STREAM("[comau_hw_interface] Failed to initialize robot driver");
    return false;
  }

  // comau action handlers : execute_joint_trajectory_handler
  try {
    execute_joints_handler_ptr.reset(new comau_action_handlers::ExecuteJointTrajectoryHandler(
        nh_, nh_local_, execute_joint_server_name_, robot_ptr_));
    if (!execute_joints_handler_ptr->initialize(use_state_server_, use_motion_server_)) {
      ROS_ERROR_STREAM("[comau_robot] Execute Joint Trajectory Handler could not initialized ");
      return false;
    }
  } catch (...) {
    ROS_ERROR_STREAM("[comau_robot] Execute Joint Trajectory Handler error");
    return false;
  }
  // comau action handlers : execute_cartesian_trajectory_handler
  try {
    execute_cartesian_handler_ptr.reset(new comau_action_handlers::ExecuteCartesianTrajectoryHandler(
        nh_, nh_local_, execute_cartesian_server_name_, robot_ptr_));
    if (!execute_cartesian_handler_ptr->initialize(use_state_server_, use_motion_server_)) {
      ROS_ERROR_STREAM("[comau_robot] Execute Cartesian Trajectory Handler could not initialized ");
      return false;
    }
  } catch (...) {
    ROS_ERROR_STREAM("[comau_robot] Execute Cartesian Trajectory Handler error");
    return false;
  }

  // Create ros_control interfaces
  for (size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM("[comau_hw_interface] Registering handles for joint " << joint_names_[i]);
    try {
      // Create joint state interface
      js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                        &joint_velocity_[i], &joint_effort_[i]));
      // Create position joint interface
      pj_interface_.registerHandle(
          hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
      // Create effort joint interface
      ej_interface_.registerHandle(
          hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &sensor_tracking_command_[i]));
      // Create Sensor Tracking interface
      sensor_tracking_interface_.registerHandle(
          hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &sensor_tracking_command_[i]));
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("[comau_hw_interface] " << e.what());
      return false;
    }
  }
  // robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
  //    "industrial_robot_status_handle", robot_status_resource_));

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&sensor_tracking_interface_);
  // registerInterface(&robot_status_interface_);

  // publishers
  robot_status_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::ComauRobotStatus>(nh_, "robot_status", 100));
  ee_pose_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(nh_, "tf", 100));
  async_enable_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "async_enable", 1, true));
  return true;
} // namespace comau_hardware_interface

void ComauHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
  // dummy feedback assume perfect execution
  if (!(use_state_server_ || use_motion_server_)) {
    if (position_controller_running_)
      copyVector(joint_position_command_, joint_position_);
    return;
  }

  if (use_state_server_) {
    if (robot_ptr_->readMessagePackage()) {
      robot_ptr_->getTimeStamp(data_timestamp);
      robot_ptr_->getStatus(robot_status); // TODO : rename to robot_status
      robot_ptr_->getJointPosition(joint_position_);
      robot_ptr_->getEePosition(ee_position_);
      packet_read_ = true;
      publishRobotStatus();
      extractEndEffectorPose();
      publishEndEffectorPose();
      execute_joints_handler_ptr->set_status(robot_status);
      execute_joints_handler_ptr->set_allow_async(robot_ptr_->checkAllowAsync());
      execute_cartesian_handler_ptr->set_status(robot_status);
      execute_cartesian_handler_ptr->set_allow_async(robot_ptr_->checkAllowAsync());
    }
  }
}

void ComauHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {

  if (!(use_state_server_ || use_motion_server_)) {
    if (sensor_tracking_controller_running_)
      printVector(sensor_tracking_command_);
  }
  if (use_motion_server_) {

    if ((robot_status == comau_tcp_interface::RobotStatus::READY ||
         robot_status == comau_tcp_interface::RobotStatus::MOVING) &&
        packet_read_) {
      if (sensor_tracking_controller_running_) {
        robot_ptr_->writeCommand(sensor_tracking_command_, comau_driver::ControlMode::MODE_SENSOR_TRACKING);
      } else if (position_controller_running_) {
        // robot_ptr_->writeCommand(joint_position_command_, comau_driver::ControlMode::MODE_POSITION);
      } else {
        // if (robot_ptr_->checkAllowAsync()) {
        //  if (execute_joints_handler_ptr->traj_ready()) {
        //    execute_joints_handler_ptr->get_traj(goal_joint_traj_);
        //    robot_ptr_->writeTrajectoryCommand(goal_joint_traj_, comau_driver::ControlMode::MODE_JOINT_TRAJECTORY);
        //    execute_joints_handler_ptr->traj_send();
        //  }
        // Not implemented yet
        //}
        holdConnection();
      }
      packet_read_ = false;
    }
    // if ((robot_status == comau_tcp_interface::RobotStatus::SUCCEEDED ||
    //     robot_status == comau_tcp_interface::RobotStatus::ERROR) &&
    //    packet_read_) {
    //  robot_ptr_->resetState();
    //}
  }
} // namespace comau_hardware_interface

} // namespace comau_hardware_interface
