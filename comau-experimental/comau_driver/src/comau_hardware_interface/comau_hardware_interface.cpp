#include <sstream>

#include <comau_hardware_interface/comau_hardware_interface.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace comau_hardware_interface {
bool ComauHardwareInterface::init(ros::NodeHandle &nh, ros::NodeHandle &nh_local) {
  // Initialize Robot driver
  robot_ptr_.reset(new comau_driver::ComauRobot(nh, nh_local));
  if (!robot_ptr_->initialize()) {
    ROS_ERROR_STREAM("[comau_hw_interface] Failed to initialize robot driver");
    return false;
  }

  loop_hz_ = nh.param("comau_hardware_interface/loop_hz", 25);

  parallel_joint_fix_ = nh.param("parallel_joint_fix", false);

  // Names of the joints. Usually, this is given in the controller config file.
  if (!nh.getParam("comau_hardware_interface/joints", joint_names_)) {
    ROS_ERROR_STREAM("[comau_hw_interface] Cannot find required parameter "
                     << nh_local.resolveName("hardware_interface/joints") << " on the parameter server.");
    return false;
  }
  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Create ros_control interfaces
  for (size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM("[comau_hw_interface] Registering handles for joint " << joint_names_[i]);
    try {
      // Create joint state interface
      joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

      // Create position joint interface
      position_joint_interface_.registerHandle(hardware_interface::JointHandle(
          joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
    } catch (const HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("[comau_hw_interface] " << e.what());
      return false;
    }

    // TODO add the following in future
    // JointLimits limits;
    // SoftJointLimits softLimits;
    // getJointLimits(joint.name, nh_, limits)
    // PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
    // positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  return true;
}

void ComauHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
  // TODO check connection
  robot_ptr_->getJointPosition(joint_position_);
  // Aura has depended the joints 2 and 3 while in urdf are independed this a trick
  if (parallel_joint_fix_) {
    joint_position_[2] += joint_position_[1];
  }
  // published TF of EE directly from robot
  robot_ptr_->pubEeTF();
  // TODO when error -> controller_reset_necessary_ = true;
}

void ComauHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
  //  robot_ptr_->writeJointCommand(joint_position_command_);

  robot_ptr_->publishRobotStatus();
}

int ComauHardwareInterface::getControlFrequency() const {
  return loop_hz_;
}

bool ComauHardwareInterface::shouldResetControllers() {
  if (controller_reset_necessary_) {
    controller_reset_necessary_ = false;
    return true;
  } else {
    return false;
  }
}

} // namespace comau_hardware_interface
