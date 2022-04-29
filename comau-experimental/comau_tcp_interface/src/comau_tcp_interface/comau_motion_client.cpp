/**
 * @file comau_motion_client.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date  20-6-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "comau_tcp_interface/comau_motion_client.h"

using namespace comau_tcp_interface::utils;

namespace comau_tcp_interface {

MotionClient::~MotionClient() {
  close();
}

bool MotionClient::initialize(const ComauTcpInterfaceParameters &params) {
  is_connected_ = false;
  // Assign parameters
  params_ptr_.reset(new ComauTcpInterfaceParameters(params));

  // Describe the incoming message
  incoming_msg_motion_descr_.push_back("timestamp");
  incoming_msg_motion_descr_.push_back("status");
  last_recv_msg_.reset(new MessagePackage(incoming_msg_motion_descr_));

  // Describe the outgoing messages

  // terminate msg
  terminate_msg_descr_.push_back("message_type");
  // reset msg
  reset_msg_descr_.push_back("message_type");
  // traj joint msg
  traj_joint_msg_descr_.push_back("message_type");
  traj_joint_msg_descr_.push_back("motion_type");
  traj_joint_msg_descr_.push_back("linear_velocity");
  traj_joint_msg_descr_.push_back("trajectory_size");
  traj_joint_msg_descr_.push_back("trajectory");
  // traj cart msg
  traj_cart_msg_descr_.push_back("message_type");
  traj_cart_msg_descr_.push_back("motion_type");
  traj_cart_msg_descr_.push_back("linear_velocity");
  traj_cart_msg_descr_.push_back("trajectory_size");
  traj_cart_msg_descr_.push_back("trajectory");
  // sensor tracking msg
  sns_trk_msg_descr_.push_back("message_type");
  sns_trk_msg_descr_.push_back("motion_type");
  sns_trk_msg_descr_.push_back("sensor_tracking_command");
  // position msg
  pos_msg_descr_.push_back("message_type");
  pos_msg_descr_.push_back("motion_type");
  pos_msg_descr_.push_back("joint_position_command");
  // gripper msg
  gripper_msg_descr_.push_back("gripper_command");

  try {
    tcp_interface_ptr_.reset(new ComauTcpConnection(*params_ptr_));
  } catch (const boost::system::system_error &e) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] " << e.what());
    return false;
  }

  return true;
}

void MotionClient::close() {
  is_connected_ = false;
  MessagePackage terminate_msg(terminate_msg_descr_);
  bool trigger_termination = true;
  if (terminate_msg.setData("terminate", trigger_termination)) {
    send(terminate_msg);
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));
}

bool MotionClient::isConnected() {
  return is_connected_;
}

bool MotionClient::getLastMessage(MessagePackage &msg) {
  MessagePackage *last = dynamic_cast<MessagePackage *>(last_recv_msg_.get());
  if (last != nullptr) {
    msg = *last;
    return true;
  }
  return false;
}

void MotionClient::receive(std::chrono::milliseconds timeout) {

  std::vector<uint8_t> recv_raw_data(last_recv_msg_->getCapacity());
  size_t read_len = 0;

  tcp_interface_ptr_->read(recv_raw_data, sizeof(recv_raw_data), read_len);
  if (read_len == 0) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Received zero bytes message");
    return;
  }

  MessageParser mp(recv_raw_data.data(), read_len);
  if (!last_recv_msg_->parseWith(mp)) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Received message could not parsed");
    return;
  }

  ROS_DEBUG_STREAM("[" << params_ptr_->log_tag << "] Received message : " << last_recv_msg_->toString());
}

bool MotionClient::send(MessagePackage &msg) {
  size_t written_size = 0;
  std::vector<uint8_t> send_buffer(msg.getSize());
  size_t serielized_buffer_size = msg.serializePackage(send_buffer.data());
  if (serielized_buffer_size > msg.getSize()) {
    send_buffer.resize(serielized_buffer_size);
  }

  tcp_interface_ptr_->write(send_buffer, serielized_buffer_size, written_size);

  if (written_size != serielized_buffer_size) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed buffer size: " << serielized_buffer_size
                         << "  written: " << written_size);
    return false;
  }
  ROS_DEBUG_STREAM("[" << params_ptr_->log_tag << "] " << msg.toString());
  return true;
}

bool MotionClient::sendJointPositionMessage(utils::vector6f_t joint_position_command) {
  utils::MessagePackage msg(getSendPosRecipe());
  bool com1 = msg.setData("message_type", MessageType::MOTION);
  bool com2 = msg.setData("motion_type", MotionType::POSITION);
  bool com3 = msg.setData("joint_position_command", joint_position_command);
  if (com1 && com2 && com3) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

bool MotionClient::sendSensorTrackingMessage(utils::vector6f_t sensor_tracking_command) {
  utils::MessagePackage msg(getSendSnsTrkRecipe());
  bool com1 = msg.setData("message_type", MessageType::MOTION);
  bool com2 = msg.setData("motion_type", MotionType::SNSTRK);
  bool com3 = msg.setData("sensor_tracking_command", sensor_tracking_command);
  if (com1 && com2 && com3) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

bool MotionClient::sendGripperMessage(bool gripper_cmd) {
  utils::MessagePackage msg(getSendGripperRecipe());
  uint32_t gripper_command;  
  if (gripper_cmd)
    gripper_command = 1;
  else
    gripper_command = 0;
  bool com1 = msg.setData("gripper_command", gripper_command);
  if (com1) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

bool MotionClient::sendJointTrajectoryMessage(utils::trajectoryf_t trajectory) {
  utils::MessagePackage msg(getSendTrajJointRecipe());
  uint32_t traj_size = static_cast<uint32_t>(trajectory.size());
  float default_linear_velocity = 0.1f;
  bool com1 = msg.setData("message_type", MessageType::MOTION);
  bool com2 = msg.setData("motion_type", MotionType::JOINT);
  bool com3 = msg.setData("linear_velocity", default_linear_velocity);
  bool com4 = msg.setData("trajectory_size", traj_size);
  bool com5 = msg.setData("trajectory", trajectory);
  if (com1 && com2 && com3 && com4 && com5) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

bool MotionClient::sendCartTrajectoryMessage(utils::trajectoryf_t trajectory) {
  utils::MessagePackage msg(getSendTrajCartRecipe());
  uint32_t traj_size = static_cast<uint32_t>(trajectory.size());
  float default_linear_velocity = 0.1f;
  bool com1 = msg.setData("message_type", MessageType::MOTION);
  bool com2 = msg.setData("motion_type", MotionType::CARTESIAN);
  bool com3 = msg.setData("linear_velocity", default_linear_velocity);
  bool com4 = msg.setData("trajectory_size", traj_size);
  bool com5 = msg.setData("trajectory", trajectory);
  if (com1 && com2 && com3 && com4 && com5) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

bool MotionClient::sendTerminateMessage() {
  utils::MessagePackage msg(getSendTerminateRecipe());
  bool com1 = msg.setData("message_type", MessageType::TERMINATE);
  if (com1) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

bool MotionClient::sendResetMessage() {
  utils::MessagePackage msg(getSendResetRecipe());
  bool com1 = msg.setData("message_type", MessageType::RESET);
  if (com1) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

} // namespace comau_tcp_interface

PLUGINLIB_EXPORT_CLASS(comau_tcp_interface::MotionClient, comau_tcp_interface::ComauClientBase)
