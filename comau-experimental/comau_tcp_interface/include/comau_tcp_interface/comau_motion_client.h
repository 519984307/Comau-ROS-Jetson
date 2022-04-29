/**
 * @file comau_motion_client.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date  20-5-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef COMAU_TCP_INTERFACE__COMAU_MOTION_CLIENT_H
#define COMAU_TCP_INTERFACE__COMAU_MOTION_CLIENT_H

#include <boost/shared_ptr.hpp>
#include <future>
#include <thread>

#include "comau_tcp_interface/comau_client_base.h"

namespace comau_tcp_interface {

class MotionClient : public ComauClientBase {
public:
  MotionClient() {}
  ~MotionClient();

  /**
   * @brief Read the params to connect with the server and initiates the receiving callback thread
   *
   * @param params ComauTcpInterfaceParameters for the State Server
   * @return true for success connection
   */
  bool initialize(const ComauTcpInterfaceParameters &params);
  /**
   * @brief Closes the connection with the server
   *
   */
  void close();
  /**
   * @brief Returns the connetion state
   *
   * @return true for connected
   * @return false for not connected
   */
  bool isConnected();
  /**
   * @brief
   *
   * @param msg
   * @return true
   * @return false
   */
  bool getLastMessage(utils::MessagePackage &msg);
  /**
   * @brief
   *
   * @param joint_values
   * @return true
   * @return false
   */
  bool sendJointTrajectoryMessage(utils::trajectoryf_t joint_values);
  /**
   * @brief
   *
   * @param joint_position_command
   * @return true
   * @return false
   */
  bool sendJointPositionMessage(utils::vector6f_t joint_position_command);
  /**
   * @brief
   *
   * @param sensor_tracking_command
   * @return true
   * @return false
   */
  bool sendSensorTrackingMessage(utils::vector6f_t sensor_tracking_command);
  /**
   * @brief
   *
   * @param joint_values
   * @return true
   * @return false
   */
  bool sendCartTrajectoryMessage(utils::trajectoryf_t trajectory);
  /**
   * @brief
   *
   * @param
   * @return true
   * @return false
   */
  bool sendTerminateMessage();
  /**
   * @brief
   *
   * @param
   * @return true
   * @return false
   */
  bool sendResetMessage();
  bool sendGripperMessage(bool gripper_cmd);

private:
  /**
   * @brief Read the message and parse it into the MessagePackage last_recv_msg_
   *
   * @param timeout  TODO implement the timeout for receive
   */
  void receive(std::chrono::milliseconds timeout);
  /**
   * @brief Send the serialized message package
   *
   * @param msg
   */
  bool send(utils::MessagePackage &msg);

  boost::shared_ptr<ComauTcpInterfaceParameters> params_ptr_; /**< private pointer for client parameters*/
  bool is_connected_;                                         /**< private variable holding the connection info*/

  boost::shared_ptr<ComauTcpConnection>
      tcp_interface_ptr_; /**< ComauTcpConnection responsible for the TCP connection */
};

} // namespace comau_tcp_interface

#endif // COMAU_MOTION_CLIENT_H
