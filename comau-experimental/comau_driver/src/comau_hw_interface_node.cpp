#include <ros/ros.h>

#include <comau_hardware_interface/comau_hw_control_loop.h>
#include <comau_hardware_interface/comau_hw_interface.h>
#include <controller_manager/controller_manager.h>
#include <csignal>

boost::shared_ptr<comau_hardware_interface::ComauHardwareInterface> c_hw_interface_ptr;
boost::shared_ptr<comau_hardware_control_loop::ComauHWControlLoop> c_hw_control_loop_ptr;

void signalHandler(int signum) {

  ROS_WARN_STREAM("[comau_hw_interface] Interrupt signal (" << signum << ") received.\n");

  c_hw_interface_ptr.reset();
  c_hw_control_loop_ptr.reset();

  exit(signum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "comau_hardware_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);
  // Create the hardware interface
  c_hw_interface_ptr.reset(new comau_hardware_interface::ComauHardwareInterface);
  if (!c_hw_interface_ptr->init(nh, nh_local)) {
    ROS_ERROR_STREAM("[comau_hw_interface_node] Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_INFO_STREAM("[comau_hw_interface_node] HW interface initialized");
  // Start the control loop
  c_hw_control_loop_ptr.reset(new comau_hardware_control_loop::ComauHWControlLoop(nh, c_hw_interface_ptr));
  c_hw_control_loop_ptr->run(); // Blocks until shutdown signal recieved

  return 0;
}
