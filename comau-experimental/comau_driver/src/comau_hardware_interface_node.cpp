#include <ros/ros.h>

#include <comau_hardware_interface/comau_hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <csignal>

std::unique_ptr<comau_hardware_interface::ComauHardwareInterface> c_hw_interface_ptr;

void signalHandler(int signum) {

  ROS_WARN_STREAM("[comau_hw_interface] Interrupt signal (" << signum << ") received.\n");

  c_hw_interface_ptr.reset();

  exit(signum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "comau_hw_interface_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  c_hw_interface_ptr.reset(new comau_hardware_interface::ComauHardwareInterface);

  if (!c_hw_interface_ptr->init(nh, nh_local)) {
    ROS_ERROR_STREAM("[comau_hw_interface_node] Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_INFO_STREAM("[comau_hw_interface_node] HW interface initialized");
  controller_manager::ControllerManager cm(c_hw_interface_ptr.get(), nh);

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  double expected_cycle_time_ms = (1.0 / (static_cast<double>(c_hw_interface_ptr->getControlFrequency()))) * 1000;
  double control_cycle_time_tolerance_ms = 2.0; // TODO read this value from contollers yaml like loop_hz

  //
  ros::Rate hardware_interface_loop_hz(c_hw_interface_ptr->getControlFrequency());

  ROS_INFO_STREAM("[comau_hw_interface_node] Starting the control loop with frequency " << c_hw_interface_ptr->getControlFrequency() << " Hz");

  while (ros::ok()) {
    // Receive current state from robot
    c_hw_interface_ptr->read(timestamp, period);

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // Update ControllerManager
    cm.update(timestamp, period, c_hw_interface_ptr->shouldResetControllers());

    // Write commands
    c_hw_interface_ptr->write(timestamp, period);
    if (period.toSec() > expected_cycle_time_ms) {
      ROS_WARN_STREAM("[comau_hw_interface_node] Could not keep cycle rate of " << expected_cycle_time_ms * 1000 << "ms");
      ROS_WARN_STREAM("[comau_hw_interface_node] Actual cycle time:" << period.toNSec() / 1000000.0 << "ms");
    }
    hardware_interface_loop_hz.sleep();
    stopwatch_last = stopwatch_now;
  }

  spinner.stop();
  ROS_INFO_STREAM("[comau_hw_interface_node] Shutting down ");
  return 0;
}
