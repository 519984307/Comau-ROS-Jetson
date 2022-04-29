#include "controller_manager_msgs/SwitchController.h"
#include "ros/ros.h"
#include <cstdlib>

/*
rosservice call /controller_manager/switch_controller "start_controllers:
- 'joint_group_position_controller'
stop_controllers:
- 'pos_joint_traj_controller'
strictness: 2"
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "switch_controlles_client");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<controller_manager_msgs::SwitchController>("/controller_"
                                                                                         "manager/"
                                                                                         "switch_"
                                                                                         "controller");
  controller_manager_msgs::SwitchController switch_controller;

  // sensor_tracking_controller
  // switch_controller.request.start_controllers.push_back("sensor_tracking_controller");
   switch_controller.request.stop_controllers.push_back("sensor_tracking_controller");
  // switch_controller.request.start_controllers.push_back("pos_joint_traj_controller");
  // switch_controller.request.stop_controllers.push_back("pos_joint_traj_controller");
  switch_controller.request.strictness = 2;

  if (client.call(switch_controller)) {
    ROS_INFO("The Switch Controller Response is : %d \n", switch_controller.response.ok);
  } else {
    ROS_ERROR("Failed to Switch Controllers");
    return 1;
  }

  return 0;
}