#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <comau_msgs/PerceptionAction.h>
#include <comau_msgs/PerceptionActionGoal.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comau_perception_action_client");
    actionlib::SimpleActionClient<comau_msgs::PerceptionAction> client("perception_handler_server/action", true);
    ROS_INFO("Waiting for perception action server to start.");
    client.waitForServer(); //wait for client to connect to server
    ROS_INFO("Perception Action server started, sending goal.");

    

    comau_msgs::PerceptionActionGoal msg;
    msg.goal.path = "/home/antonio/Lib/RosService/Point_Cloud/pcl_obstacle.pcd";

    client.sendGoal(msg.goal);

    bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}