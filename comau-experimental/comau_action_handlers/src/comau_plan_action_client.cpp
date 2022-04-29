// rosrun comau_action_handlers comau_plan_action_client.cpp _pipeline:="ompl_stomp" _pose:=screw _planner:="SBLkConfigDefault" _timeout:=10 _runs:=100

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <comau_msgs/MoveToPoseMoveItAction.h>
#include <comau_msgs/MoveToPoseMoveItActionGoal.h>
#include <comau_msgs/MoveToPoseMoveItResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model/robot_model.h>
#include <tf2_eigen/tf2_eigen.h>
#include "moveit/robot_state/conversions.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "moveit_msgs/GetStateValidity.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/GetPositionFK.h>

double pathLength(moveit_msgs::MotionPlanResponse plan);
Eigen::Isometry3d fk(std::vector<double> joint_values);
std::vector<geometry_msgs::PoseStamped> battery();


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "comau_plan_action_client");
    ros::NodeHandle nh("~");
    actionlib::SimpleActionClient<comau_msgs::MoveToPoseMoveItAction> client("move_topose_handler_server/action", true);

    
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer(); //wait for client to connect to server
    ROS_INFO("Action server started, sending goal.");

    
    std::string pipeline;
    std::string planner;
    std::string pose;
    double timeout;
    double runs = 10;
    nh.getParam("pipeline", pipeline);
    nh.getParam("planner", planner);    
    nh.getParam("timeout", timeout);   
    nh.getParam("runs", runs);
    nh.getParam("pose",pose);
    std::vector<geometry_msgs::PoseStamped> targets;
    geometry_msgs::PoseStamped target;
    if (pose == "piastra")
    {
        // Pose piastra      
        target.header.frame_id = "base_link";
        target.pose.position.x = 0.5;
        target.pose.position.y = -0.3;
        target.pose.position.z = 0.1;
        target.pose.orientation.w = 0.0008719608304090798;
        target.pose.orientation.x = 2.754180422925856e-05;
        target.pose.orientation.y = 0.9999996423721313;
        target.pose.orientation.z = -1.1697989975800738e-05;
        targets.push_back(target);
    }
    else if(pose == "screw")
    {
        // Pose screwdriver
        target.header.frame_id = "base_link";
        target.pose.position.x = 0.5;
        target.pose.position.y = -0.3;
        target.pose.position.z = 0.1;
        target.pose.orientation.w = 0.0031610350124537945;
        target.pose.orientation.x = 0.7064439058303833;
        target.pose.orientation.y = 0.7077572345733643;
        target.pose.orientation.z = 0.002620849059894681;
        targets.push_back(target);
    }
    else if(pose == "battery")
    {
        targets = battery();
    }
    
    

    comau_msgs::MoveToPoseMoveItActionGoal msg;
    msg.goal.constraint_mode = 0;
    
    msg.goal.execute = false;
    msg.goal.send_trajectory = false;
    msg.goal.timeout = timeout;
    msg.goal.id = "";
    msg.goal.planning_pipeline = pipeline;
    msg.goal.geometric_planner = planner;
    msg.goal.velocity = 1.0;
    msg.goal.acceleration = 1.0;
    msg.goal.hasPlan = false;
    // List of planners
    std::vector<std::string> planner_list;
    // planner_list.push_back("AnytimePathShorteningkConfigDefault");
    // planner_list.push_back("RRTConnectkConfigDefault");
    planner_list.push_back("SBLkConfigDefault");
    // planner_list.push_back("ESTkConfigDefault");
    // planner_list.push_back("LBKPIECEkConfigDefault");
    // planner_list.push_back("BKPIECEkConfigDefault");
    // planner_list.push_back("KPIECEkConfigDefault");
    // planner_list.push_back("RRTkConfigDefault");
    // planner_list.push_back("RRTstarkConfigDefault");
    // planner_list.push_back("TRRTkConfigDefault");
    // planner_list.push_back("PRMkConfigDefault");
    // planner_list.push_back("PRMstarkConfigDefault");
    // planner_list.push_back("BiESTkConfigDefault");
    // planner_list.push_back("ProjESTkConfigDefault");
    // planner_list.push_back("LazyPRMstarkConfigDefault");
    
    
    int succeded = 0;
    double total_time = 0;
    comau_msgs::MoveToPoseMoveItActionGoal msg_ex;
    for (size_t j = 0; j < runs; j++)
    {   
        double cycle_time = 0;
        for (size_t i = 0; i < targets.size(); i++)
        {
            msg.goal.target_pose = targets[i];
            comau_msgs::MoveToPoseMoveItResultConstPtr result;                     
            client.sendGoal(msg.goal);
            bool finished_before_timeout = client.waitForResult(ros::Duration(60.0));
            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = client.getState();              
                //ROS_INFO("Action finished: %s",state.toString().c_str());
                if(state.toString() == "SUCCEEDED")
                {
                    result = client.getResult();
                    //l = l + pathLength(result->action_result.plan);
                    cycle_time  = cycle_time + result->action_result.plan.planning_time;
                    ROS_INFO_STREAM("Time planning pose " << i << " - " << result->action_result.plan.planning_time);
                    succeded = succeded + 1;
                }
                else
                {
                    ROS_INFO_STREAM("Fail");
                    break;
                }
            
            
                msg_ex.goal.send_trajectory = false;
                msg_ex.goal.execute = true;
                msg_ex.goal.hasPlan = false;
                client.sendGoal(msg_ex.goal);
                bool finished_before_timeout_ex = client.waitForResult(ros::Duration(30));
                if (finished_before_timeout_ex)
                {
                    actionlib::SimpleClientGoalState state = client.getState();
                    ROS_INFO("Action finished: %s",state.toString().c_str());
                }
                else
                {
                    ROS_INFO("Action did not finish before the time out.");             
                }
            }            
        }
        double succ_rate = (succeded/targets.size()) * 100;
        ROS_INFO_STREAM("Cycle Success Rate: " << succ_rate << "%");
        total_time = total_time + cycle_time;
        succeded = 0;
    }
    
    // Stats
    double av_time = total_time/runs;
    
    std::cout << "\n";
    std::cout << "====================================================";
    std::cout << "\n";
    ROS_INFO_STREAM("Planning pipeline: " << msg.goal.planning_pipeline);
    ROS_INFO_STREAM("Planner: " << msg.goal.geometric_planner);
    ROS_INFO_STREAM("Num runs: " << runs);
    ROS_INFO_STREAM("Timeout: " << msg.goal.timeout);
    ROS_INFO_STREAM("Average Time: " << av_time);
    //ROS_INFO_STREAM("Average Length: " << av_length);
    
    std::cout << "====================================================";
    std::cout << "\n";











    // for (size_t i = 0; i < targets.size(); i++)
    // {         
    //     for (size_t i = 0; i < planner_list.size(); i++)
    //     {
    //         double l = 0;   
                   
    //         int count = 0;
    //         int succeded = 0;
    //         int failed = 0;
    //         //msg.goal.geometric_planner = planner_list.at(i);
            
    //         msg.goal.target_pose = targets[i];
    //         comau_msgs::MoveToPoseMoveItResultConstPtr result;
                      
    //         client.sendGoal(msg.goal);
    //         bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));
    //         if (finished_before_timeout)
    //         {
    //             actionlib::SimpleClientGoalState state = client.getState();              
    //             //ROS_INFO("Action finished: %s",state.toString().c_str());
    //             if(state.toString() == "SUCCEEDED")
    //             {
    //                 result = client.getResult();
    //                 //l = l + pathLength(result->action_result.plan);
    //                 time  = time + result->action_result.plan.planning_time;
    //                 succeded = succeded + 1;
    //             }
    //         }
    //         else
    //         {
    //             failed = failed + 1;
    //             //ROS_INFO("Action did not finish before the time out.");
    //         }
                    
                

    //         ros::Duration(0.1).sleep();
            
    //         // Stats
    //         double av_time = time/succeded;
    //         //double av_length = l/succeded;
    //         double succ_rate = (succeded/runs) * 100;
    //         std::cout << "\n";
    //         std::cout << "====================================================";
    //         std::cout << "\n";
    //         ROS_INFO_STREAM("Planning pipeline: " << msg.goal.planning_pipeline);
    //         ROS_INFO_STREAM("Planner: " << msg.goal.geometric_planner);
    //         ROS_INFO_STREAM("Num runs: " << runs);
    //         ROS_INFO_STREAM("Timeout: " << msg.goal.timeout);
    //         ROS_INFO_STREAM("Average Time: " << av_time);
    //         //ROS_INFO_STREAM("Average Length: " << av_length);
    //         ROS_INFO_STREAM("Success Rate: " << succ_rate << "%");
    //         std::cout << "====================================================";
    //         std::cout << "\n";

    //     }
    // }
    
    
    
    

  //exit
  return 0;
};

Eigen::Isometry3d fk(std::vector<double> joint_values)
{
    
    moveit::planning_interface::MoveGroupInterface move_group_interface_("arm");
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface_.getCurrentState()->getJointModelGroup("arm");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));   
    joint_model_group = kinematic_model->getJointModelGroup("arm");
    kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    return kinematic_state->getGlobalLinkTransform("ee_link");
};

double pathLength(moveit_msgs::MotionPlanResponse plan)
{
    int num_poses = plan.trajectory.joint_trajectory.points.size();
    Eigen::Isometry3d p1,p2;
    double len = 0;
    double l;

    for (int i = 0; i < num_poses-1; i++)
    {
        p1 = fk(plan.trajectory.joint_trajectory.points[i].positions);
        p2 = fk(plan.trajectory.joint_trajectory.points[i+1].positions); 
        l = pow(pow((p2.translation().x() - p1.translation().x()),2) +
                pow((p2.translation().y() - p1.translation().y()),2) +
                pow((p2.translation().z() - p1.translation().z()),2),0.5);
        len = len + l;
    }
    return len;
};

std::vector<geometry_msgs::PoseStamped> battery()
{
    std::vector<geometry_msgs::PoseStamped> targets;
    geometry_msgs::PoseStamped target;
    geometry_msgs::PoseStamped home;

    home.header.frame_id = "base_link";
    home.pose.position.x = -0.012939024716615677;
    home.pose.position.y = -0.5067999362945557;
    home.pose.position.z = 0.10344038903713226 + 1*0.15;
    home.pose.orientation.w = -0.000995530979707837;
    home.pose.orientation.x = 0.9980995059013367;
    home.pose.orientation.y = 0.06158880516886711;
    home.pose.orientation.z = 0.001798825804144144;
    targets.push_back(home);

    target.header.frame_id = "base_link";
    target.pose.position.x = .21181628108024597;
    target.pose.position.y = 0.39044255018234253;
    target.pose.position.z =  0.011182015761733055;
    target.pose.orientation.w = -0.48528018593788147;
    target.pose.orientation.x =  0.515669584274292;
    target.pose.orientation.y = -0.49628162384033203;
    target.pose.orientation.z = 0.5022873878479004;
    targets.push_back(target);
    targets.push_back(home);

    target.header.frame_id = "base_link";
    target.pose.position.x = 0.21158117055892944;
    target.pose.position.y = 0.46766752004623413;
    target.pose.position.z =  -0.09574399888515472;
    target.pose.orientation.w = -0.48528072237968445;
    target.pose.orientation.x =  0.5155258774757385;
    target.pose.orientation.y = -0.4963207244873047;
    target.pose.orientation.z = 0.5023956894874573;
    targets.push_back(target);
    targets.push_back(home);

    target.header.frame_id = "base_link";
    target.pose.position.x = 0.46948131918907166;
    target.pose.position.y = 0.2743290960788727;
    target.pose.position.z =  0.09132146090269089;
    target.pose.orientation.w = 0.000248766562435776;
    target.pose.orientation.x =  0.7060626149177551;
    target.pose.orientation.y = 0.7081450819969177;
    target.pose.orientation.z = 0.0024617479648441076;
    targets.push_back(target);
    targets.push_back(home);

    target.header.frame_id = "base_link";
    target.pose.position.x = 0.46971338987350464;
    target.pose.position.y = 0.19560077786445618;
    target.pose.position.z =  0.091075599193573;
    target.pose.orientation.w = 0.0002487596939317882;
    target.pose.orientation.x =  0.7060626149177551;
    target.pose.orientation.y = 0.7081450819969177;
    target.pose.orientation.z = 0.002461748430505395;
    targets.push_back(target);
    targets.push_back(home);

    target.header.frame_id = "base_link";
    target.pose.position.x = 0.4704112708568573;
    target.pose.position.y = -0.04126343876123428;
    target.pose.position.z =  0.09033491462469101;
    target.pose.orientation.w = 0.0002491936320438981;
    target.pose.orientation.x =  0.7060627937316895;
    target.pose.orientation.y = 0.7081449031829834;
    target.pose.orientation.z = 0.0024620364420115948;
    targets.push_back(target);
    targets.push_back(home);
    return targets;
}