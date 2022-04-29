#pragma once

#include <ros/ros.h>
#include <sys/ioctl.h> // For ioctl, TIOCGWINSZ
#include <unistd.h>    // For STDOUT_FILENO
#include <utility>

#include "naming_constants.h"

// action libs
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <comau_msgs/ActionResultStatusConstants.h>
#include <comau_msgs/PerceptionAction.h>

// moveit libs
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

// pcl headers
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/publisher.h>

// tf static broadcaster
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

typedef actionlib::SimpleActionServer<comau_msgs::PerceptionAction> PerceptionActionServer;

class PerceptionHandlerServer 
{
    public:
    /**
     * @brief Construct a new PerceptionHandlerServer object
     *
     * @param nh
     * @param nh_local
     * @param name
     */
    PerceptionHandlerServer(ros::NodeHandle &nh, ros::NodeHandle &nh_local,
                            std::string name);
    /**
     * @brief Destroy the Move To Pose Handler Server object
     *
     */
    ~PerceptionHandlerServer(void);
    /**
     * @brief Initialization function
     *
     * @return true
     * @return false if something went wrong at the initialization
     */
    bool initialize();

    /**
   * @brief prints message when a goal has ended
   *
   */
    void printLine();

    private:
    /**
     * @brief A callback function that handles the incoming goal
     *
     * @param goal see comau_msgs::MoveToPoseMoveItGoal
     */
    void executeCallback(const comau_msgs::PerceptionGoalConstPtr &goal);

    /**
     * @brief Function that resize a point cloud from [mm] to [m]
     *
     * @param cloud point cloud to be resized
     */
    pcl::PointCloud<pcl::PointXYZ> pcl_resize(pcl::PointCloud<pcl::PointXYZ> cloud);

    /**
     * @brief Function that convert a pcl to ros msg. Then publish the point cloud.
     */
    void publishPCL();
    

    // Action
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    std::string action_name_;
    comau_msgs::PerceptionFeedback feedback_;
    comau_msgs::PerceptionResult result_;
    ros::WallTime _start;
    boost::shared_ptr<PerceptionActionServer> as_ptr_;
    bool moveit_running = true, publish_succeeded_, publish_thread_done_;

    // PCL
    std::string tf_frame;
    sensor_msgs::PointCloud2 cloud_ros_msgs;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::string file_name, cloud_topic;
    pcl_ros::Publisher<sensor_msgs::PointCloud2> pub;
};