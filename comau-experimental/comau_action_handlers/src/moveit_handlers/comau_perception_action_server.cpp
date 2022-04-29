#include "moveit_handlers/comau_perception_action_server.h"

PerceptionHandlerServer::PerceptionHandlerServer(ros::NodeHandle &nh, ros::NodeHandle &nh_local, std::string name)
    : nh_(nh), nh_local_(nh_local), action_name_(std::move(name)), tf_frame("camera") 
{
    cloud_topic = "cloud";
    pub.advertise(nh_, cloud_topic.c_str(), 1);
    nh_local.param("frame_id", tf_frame, std::string("camera"));
    ROS_INFO_STREAM("Publishing data on topic \"" << nh_.resolveName(cloud_topic) << "\" with frame_id \"" << tf_frame << "\"");

}

PerceptionHandlerServer::~PerceptionHandlerServer(void) {}

bool PerceptionHandlerServer::initialize()
{
    // // Get planning group from parameter server
    // if (!nh_local_.getParam("move_group", PLANNING_GROUP_))
    // {
    //     ROS_ERROR_STREAM("[ PLANNING_GROUP_ ] Required parameter " << nh_local_.resolveName("move_group") << " not given.");
    //     return false;
    // }

    // Start Comau Perception Action Server
    ROS_INFO_STREAM("Starting up the Comau Perception Action Server ...  ");
    try 
    {
        as_ptr_.reset(new PerceptionActionServer(nh_, action_name_, boost::bind(&PerceptionHandlerServer::executeCallback, this, _1), false));
        as_ptr_->start();
    } 
    catch (...) {
        ROS_ERROR_STREAM("Comau Perception Action Server cannot not start.");
        return false;
    }
    ROS_INFO_STREAM("Ready to publish scenes!  ");
    return true;   
}

void PerceptionHandlerServer::executeCallback(const comau_msgs::PerceptionGoalConstPtr &goal)
{
    // Start a timer
    ros::WallTime _start;
    _start = ros::WallTime::now(); // Start timer
    ROS_INFO("[%s] Perception goal received", action_name_.c_str());

    // Check if moveit framework is running. If false, abort goal request by the client
    if (!moveit_running) // TODO - check if moveit is running, default true!
    {
        ROS_ERROR("Trying to plan goal with MoveIt but MoveIt is not running.");
        feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        // feedback_.state = "Cannot process goal";
        as_ptr_->publishFeedback(feedback_);
        ros::Duration(0.1).sleep();
        result_.action_result.success = false;
        result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
        result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
        as_ptr_->setAborted(result_);
        printLine();
        return;
    }

    // Publish static transform base_link / camera frames
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "base_link";
    static_transformStamped.child_frame_id = tf_frame;
    static_transformStamped.transform.translation.x = goal->x;
    static_transformStamped.transform.translation.y = goal->y;
    static_transformStamped.transform.translation.z = goal->z;
    tf2::Quaternion quat;
    quat.setRPY(goal->r, goal->p, goal->yw);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);

    // Load PCD file
    file_name = goal->path;
    if (file_name == "" || pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) == -1)
    {
        ROS_ERROR_STREAM("Could not load file \"" << file_name  <<"\". Aborting...");
        feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        as_ptr_->publishFeedback(feedback_);
        ros::Duration(0.1).sleep();
        result_.action_result.success = false;
        result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
        result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
        as_ptr_->setAborted(result_);
        printLine();
        return;
    }
    else
    {
        cloud.header.frame_id = tf_frame;
        publish_thread_done_ = false;
        boost::thread perception_thread(&PerceptionHandlerServer::publishPCL, this);

        while(!publish_thread_done_)
        {
            ros::Duration(0.001).sleep();
            feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            as_ptr_->publishFeedback(feedback_);

            if (as_ptr_->isPreemptRequested() || !ros::ok()) // stop if ros shutdown or a new request is preempted(?)
            {
                feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
                as_ptr_->publishFeedback(feedback_);
                ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                            // publishing the result
                ROS_WARN("[%s] Preempted", action_name_.c_str());
                result_.action_result.success = false;
                result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;
                result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
                as_ptr_->setPreempted(result_);
                printLine();
                return;
            }
        }

        result_.action_result.success = true;
        result_.action_result.status = comau_msgs::ActionResultStatusConstants::SUCCESS;
        result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
        as_ptr_->setSucceeded(result_);
        return;
    }

    // Action aborted mock
        // feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        // as_ptr_->publishFeedback(feedback_);
        // ros::Duration(0.1).sleep();
        // result_.action_result.success = false;
        // result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
        // result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
        // as_ptr_->setAborted(result_);
        // printLine();
        // return;
    
    // Action succeded mock
        // result_.action_result.success = true;
        // result_.action_result.status = comau_msgs::ActionResultStatusConstants::SUCCESS;
        // result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
        // as_ptr_->setSucceeded(result_);
        // return;

printLine();
}

pcl::PointCloud<pcl::PointXYZ> PerceptionHandlerServer::pcl_resize(pcl::PointCloud<pcl::PointXYZ> cloud)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_resized = cloud; 
    std::cout << cloud.sensor_origin_[0];
    std::cout << cloud.sensor_origin_[1];
    std::cout << cloud.sensor_origin_[2];
    std::cout << cloud.sensor_origin_[3];

    std::cout << "w" << cloud.sensor_orientation_.w();
    std::cout << "x" << cloud.sensor_orientation_.x();
    std::cout << "y" << cloud.sensor_orientation_.y();
    std::cout << "z" << cloud.sensor_orientation_.z();
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud_resized.points[i].x = cloud.points[i].x * 1e-03;
        cloud_resized.points[i].y = cloud.points[i].y * 1e-03;
        cloud_resized.points[i].z = cloud.points[i].z * 1e-03;
    }
    return cloud_resized;
  }

void PerceptionHandlerServer::publishPCL()
{
    ROS_INFO("Publishing point cloud to octomap server..");
    pcl::PointCloud<pcl::PointXYZ> cloud_resized = pcl_resize(cloud);  
    pcl::toROSMsg(cloud_resized, cloud_ros_msgs);
    pub.publish(cloud_ros_msgs);
    ROS_INFO("Point cloud published.");
    publish_thread_done_ = true;
}

void PerceptionHandlerServer::printLine() 
{
  struct winsize size;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
  for (uint i = 0; i < size.ws_col; i++)
    std::cout << "=";
  std::cout << "\n";
}