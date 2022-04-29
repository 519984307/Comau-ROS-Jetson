#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <boost/scoped_ptr.hpp>
#include <comau_msgs/SnsTrkPlot.h>
#include <comau_msgs/SnsTrkPlotReset.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>

namespace comau_controllers {

class SensorTrackingController : public controller_interface::Controller<hardware_interface::EffortJointInterface>,
                                 public hardware_interface::EffortJointInterface {
public:
  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh);

  void update(const ros::Time &time, const ros::Duration &period);

  void starting(const ros::Time &time);
  void stopping(const ros::Time &time);
 

private:
  void velCmdCB(const geometry_msgs::TwistStampedConstPtr &msg);
  void snsTrk(const std_msgs::Int32::ConstPtr &msg);
  bool resetplot(comau_msgs::SnsTrkPlotReset::Request &req, comau_msgs::SnsTrkPlotReset::Response &res);
  hardware_interface::JointHandle joint_;
  ros::Subscriber vel_cmd_sub_;
  ros::Subscriber sns_trk_type_sub_; /* sensor tracking type */
  std::vector<hardware_interface::JointHandle> joint_handles_;

  std::string root_name_, tip_name_;
  bool got_msg_;
  ros::Time last_msg_;
  ros::Duration dead_man_timeout_;

  double ee_vel_limit_;
  bool with_plot_;
  
  size_t num_joints_;
  std::vector<std::string> joint_names_;
  int loop_hz_,sns_trk_type_,sns_trk_type_prev, t_msg;
  double diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw;
  double diff_x_last, diff_y_last, diff_z_last, diff_roll_last, diff_pitch_last, diff_yaw_last;
  // ====== for plotting data ==============================================
  std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::SnsTrkPlot>>
      sns_trk_plot_pub_; /**< ROS helper publisher for sensor tracking performance see SnsTrkPlot.msg */
  comau_msgs::SnsTrkPlot plot_msg_;
  double running_sum_target_, running_sum_real_, error_, total_error_;
  bool first_msg_ = true;
  tf::Vector3 start_ee_pos_;    // this gets removed from ee pose
  tf::Vector3 start_ee_ori_; // this gets removed from ee pose
  tf::TransformListener ee_tf_listener_;
  tf::StampedTransform ee_transform_;
  ros::ServiceServer plot_reset_service_;
  //=======================================================================
};

} // namespace comau_controllers
