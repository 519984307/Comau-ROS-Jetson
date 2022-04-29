#include <comau_tools/SnsTrkPlotConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <math.h>
#include <sstream>

bool en_x = false, en_y = false, en_z = false, en_roll = false, en_pitch = false, en_yaw = false, en_absolute_mode = false, en_relative_mode = false;
double amplitude = 0, frequency = 0;
int cycles = 0;
bool start = false;

void callback(comau_tools::SnsTrkPlotConfig &config, uint32_t level) {
  en_x = config.en_x;
  en_y = config.en_y;
  en_z = config.en_z;
  en_roll = config.en_roll;
  en_pitch = config.en_pitch;
  en_yaw = config.en_yaw;
  en_absolute_mode = config.en_absolute_mode;
  en_relative_mode = config.en_relative_mode;
  amplitude = config.amplitude;
  frequency = config.frequency;
  cycles = config.cycles;
  start = config.start;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "twist_sin_pub");

  dynamic_reconfigure::Server<comau_tools::SnsTrkPlotConfig> server;
  dynamic_reconfigure::Server<comau_tools::SnsTrkPlotConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::NodeHandle nh;
  ros::Publisher twist_sin_vel_pub      = nh.advertise<geometry_msgs::TwistStamped>("arm_cmd_vel", 1000);
  ros::Publisher twist_sns_trk_type_pub = nh.advertise<std_msgs::Int32>("sns_trk_type", 1000);

  ros::Rate loop_rate(200);
  geometry_msgs::TwistStamped twist_sin_msg_, twist_stop_msg_;
  twist_sin_msg_.header.stamp = ros::Time::now();
  twist_sin_msg_.twist.linear.y = 0;
  twist_sin_msg_.twist.linear.z = 0;
  twist_sin_msg_.twist.angular.x = 0;
  twist_sin_msg_.twist.angular.y = 0;
  twist_sin_msg_.twist.angular.z = 0;
  twist_stop_msg_ = twist_sin_msg_;
  
  std_msgs::Int32 twist_sns_trk_val;
  twist_sns_trk_val.data = -1;

  double val;

  int i = 0;
  // sin(i/50)*0.5
  while (ros::ok()) {
    
    if (frequency != 0.0)
      val = sin(2. * M_PI * frequency * (i * (1. / 500.))) * amplitude;
    else
      val = amplitude;

    twist_sin_msg_.header.stamp = ros::Time::now();

    if (en_x)
      twist_sin_msg_.twist.linear.x = val;
    else
      twist_sin_msg_.twist.linear.x = 0;
    if (en_y)
      twist_sin_msg_.twist.linear.y = val;
    else
      twist_sin_msg_.twist.linear.y = 0;
    if (en_z)
      twist_sin_msg_.twist.linear.z = val;
    else
      twist_sin_msg_.twist.linear.z = 0;
    if (en_roll)
      twist_sin_msg_.twist.angular.x = val;
    else
      twist_sin_msg_.twist.angular.x = 0;
    if (en_pitch)
      twist_sin_msg_.twist.angular.y = val;
    else
      twist_sin_msg_.twist.angular.y = 0;
    if (en_yaw)
      twist_sin_msg_.twist.angular.z = val;
    else
      twist_sin_msg_.twist.angular.z = 0;

    if (start) {
		
		if(en_absolute_mode == true && en_relative_mode == false)
		{
			twist_sns_trk_val.data = 10;
		} else if(en_absolute_mode == false && en_relative_mode == true)
		{
			twist_sns_trk_val.data = 6;
		} else if(en_absolute_mode == false && en_relative_mode == false)
		{
			twist_sns_trk_val.data = -1;
		} else {
			twist_sns_trk_val.data = -1;
		}
		
      twist_sin_vel_pub.publish(twist_sin_msg_);
      twist_sns_trk_type_pub.publish(twist_sns_trk_val);
      if (i == 4 * cycles * 500) {
        start = false;
        i = 0;
      }
      i++;
    } else {
      twist_sin_vel_pub.publish(twist_stop_msg_);
      twist_sns_trk_val.data = -1;
      twist_sns_trk_type_pub.publish(twist_sns_trk_val);
      i=0;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
