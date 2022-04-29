#include <comau_controllers/sensor_tracking_controller.h>
#include <pluginlib/class_list_macros.h>
namespace comau_controllers {

bool SensorTrackingController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh) {

  // Names of the joints. Usually, this is given in the controller config file.
  if (!nh.getParam("/sensor_tracking_controller/joints", joint_names_)) {
    ROS_ERROR_STREAM("[comau_controllers] Cannot find required parameter "
                     << nh.resolveName("comau_hardware_interface/joints") << " on the parameter server.");
    return false;
  }
  double dead_man_timeout;
  nh.param<double>("/sensor_tracking_controller/dead_man_timeout", dead_man_timeout, 0.2);
  dead_man_timeout_ = ros::Duration(dead_man_timeout);

  nh.param<double>("/sensor_tracking_controller/ee_vel_limit", ee_vel_limit_, 0.1);
  loop_hz_ = nh.param("/comau_hardware_control_loop/loop_hz", 500);
  with_plot_ = nh.param("/sensor_tracking_controller/with_plot", false);
  num_joints_ = joint_names_.size();

  // Create ros_control interfaces
  for (size_t i = 0; i < num_joints_; ++i) {
    ROS_INFO_STREAM("[comau_controllers] Registering handles for joint " << joint_names_[i]);
    try {
      // get the joint object to use in the realtime loop
      joint_handles_.push_back(hw->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("[comau_controllers] " << e.what());
      return false;
    }
  }
  
  t_msg = 0;
  
  vel_cmd_sub_ =
      nh.subscribe<geometry_msgs::TwistStamped>("/arm_cmd_vel", 1, &SensorTrackingController::velCmdCB, this);
  sns_trk_type_sub_ =
      nh.subscribe<std_msgs::Int32>("/sns_trk_type", 1, &SensorTrackingController::snsTrk, this);
  // ========= Publish plot data

  sns_trk_plot_pub_.reset(new realtime_tools::RealtimePublisher<comau_msgs::SnsTrkPlot>(nh, "plot", 500));
  plot_msg_.target.resize(6);
  plot_msg_.real.resize(6);
  plot_msg_.error.resize(6);
  plot_msg_.total_error.resize(6);
  plot_reset_service_ = nh.advertiseService("/reset_plot_data", &SensorTrackingController::resetplot, this);

  return true;
}

void SensorTrackingController::update(const ros::Time &time, const ros::Duration &period) {
  if (got_msg_) {
	
    if ((time - last_msg_) >= dead_man_timeout_) {
      diff_x = 0.0;
      diff_y = 0.0;
      diff_z = 0.0;
      diff_roll = 0.0;
      diff_pitch = 0.0;
      diff_yaw = 0.0;
      got_msg_ = false;

      for (unsigned int i = 0; i < num_joints_; i++) {
        joint_handles_[i].setCommand(0.0);
      }

      return;
    }
    // Limits
    if (std::fabs(diff_x) > ee_vel_limit_) {
      if (diff_x >= 0)
        diff_x = ee_vel_limit_;
      else
        diff_x = -ee_vel_limit_;
    }
    if (std::fabs(diff_y) > ee_vel_limit_) {
      if (diff_y >= 0)
        diff_y = ee_vel_limit_;
      else
        diff_y = -ee_vel_limit_;
    }
    if (std::fabs(diff_z) > ee_vel_limit_) {
      if (diff_z >= 0)
        diff_z = ee_vel_limit_;
      else
        diff_z = -ee_vel_limit_;
    }
    if (std::fabs(diff_roll) > ee_vel_limit_) {
      if (diff_roll >= 0)
        diff_roll = ee_vel_limit_;
      else
        diff_roll = -ee_vel_limit_;
    }
    if (std::fabs(diff_pitch) > ee_vel_limit_) {
      if (diff_pitch >= 0)
        diff_pitch = ee_vel_limit_;
      else
        diff_pitch = -ee_vel_limit_;
    }
    if (std::fabs(diff_yaw) > ee_vel_limit_) {
      if (diff_yaw >= 0)
        diff_yaw = ee_vel_limit_;
      else
        diff_yaw = -ee_vel_limit_;
    }
    
    // Calculate end effector correction based on cmd_vel
	if( (sns_trk_type_ == 10) )
	{
		joint_handles_[0].setCommand(diff_x);
		joint_handles_[1].setCommand(diff_y);
		joint_handles_[2].setCommand(diff_z);
		joint_handles_[3].setCommand(diff_roll);
		joint_handles_[4].setCommand(diff_pitch);
		joint_handles_[5].setCommand(diff_yaw);
		
		diff_x_last     = diff_x;
		diff_y_last     = diff_y;
		diff_z_last     = diff_z;
		diff_roll_last  = diff_roll;
		diff_pitch_last = diff_pitch;
		diff_yaw_last   = diff_yaw;
		
		t_msg = 0;
	} else if ( (sns_trk_type_ == 6) )
	{
		joint_handles_[0].setCommand(diff_x / double(loop_hz_));
		joint_handles_[1].setCommand(diff_y / double(loop_hz_));
		joint_handles_[2].setCommand(diff_z / double(loop_hz_));
		joint_handles_[3].setCommand(diff_roll / double(loop_hz_));
		joint_handles_[4].setCommand(diff_pitch / double(loop_hz_));
		joint_handles_[5].setCommand(diff_yaw / double(loop_hz_));
		
		diff_x_last     = diff_x / double(loop_hz_);
		diff_y_last     = diff_y / double(loop_hz_);
		diff_z_last     = diff_z / double(loop_hz_);
		diff_roll_last  = diff_roll / double(loop_hz_);
		diff_pitch_last = diff_pitch / double(loop_hz_);
		diff_yaw_last   = diff_yaw / double(loop_hz_);
		
		t_msg = 0;
    } else {
		if(t_msg <= 1000)
		{
			t_msg = t_msg + 1;
			diff_x     = diff_x_last - t_msg*diff_x_last/1000;
			diff_y     = diff_y_last - t_msg*diff_y_last/1000;
			diff_z     = diff_z_last - t_msg*diff_z_last/1000;
			diff_roll  = diff_roll_last - t_msg*diff_roll_last/1000;
			diff_pitch = diff_pitch_last - t_msg*diff_pitch_last/1000;
			diff_yaw   = diff_yaw_last - t_msg*diff_yaw_last/1000;
			
			joint_handles_[0].setCommand(diff_x);
			joint_handles_[1].setCommand(diff_y);
			joint_handles_[2].setCommand(diff_z);
			joint_handles_[3].setCommand(diff_roll);
			joint_handles_[4].setCommand(diff_pitch);
			joint_handles_[5].setCommand(diff_yaw);
		}
	}
    
    // ============ Publish Plot Data =================
    if (with_plot_) {
      try {
        //ROS_WARN("I LOOK TRANSFORM");
        ee_tf_listener_.lookupTransform("base_link", "tool_controller", ros::Time(0), ee_transform_);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }
      if (first_msg_) {
        //ROS_WARN("FIRST MSG");
        start_ee_pos_ = ee_transform_.getOrigin();
        tf::Matrix3x3 mat(ee_transform_.getRotation());
        mat.getRPY(start_ee_ori_[0], start_ee_ori_[1], start_ee_ori_[2]);
        for (size_t i = 0; i < 6; i++) {
          plot_msg_.target[i] = 0.0;
          plot_msg_.real[i] = 0.0;
          plot_msg_.error[i] = 0.0;
          plot_msg_.total_error[i] = 0.0;
        }
        first_msg_ = false;
      }
      plot_msg_.header.stamp = ros::Time::now();
      
      if( (sns_trk_type_ == 10)  )
      {
        plot_msg_.real[0] = ee_transform_.getOrigin().x() - start_ee_pos_[0];
        plot_msg_.target[0] = diff_x;
        plot_msg_.error[0] = plot_msg_.target[0] - plot_msg_.real[0];
        plot_msg_.total_error[0] += plot_msg_.error[0];
            
        plot_msg_.real[1] = ee_transform_.getOrigin().y() - start_ee_pos_[1];
        plot_msg_.target[1] = diff_y;
        plot_msg_.error[1] = plot_msg_.target[0] - plot_msg_.real[1];
        plot_msg_.total_error[1] += plot_msg_.error[1];
            
        plot_msg_.real[2] = ee_transform_.getOrigin().z() - start_ee_pos_[2];
        plot_msg_.target[2] = diff_z;
        plot_msg_.error[2] = plot_msg_.target[2] - plot_msg_.real[2];
        plot_msg_.total_error[2] += plot_msg_.error[2];
            
        double roll, pitch, yaw;
        tf::Matrix3x3 mat(ee_transform_.getRotation());
        mat.getRPY(roll, pitch, yaw);
            
        plot_msg_.real[3] = roll - start_ee_ori_[0];
        plot_msg_.target[3] = diff_roll;
        plot_msg_.error[3] = plot_msg_.target[3] - plot_msg_.real[3];
        plot_msg_.total_error[3] += plot_msg_.error[3];
            
        plot_msg_.real[4] = pitch - start_ee_ori_[1];
        plot_msg_.target[4] = diff_pitch;
        plot_msg_.error[4] = plot_msg_.target[4] - plot_msg_.real[4];
        plot_msg_.total_error[4] += plot_msg_.error[4];
            
        plot_msg_.real[5] = yaw - start_ee_ori_[2];
        plot_msg_.target[5] = diff_yaw;
        plot_msg_.error[5] = plot_msg_.target[5] - plot_msg_.real[5];
        plot_msg_.total_error[5] += plot_msg_.error[5];
      } else if ( (sns_trk_type_ == 6) ){
        plot_msg_.real[0] = ee_transform_.getOrigin().x() - start_ee_pos_[0];
        plot_msg_.target[0] += diff_x / double(loop_hz_);
        plot_msg_.error[0] = plot_msg_.target[0] - plot_msg_.real[0];
        plot_msg_.total_error[0] += plot_msg_.error[0];
            
        plot_msg_.real[1] = ee_transform_.getOrigin().y() - start_ee_pos_[1];
        plot_msg_.target[1] += diff_y / double(loop_hz_);
        plot_msg_.error[1] = plot_msg_.target[0] - plot_msg_.real[1];
        plot_msg_.total_error[1] += plot_msg_.error[1];
            
        plot_msg_.real[2] = ee_transform_.getOrigin().z() - start_ee_pos_[2];
        plot_msg_.target[2] += diff_z / double(loop_hz_);
        plot_msg_.error[2] = plot_msg_.target[2] - plot_msg_.real[2];
        plot_msg_.total_error[2] += plot_msg_.error[2];
            
        double roll, pitch, yaw;
        tf::Matrix3x3 mat(ee_transform_.getRotation());
        mat.getRPY(roll, pitch, yaw);
            
        plot_msg_.real[3] = roll - start_ee_ori_[0];
        plot_msg_.target[3] += diff_roll / double(loop_hz_);
        plot_msg_.error[3] = plot_msg_.target[3] - plot_msg_.real[3];
        plot_msg_.total_error[3] += plot_msg_.error[3];
            
        plot_msg_.real[4] = pitch - start_ee_ori_[1];
        plot_msg_.target[4] += diff_pitch / double(loop_hz_);
        plot_msg_.error[4] = plot_msg_.target[4] - plot_msg_.real[4];
        plot_msg_.total_error[4] += plot_msg_.error[4];
            
        plot_msg_.real[5] = yaw - start_ee_ori_[2];
        plot_msg_.target[5] += diff_yaw / double(loop_hz_);
        plot_msg_.error[5] = plot_msg_.target[5] - plot_msg_.real[5];
        plot_msg_.total_error[5] += plot_msg_.error[5];
    }
    
      // publish data
      if (sns_trk_plot_pub_) {
        if (sns_trk_plot_pub_->trylock()) {
          sns_trk_plot_pub_->msg_ = plot_msg_;
          sns_trk_plot_pub_->unlockAndPublish();
        }
      }
    }
    // ================================================

    return;

  } else {
    for (unsigned int i = 0; i < num_joints_; i++) {
      joint_handles_[i].setCommand(0.0);
    }
  }
}

void SensorTrackingController::starting(const ros::Time &time) {
  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_roll = 0.0;
  diff_pitch = 0.0;
  diff_yaw = 0.0;
  sns_trk_type_ = 10;
  sns_trk_type_prev = -1;
  last_msg_ = ros::Time::now();
  got_msg_ = false;
  for (unsigned int i = 0; i < num_joints_; i++) {
    joint_handles_[i].setCommand(0.0);
  }
}
void SensorTrackingController::stopping(const ros::Time &time) {
  for (unsigned int i = 0; i < num_joints_; i++) {
    joint_handles_[i].setCommand(0.0);
  }
}

void SensorTrackingController::snsTrk(const std_msgs::Int32::ConstPtr &msg) {
  sns_trk_type_ = msg->data;
  
  last_msg_ = ros::Time::now();
  got_msg_ = true;
}

void SensorTrackingController::velCmdCB(const geometry_msgs::TwistStampedConstPtr &msg) {
  diff_x = msg->twist.linear.x;
  diff_y = msg->twist.linear.y;
  diff_z = msg->twist.linear.z;
  diff_roll = msg->twist.angular.x;
  diff_pitch = msg->twist.angular.y;
  diff_yaw = msg->twist.angular.z;

  last_msg_ = ros::Time::now();
  got_msg_ = true;
}

bool SensorTrackingController::resetplot(comau_msgs::SnsTrkPlotReset::Request &req,
                                         comau_msgs::SnsTrkPlotReset::Response &res) {
  first_msg_ = true;
  res.success = true;
  return true;
}

} // namespace comau_controllers

PLUGINLIB_EXPORT_CLASS(comau_controllers::SensorTrackingController, controller_interface::ControllerBase)
