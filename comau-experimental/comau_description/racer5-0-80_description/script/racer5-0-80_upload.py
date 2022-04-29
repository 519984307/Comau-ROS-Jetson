#!/usr/bin/env python

import roslaunch
import rospy
import rospkg

# Init node
rospy.init_node('upload_gripper', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# Get load_gripper flag from parameter server
load_gripper = rospy.get_param("/load_gripper", default='false')
gripper_yaml_file = rospy.get_param("/gripper_yaml_file", default='gripper.yaml')

# Get absolute path of package robot description
rospack = rospkg.RosPack()
path = rospack.get_path('racer5-0-80_description')
rospy.loginfo(load_gripper)


# Override (test)
#load_gripper = True

# Check if gripper has to be set
if load_gripper == True:
    
    cli_args = [path + '/launch/racer5-0-80_upload_1.launch','load_gripper:=true','gripper_yaml_file:=' + gripper_yaml_file]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
else:
    
    cli_args = [path + '/launch/racer5-0-80_upload_1.launch','load_gripper:=false']
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

# Roslaunch equivalent
launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
launch.start()
launch.spin()



