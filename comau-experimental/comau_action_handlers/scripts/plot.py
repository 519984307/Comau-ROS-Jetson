#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
import numpy as np
import moveit_msgs
from moveit_msgs.msg import MoveGroupActionResult

vel_1 = []
t1 = []
vel_2 = []
t2 = []
vel_3 = []
t3 = []
vel_4 = []
t4 = []
vel_5 = []
t5 = []
vel_6 = []
t6 = []
def callback(msg):
    points = msg.result.planned_trajectory.joint_trajectory.points # array of waypoints
    fig,ax = plt.subplots(3,2)
    for i in range(np.size(points)):
        # vel_1.append(points[i].positions[0]), t1.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_2.append(points[i].positions[1]), t2.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_3.append(points[i].positions[2]), t3.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_4.append(points[i].positions[3]), t4.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_5.append(points[i].positions[4]), t5.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_6.append(points[i].positions[5]), t6.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)

        vel_1.append(points[i].velocities[0]), t1.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        vel_2.append(points[i].velocities[1]), t2.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        vel_3.append(points[i].velocities[2]), t3.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        vel_4.append(points[i].velocities[3]), t4.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        vel_5.append(points[i].velocities[4]), t5.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        vel_6.append(points[i].velocities[5]), t6.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)

        # vel_1.append(points[i].accelerations[0]), t1.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_2.append(points[i].accelerations[1]), t2.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_3.append(points[i].accelerations[2]), t3.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_4.append(points[i].accelerations[3]), t4.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_5.append(points[i].accelerations[4]), t5.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)
        # vel_6.append(points[i].accelerations[5]), t6.append(points[i].time_from_start.secs + points[i].time_from_start.nsecs*1e-09)

        

    ax[0,0].plot(t1,vel_1,'-o')
    ax[1,0].plot(t2,vel_2,'-o')
    ax[2,0].plot(t3,vel_3,'-o')
    ax[0,1].plot(t4,vel_4,'-o')
    ax[1,1].plot(t5,vel_5,'-o')
    ax[2,1].plot(t6,vel_6,'-o')
    #plt.plot(vel_1)
    #plt.ylabel('Velocity')
    plt.show()
    vel_1.clear()
    vel_2.clear()
    vel_3.clear()
    vel_4.clear()
    vel_5.clear()
    vel_6.clear()
    t1.clear()
    t2.clear()
    t3.clear()
    t4.clear()
    t5.clear()
    t6.clear()
    



def listener():


    rospy.init_node('plot', anonymous=True)

    rospy.Subscriber("/move_group/result", MoveGroupActionResult, callback)

    
    rospy.spin()

if __name__ == '__main__':
    listener()