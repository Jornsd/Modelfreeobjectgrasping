#!/usr/bin/env python
# license removed for brevity
from pickle import TRUE
import rospy
import std_msgs.msg
import time
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 



#-----------------Variables -------------------
# 'left_finger', 'right_finger'
#----- Scanning pose positions (given in radians) -------
home_pos =      [0.03, -0.03]      #     Home/start position

open_pos =     [0.057, -0.057]      #     1. scanning position From above x = 0.4
close_pos =    [0.021, -0.021]  #     2. scanning position From sleep position x = 0

pos_nr = 2           #Counter for loop. Nr of poses to visit (visit home in start and end)
sleepy_time = 5      #seconds waited afther each position initiated not to affect transformation of points.






def gripper():
    pub = rospy.Publisher('vx300/gripper_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('grip_trajectory_pub', anonymous=True)

    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time


    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time
    
    for i in range(pos_nr): #loop for movements
        if i==1:
            new_pos = open_pos
            pos_message = "Move robot back to sleep position."
            time.sleep(3)
            
        elif i==0:
            new_pos = close_pos
            pos_message = "Move robot to home before scan patteren initiated."
            
        else:
            new_pos = home_pos
            pos_message = "No position available, move to home..."


        joints_str = JointTrajectory()
        joints_str.header = std_msgs.msg.Header()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.joint_names = ['left_finger', 'right_finger']
        point=JointTrajectoryPoint()
        point.positions = new_pos
        point.time_from_start = rospy.Duration(1,0)
        joints_str.points.append(point)

        pub.publish(joints_str) #gives command message to arm_controller
        rospy.loginfo(pos_message)

        time.sleep(sleepy_time)

    rospy.loginfo("The robot is done with the scanning of the object. Script will stop execution.")

    executionTime = rospy.Time.now() - startTime
    xTime_seconds = executionTime.to_sec()

    print("The execution time for current point cloud collection used ", xTime_seconds, " seconds.")

if __name__ == '__main__':
    try:
        gripper()
    except rospy.ROSInterruptException:
        pass

