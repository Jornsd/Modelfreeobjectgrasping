#!/usr/bin/env python
# license removed for brevity
from curses.panel import bottom_panel
from pickle import TRUE
#from tkinter import Y
import rospy
import std_msgs.msg
import time
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 


import camera_PC_transform as camera_pc_tf #Imports function from python script
import pc_assembler as pc_assemble #Imports function from python script

# 1 : roslaunch interbotix_xsarm_moveit xsarm_moveit.launch
# 2 : unpause gazebo (play button in left corner)
# 3 : CD into /vx300env_description/scripts
# 4 : python scan_trajectory.py

#-----------------Variables -------------------
# 'waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate'
#----- Scanning pose positions (given in radians) -------
home_pos =      [0, 0, 0, 0, 0]#           Home/start position


pre_pre_grasp_pos = [0, -1, 1, 0, 0]


pre_grasp_pos =[ 0.34689286,  0.34067659 , 0.540595 ,  -0.89111529 , 0.78685275]



#    Before grasping object
grasp_pos =[ 0.19059756,  0.51512862,  0.61672636, -1.14169869 , 0.78685275]

#      Grasping position
post_grasp_pos =[ 0.19059755,  0.00689241,  0.47482916, -0.49156515,  0.78685259]





#      After grasping object

sleepy_time = 5#seconds waited afther each position initiated not to affect transformation of points.

#-----------------Variables -------------------
# 'left_finger', 'right_finger'
#----- Scanning pose positions (given in radians) -------
home =      [0.03, -0.03]      #     Home/start position

open =     [0.057, -0.057]      #     1. scanning position From above x = 0.4
close =    [0.021, -0.021]  #     2. scanning position From sleep position x = 0

pub = rospy.Publisher('vx300/arm_controller/command', JointTrajectory, queue_size=10)
rospy.init_node('scan_trajectory_pub', anonymous=True)

#----------------- Workbench scanning script  -----------------

def home_position():
    
    pub = rospy.Publisher('vx300/arm_controller/command', JointTrajectory, queue_size=10)
    
    new_pos = home_pos
    pos_message= "Moving to pre_pre grasp position"

    joints_str = JointTrajectory()
    joints_str.header = std_msgs.msg.Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
    point=JointTrajectoryPoint()
    point.positions = new_pos
    point.time_from_start = rospy.Duration(1,0)
    joints_str.points.append(point)

    pub.publish(joints_str) #gives command message to arm_controller
    rospy.loginfo(pos_message)

def pre_pre_grasp():
    
    pub = rospy.Publisher('vx300/arm_controller/command', JointTrajectory, queue_size=10)
    
    new_pos = pre_pre_grasp_pos
    pos_message= "Moving to pre_pre grasp position"

    joints_str = JointTrajectory()
    joints_str.header = std_msgs.msg.Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
    point=JointTrajectoryPoint()
    point.positions = new_pos
    point.time_from_start = rospy.Duration(1,0)
    joints_str.points.append(point)

    pub.publish(joints_str) #gives command message to arm_controller
    rospy.loginfo(pos_message)


def pre_grasp():

    pub = rospy.Publisher('vx300/arm_controller/command', JointTrajectory, queue_size=10)
    
    new_pos = pre_grasp_pos
    pos_message= "Moving to pre grasp position"

    joints_str = JointTrajectory()
    joints_str.header = std_msgs.msg.Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
    point=JointTrajectoryPoint()
    point.positions = new_pos
    point.time_from_start = rospy.Duration(1,0)
    joints_str.points.append(point)

    pub.publish(joints_str) #gives command message to arm_controller
    rospy.loginfo(pos_message)


def grasp():

    pub = rospy.Publisher('vx300/arm_controller/command', JointTrajectory, queue_size=10)
   
    new_pos = grasp_pos
    pos_message= "Moving to grasp position"

    joints_str = JointTrajectory()
    joints_str.header = std_msgs.msg.Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
    point=JointTrajectoryPoint()
    point.positions = new_pos
    point.time_from_start = rospy.Duration(1,0)
    joints_str.points.append(point)

    pub.publish(joints_str) #gives command message to arm_controller
    rospy.loginfo(pos_message)

def pick():
    pub = rospy.Publisher('vx300/gripper_controller/command', JointTrajectory, queue_size=10)

    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time

    new_grasp = close
    pos_message = "Grasping object"
        
    joints_str = JointTrajectory()
    joints_str.header = std_msgs.msg.Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['left_finger', 'right_finger']
    point=JointTrajectoryPoint()
    point.positions = new_grasp
    point.time_from_start = rospy.Duration(1,0)
    joints_str.points.append(point)

    pub.publish(joints_str) #gives command message to arm_controller
    rospy.loginfo(pos_message)

    time.sleep(sleepy_time)

    rospy.loginfo("The robot is done with the scanning of the object. Script will stop execution.")

    executionTime = rospy.Time.now() - startTime
    xTime_seconds = executionTime.to_sec()

    print("The execution time for current point cloud collection used ", xTime_seconds, " seconds.")

def post_grasp():

    pub = rospy.Publisher('vx300/arm_controller/command', JointTrajectory, queue_size=10)
    
    new_pos = post_grasp_pos
    pos_message= "Moving to post grasp position"

    joints_str = JointTrajectory()
    joints_str.header = std_msgs.msg.Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
    point=JointTrajectoryPoint()
    point.positions = new_pos
    point.time_from_start = rospy.Duration(1,0)
    joints_str.points.append(point)

    pub.publish(joints_str) #gives command message to arm_controller
    rospy.loginfo(pos_message)

def release():
    pub = rospy.Publisher('vx300/gripper_controller/command', JointTrajectory, queue_size=10)

    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time

    new_grasp = open
    pos_message = "Releaseing object"
        
    joints_str = JointTrajectory()
    joints_str.header = std_msgs.msg.Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['left_finger', 'right_finger']
    point=JointTrajectoryPoint()
    point.positions = new_grasp
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
        time.sleep(2)
        home_position()
        time.sleep(1)
        release()
        time.sleep(1)
        pre_pre_grasp()
        time.sleep(1)
       # post_grasp()
       # time.sleep(1)
        pre_grasp()
        time.sleep(2)
        grasp()
        time.sleep(4)
        pick()
        post_grasp()
        time.sleep(2)
        release()
        
    except rospy.ROSInterruptException:
        pass