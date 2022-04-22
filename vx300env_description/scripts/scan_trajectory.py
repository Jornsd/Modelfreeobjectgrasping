#!/usr/bin/env python
# license removed for brevity
from curses.panel import bottom_panel
from pickle import TRUE
import rospy
import std_msgs.msg
import time
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 


import camera_PC_transform as camera_pc_tf #Imports function from python script
import pc_assembler as pc_assemble #Imports function from python script

# 1 : roslaunch interbotix_xsarm_movit xsarm_moveit.launch
# 2 : unpause gazebo (play knappen i venstre hjorne)
# 3 : Gaa inn i /vx300env_description/scripts
# 4 : python scan_trajectory.py

#-----------------Variables -------------------
# 'waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate'
#----- Scanning pose positions (given in radians) -------
home_pos =      [0, 0, 0, 0, 0]#           Home/start position

first_pos =     [0, 0, -0.5, 2, 0]         #  1. scanning position From above x = 0.4
second_pos =    [0, -1.75, 1, 1.2, 0]      #  2. scanning position From sleep position x = 0
third_pos =     [0, -0.8, 0, 1.8, 0]       #  3. scanning position From High sleep position x = 0
fourth_pos =    [0, 0.5, -1.1, 2.2, 0]     #  4. scanning position Other side of object x = 0.6
fifth_pos =     [0.3, 0, -0.5, 2, 0]       #  5. scanning position left side of object x = 0.3, y = 0.5
sixth_pos =     [-0.3, 0, -0.5, 2, 0,]     #  6. scanning position right side of object x = 0.3, y = -0.5

pos_nr = 8 #Counter for loop. Nr of poses to visit (visit home in start and end)
sleepy_time = 5#seconds waited afther each position initiated not to affect transformation of points.

#-----------------Variables -------------------
# 'left_finger', 'right_finger'
#----- Scanning pose positions (given in radians) -------
home =      [0.03, -0.03]      #     Home/start position

open =     [0.057, -0.057]      #     1. scanning position From above x = 0.4
close =    [0.021, -0.021]  #     2. scanning position From sleep position x = 0

gripper_nr = 2           #Counter for loop. Nr of poses to visit (visit home in start and end)

move_nr = 3

#----------------- Workbench scanning script  -----------------

def gripper():
    pub = rospy.Publisher('vx300/gripper_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('grip_trajectory_pub', anonymous=True)

    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time


    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time
    
    for i in range(gripper_nr): #loop for movements
        if i==1:
            new_grasp = open
            pos_message = "Opening gripper"
            time.sleep(3)
            
        elif i==0:
            new_grasp = close
            pos_message = "Move robot to home before scan patteren initiated."
            
        else:
            new_grasp = home_pos
            pos_message = "No position available, move to home..."


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

def scan_trajectory():
    pub = rospy.Publisher('vx300/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('scan_trajectory_pub', anonymous=True)
    

    take_picture = False #If position are reach for taking picture or returning to home.

    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time
    
    for i in range(pos_nr): #loop for movements
        if i==7:
            new_pos = second_pos
            pos_message = "Move robot back to sleep position."
            time.sleep(3)
            take_picture = False
        elif i==6:
            new_pos = sixth_pos
            pos_message = "Move robot to sixth position."
            time.sleep(3)
            take_picture = False
        elif i==5:
            new_pos = fifth_pos
            pos_message = "Move robot to fifth position."
            time.sleep(3)
            take_picture = False
        elif i==4:
            new_pos = fourth_pos
            pos_message = "Move robot to fourth position."
            time.sleep(3)
            take_picture = False  
        elif i==3:
            new_pos = third_pos
            pos_message = "Move robot to third position."
            time.sleep(3)
            take_picture = False
        elif i==2:
            new_pos = second_pos
            pos_message = "Move robot to second position."
            time.sleep(0)
            take_picture = False
        elif i==1:
            new_pos = first_pos
            pos_message = "Move robot to first position."
            time.sleep(0)
            take_picture = False
        elif i==0:
            new_pos = home_pos
            pos_message = "Move robot to home before scan patteren initiated."
            take_picture = False
        else:
            new_pos = home_pos
            pos_message = "No position available, move to home..."
            take_picture = False

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
        
        time.sleep(sleepy_time)

        if take_picture == True:
            camera_pc_tf.Tf_camera_PC()
            rospy.wait_for_message('/camera_PC_transposed', PointCloud2)
            pc_assemble.assemble_voxel_pc()
            take_picture = False

    rospy.loginfo("The robot is done with the scanning of the object. Script will stop execution.")

    executionTime = rospy.Time.now() - startTime
    xTime_seconds = executionTime.to_sec()

    print("The execution time for current point cloud collection used ", xTime_seconds, " seconds.")

def go_to_pre_pose():

    pub = rospy.Publisher('vx300/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('scan_trajectory_pub', anonymous=True)

    for i in range(pos_nr):
        i=0
        new_pos =home_pos
        pos_message= "GETTING READY FOR GRASP"

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





if __name__ == '__main__':
    try:
        scan_trajectory()
    except rospy.ROSInterruptException:
        pass