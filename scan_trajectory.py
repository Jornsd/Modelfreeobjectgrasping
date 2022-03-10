#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import time
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 

import camera_PC_transform as camera_pc_tf #Imports function from python script
import pc_assembler as pc_assemble #Imports function from python script

#-----------------Variables -------------------

#----- Scanning pose positions (given in radians) -------
home_pos =      [0, -1.5, 1.5, -1.5, -1.5, 0]#           Home/start position

first_pos =     [1.2, -1.5, 1.5, -1.0, -2.3, 0, 0]#     1. scanning position
second_pos =    [0, -2.6, 1.8, -1.4, -1.5, 0, 0]#       2. scanning position
third_pos =     [-1.5, -1.4, 1.3, -0.8, -0.9, 0, 0]#    3. scanning position
fourth_pos =    [-0.6, -1.1, 0.4, -0.7, -1.2, 0, 0]#    4. scanning position
fifth_pos =     [0, -1.0, 0.3, -0.7, -1.5, 0, 0]#       5. scanning position
sixth_pos =     [0.5, -1.0, 0.3, -0.7, -1.9, 0, 0]#     6. scanning position

pos_nr = 8 #Counter for loop. Nr of poses to visit (visit home in start and end)
sleepy_time = 5#seconds waited afther each position initiated not to affect transformation of points.


#----------------- Workbench scanning script  -----------------

def scan_trajectory():
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('scan_trajectory_pub', anonymous=True)

    take_picture = False #If position are reach for taking picture or returning to home.

    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time
    
    for i in range(pos_nr): #loop for movements
        if i==7:
            new_pos = home_pos
            pos_message = "Move robot back to home position."
            take_picture = False
        elif i==6:
            new_pos = sixth_pos
            pos_message = "Move robot to sixth position."
            take_picture = True
        elif i==5:
            new_pos = fifth_pos
            pos_message = "Move robot to fifth position."
            take_picture = True
        elif i==4:
            new_pos = fourth_pos
            pos_message = "Move robot to fourth position."
            take_picture = True
        elif i==3:
            new_pos = third_pos
            pos_message = "Move robot to third position."
            take_picture = True
        elif i==2:
            new_pos = second_pos
            pos_message = "Move robot to second position."
            take_picture = True
        elif i==1:
            new_pos = first_pos
            pos_message = "Move robot to first position."
            take_picture = True
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
        joints_str.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
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

if __name__ == '__main__':
    try:
        scan_trajectory()
    except rospy.ROSInterruptException:
        pass