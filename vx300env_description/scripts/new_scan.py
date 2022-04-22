from operator import truediv
import rospy
import std_msgs.msg
import time
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from interbotix_xs_modules.arm import InterbotixManipulatorXS

import camera_PC_transform as camera_pc_tf #Imports function from python script
import pc_assembler as pc_assemble #Imports function from python script

#-----------------Variables -------------------
# 'waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate'
#----- Scanning pose positions (given in radians) -------

sleepy_time = 5
pos_nr = 8
## Home position
home_pos =      [0, 0, 0, 0, 0]                  # Home
## Scanning positions
first_pos =     [0, 0, -0.5, 2, 0]           # 1 
second_pos =    [0, -1.85, 1.55, 0.8, 0]     # 2
third_pos =     [0, -0.8, 0, 1.8, 0]         # 3
fourth_pos =    [0, 0.5, -1.1, 2.2, 0]       # 4
fifth_pos =     [0.3, 0, -0.5, 2, 0]         # 5
sixth_pos =     [-0.3, 0, -0.5, 2, 0,]       # 6

def scan_traj():
    bot = InterbotixManipulatorXS("vx300", "arm", "gripper")
    
    
    take_picture = False #If position are reach for taking picture or returning to home.

    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time

    for i in range(pos_nr): #loop for movements
        if i==7:
            bot.arm.go_to_sleep_pose()
            pos_message = "Move robot back to sleep position."
            time.sleep(3)
            take_picture = False
        elif i==6:
            bot.arm.set_joint_positions(sixth_pos)
            pos_message = "Move robot to sixth position."
            time.sleep(3)
            take_picture = True
        elif i==5:
            bot.arm.set_joint_positions(fifth_pos)
            pos_message = "Move robot to fifth position."
            time.sleep(3)
            take_picture = True
        elif i==4:
            bot.arm.set_joint_positions(fourth_pos)
            pos_message = "Move robot to fourth position."
            time.sleep(3)
            take_picture = True
        elif i==3:
            bot.arm.set_joint_positions(third_pos)
            pos_message = "Move robot to third position."
            time.sleep(3)
            take_picture = True
        elif i==2:
            bot.arm.set_joint_positions(second_pos)
            pos_message = "Move robot to second position."
            time.sleep(3)
            take_picture = True
        elif i==1:
            bot.arm.set_joint_positions(first_pos)
            pos_message = "Move robot to first position."
            time.sleep(3)
            take_picture = True
        elif i==0:
            bot.arm.go_to_home_pose()
            pos_message = "Move robot to home before scan patteren initiated."
            time.sleep(1)
            take_picture = False
        else:
            bot.arm.go_to_home_pose()
            pos_message = "No position available, move to home..."
            take_picture = False

            
        time.sleep(sleepy_time)
        
        if take_picture == True:
              camera_pc_tf.Tf_camera_PC()
              rospy.wait_for_message('/camera_PC_transposed', PointCloud2)
              pc_assemble.assemble_voxel_pc()
              take_picture = False

        rospy.loginfo(pos_message)
    rospy.loginfo("The robot is done with the scanning of the object. Script will stop execution.")

    executionTime = rospy.Time.now() - startTime
    xTime_seconds = executionTime.to_sec()

    print("The execution time for current point cloud collection used ", xTime_seconds, " seconds.")
if __name__=='__main__':
    scan_traj()