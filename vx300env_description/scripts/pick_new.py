import rospy
import std_msgs.msg
import time
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from interbotix_xs_modules.arm import InterbotixManipulatorXS

## Under lagt til fra annen
import sys
import copy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
#from six.moves import input
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from pickle import TRUE

#-----------------Variables -------------------
# 'waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate'
#----- Scanning pose positions (given in radians) -------

sleepy_time = 5
pos_nr = 5
## Home position
home_pos = [0, 0, 0, 0, 0]              # Home
## Scanning positions
#pre_position = x= 
#picking_position = 
#post_position = x=
## Grasp positions
open_pos = [1, 0.057, -0.057]          #           Home/start position
close_pos = [1, 0.021, -0.021]#     1. scanning position From above x = 0.4
#home_pos =    [0.03, -0.03]#       2. scanning position From sleep position x = 0

def grasp():
    bot = InterbotixManipulatorXS("vx300", "arm", "gripper")
    
    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time

    for i in range(pos_nr): #loop for movements
        
        if i==4:
            bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.4, roll=0, pitch=0)
            pos_message = "Move robot to post grasp position."
            print(pos_message)
            time.sleep(1)
        elif i==3:
            bot.gripper.close(2.0)
            #(close_pos)
            pos_message = "Close gripper."
            print(pos_message)
            time.sleep(1)
        elif i==2:
            bot.arm.set_ee_pose_components(x=0.4, y=0, z=0.05, roll=0, pitch=0)
            pos_message = "Move robot to pre grasp position."
            print(pos_message)
            time.sleep(1)
        elif i==1:
            bot.gripper.open(1.0)
            #(open_pos)
            pos_message = "Open gripper."
            print(pos_message)
            time.sleep(1)
        elif i==0:
            bot.arm.set_ee_pose_components(x=0.2, y=0, z=0.2, roll=0, pitch=0)
            pos_message = "Move robot to home before pick patteren initiated."
            print(pos_message)
            time.sleep(1)
        else:
            bot.arm.go_to_home_pose()
            pos_message = "No position available, move to home..."
            print(pos_message)

            
        time.sleep(sleepy_time)
        

       
    executionTime = rospy.Time.now() - startTime
    xTime_seconds = executionTime.to_sec()

    print("The execution time for current picking used ", xTime_seconds, " seconds.")
if __name__=='__main__':
    grasp()