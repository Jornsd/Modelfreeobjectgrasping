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

def scan_traj():
    bot = InterbotixManipulatorXS("vx300", "arm", "gripper")
    arm = InterbotixManipulatorXS("vx300", "arm", "gripper")
    
    take_picture = False #If position are reach for taking picture or returning to home.

    print(rospy.Time.now()) #Initiate time to get update for first position. Needs interaction/use to collect correct time
    time.sleep(1)#give one second to be able to update clock/time until next time initiated, if not aren't the first position initiated
    startTime = rospy.Time.now()#used for calculation of execution time

    for i in range(pos_nr): #loop for movements
        
        if i==6:
            bot.arm.set_ee_pose_components(x=0.3, y=-0.15, z=0.4, roll=0, pitch=1.5)
            pos_message = "Move robot to fifth position."
            time.sleep(2)
            take_picture = True
        elif i==5:
            bot.arm.set_ee_pose_components(x=0.3, y=0.116, z=0.4, roll=0, pitch=1.5)
            pos_message = "Move robot to fourth position."
            time.sleep(2)
            take_picture = True
        elif i==4:
            bot.arm.set_ee_pose_components(x=0.45, y=0, z=0.4, roll=0, pitch=1.8)
            pos_message = "Move robot to third position."
            time.sleep(2)
            take_picture = True
        elif i==3:
            bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.4, roll=0, pitch=1.5)
            pos_message = "Move robot to second position."
            time.sleep(2)
            take_picture = True
        elif i==2:
            bot.arm.set_ee_pose_components(x=0.15, y=0, z=0.4, roll=0, pitch=1.2)
            pos_message = "Move robot to first position."
            time.sleep(2)
            take_picture = True
        elif i==1:
            bot.arm.set_ee_pose_components(x=0.15, y=0, z=0.4, roll=0, pitch=1.2)
            pos_message = "Move robot to first position."
            time.sleep(2)
            take_picture = True
        elif i==0:
            bot.arm.go_to_home_pose()
            pos_message = "Move robot to home before scan patteren initiated."
            time.sleep(2)
            take_picture = False
        else:
            bot.arm.go_to_sleep_pose()
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