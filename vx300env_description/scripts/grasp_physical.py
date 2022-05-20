from posixpath import split
from turtle import position
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np


import math
import rospy
import numpy as np
import modern_robotics as mr
from interbotix_xs_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from interbotix_common_modules import angle_manipulation as ang
from interbotix_xs_modules import mr_descriptions as mrd
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_xs_modules.gripper import InterbotixGripperXSInterface

# This script makes the end-effector go to a specific pose by defining the pose components
#
# To get started, open a terminal and type 'roslaunch vx300env_description vx300envreal.launch'
# Then change to this directory and type 'python grasp_physical.py'





def rotation_angles(matrix, order):
    """
    input
        matrix = 3x3 rotation matrix (numpy array)
        oreder(str) = rotation order of x, y, z : e.g, rotation XZY -- 'xzy'
    output
        theta1, theta2, theta3 = rotation angles in rotation order
    """
    r11, r12, r13 = matrix[0]
    r21, r22, r23 = matrix[1]
    r31, r32, r33 = matrix[2]

    if order == 'xzx':
        theta1 = np.arctan(r31 / r21)
        theta2 = np.arctan(r21 / (r11 * np.cos(theta1)))
        theta3 = np.arctan(-r13 / r12)

    elif order == 'xyx':
        theta1 = np.arctan(-r21 / r31)
        theta2 = np.arctan(-r31 / (r11 *np.cos(theta1)))
        theta3 = np.arctan(r12 / r13)

    elif order == 'yxy':
        theta1 = np.arctan(r12 / r32)
        theta2 = np.arctan(r32 / (r22 *np.cos(theta1)))
        theta3 = np.arctan(-r21 / r23)

    elif order == 'yzy':
        theta1 = np.arctan(-r32 / r12)
        theta2 = np.arctan(-r12 / (r22 *np.cos(theta1)))
        theta3 = np.arctan(r23 / r21)

    elif order == 'zyz':
        theta1 = np.arctan(r23 / r13)
        theta2 = np.arctan(r13 / (r33 *np.cos(theta1)))
        theta3 = np.arctan(-r32 / r31)

    elif order == 'zxz':
        theta1 = np.arctan(-r13 / r23)
        theta2 = np.arctan(-r23 / (r33 *np.cos(theta1)))
        theta3 = np.arctan(r31 / r32)

    elif order == 'xzy':
        theta1 = np.arctan(r32 / r22)
        theta2 = np.arctan(-r12 * np.cos(theta1) / r22)
        theta3 = np.arctan(r13 / r11)

    elif order == 'xyz':
        theta1 = np.arctan(-r23 / r33)
        theta2 = np.arctan(r13 * np.cos(theta1) / r33)
        theta3 = np.arctan(-r12 / r11)

    elif order == 'yxz':
        theta1 = np.arctan(r13 / r33)
        theta2 = np.arctan(-r23 * np.cos(theta1) / r33)
        theta3 = np.arctan(r21 / r22)

    elif order == 'yzx':
        theta1 = np.arctan(-r31 / r11)
        theta2 = np.arctan(r21 * np.cos(theta1) / r11)
        theta3 = np.arctan(-r23 / r22)

    elif order == 'zyx':
        theta1 = np.arctan(r21 / r11)
        theta2 = np.arctan(-r31 * np.cos(theta1) / r11)
        theta3 = np.arctan(r32 / r33)

    elif order == 'zxy':
        theta1 = np.arctan(-r12 / r22)
        theta2 = np.arctan(r32 * np.cos(theta1) / r22)
        theta3 = np.arctan(-r31 / r33)

    #theta1 = theta1 * 180 / np.pi
    #theta2 = theta2 * 180 / np.pi
    #theta3 = theta3 * 180 / np.pi

    return (theta1, theta2, theta3)




def main():
    
    #skraakube
    #pre_numbers_str = "-0.528431 -0.142381 0.836952 0 0.297346 0.892354 0.339543 0 -0.795202 0.428289 -0.429211 0 0.280638 -0.0116398 0.201241 1"

    #grasp_numbers_str = "-0.528431 -0.142381 0.836952 0 0.297346 0.892354 0.339543 0 -0.795202 0.428289 -0.429211 0 0.333481 0.00259828 0.117546 1"    

    #post_numbers_str = " -0.528431 -0.142381 0.836952 0 0.297346 0.892354 0.339543 0 -0.795202 0.428289 -0.429211 0 0.333481 0.00259828 0.317546 1"

    #kube
    #pre_numbers_str = " -0.746317 0.0691528 0.661988 0 0.18843 0.975851 0.110494 0 -0.638361 0.207202 -0.741325 0 0.238953 0.0186566 0.163656 1"

    #grasp_numbers_str = " -0.746317 0.0691528 0.661988 0 0.18843 0.975851 0.110494 0 -0.638361 0.207202 -0.741325 0 0.313585 0.0117413 0.0974574 1"    

    #post_numbers_str = " -0.746317 0.0691528 0.661988 0 0.18843 0.975851 0.110494 0 -0.638361 0.207202 -0.741325 0 0.313585 0.0117413 0.297457 1"

    #trekant, med pre.grasp til kube
    #pre_numbers_str = " -0.746317 0.0691528 0.661988 0 0.18843 0.975851 0.110494 0 -0.638361 0.207202 -0.741325 0 0.238953 0.0186566 0.163656 1"

    #grasp_numbers_str = " -0.996709 -0.0161948 0.0794272 0 0.0173489 -0.999753 0.0138615 0 0.0791832 0.0151939 0.996744 0 0.288598 -0.0034411 0.0513391 1"    

    #post_numbers_str = "  -0.996709 -0.0161948 0.0794272 0 0.0173489 -0.999753 0.0138615 0 0.0791832 0.0151939 0.996744 0 0.288598 -0.0034411 0.251339 1"

    #flatSylinder
    #pre_numbers_str = " 0.987702 -0.108234 -0.112824 0 0.110462 -0.993786 -0.0136669 0 -0.110644 -0.0259616 0.993521 0 0.18987 -0.0206775 0.0616507 1"

    #grasp_numbers_str = " -0.987702 -0.108234 -0.112824 0 0.110462 -0.993786 -0.0136669 0 -0.110644 -0.0259616 0.993521 0 0.28864 -0.00985415 0.0729331 1"    

    #post_numbers_str = "  -0.987702 -0.108234 -0.112824 0 0.110462 -0.993786 -0.0136669 0 -0.110644 -0.0259616 0.993521 0 0.28864 -0.00985415 0.272933 1"

    #halvsirkel - Fungerer ikke skikkelig ps: har satt y = 0 i y retning ac pre og grasp
    #pre_numbers_str = " -0.606504 0.43901 0.66289 0 -0.207862 0.717192 -0.665154 0 -0.767428 -0.541208 -0.343726 0 0.283431 0.0 0.162637 1"

    #grasp_numbers_str = "  -0.606504 0.43901 0.66289 0 -0.207862 0.717192 -0.665154 0 -0.767428 -0.541208 -0.343726 0 0.344081 0.0 0.0963481 1"    

    #post_numbers_str = "  -0.606504 0.43901 0.66289 0 -0.207862 0.717192 -0.665154 0 -0.767428 -0.541208 -0.343726 0 0.344081 0.0606602 0.296348 1"

    #tape with cube's pre-grasp, working!
    #pre_numbers_str = " -0.746317 0.0691528 0.661988 0 0.18843 0.975851 0.110494 0 -0.638361 0.207202 -0.741325 0 0.238953 0.0186566 0.163656 1"

    #grasp_numbers_str = " -0.996221 -0.0805244 0.0325568 0 0.0675949 -0.954153 -0.291588 0 0.0545441 -0.288286 0.95599 0 0.303635 -0.0392346 0.0512467 1"

    #post_numbers_str = " -0.996221 -0.0805244 0.0325568 0 0.0675949 -0.954153 -0.291588 0 0.0545441 -0.288286 0.95599 0 0.303635 -0.0392346 0.251247 1"

    #Rectangle, working!
    #pre_numbers_str = " -0.746317 0.0691528 0.661988 0 0.18843 0.975851 0.110494 0 -0.638361 0.207202 -0.741325 0 0.238953 0.0186566 0.163656 1"

    #grasp_numbers_str = "-0.989476 -0.0318476 0.141147 0 -0.0041899 0.981374 0.192059 0 -0.144635 0.189446 -0.97118 0 0.321836 -0.0128691 0.0508702 1"

    #post_numbers_str = " -0.989476 -0.0318476 0.141147 0 -0.0041899 0.981374 0.192059 0 -0.144635 0.189446 -0.97118 0 0.321836 -0.0128691 0.25087 1"



    #Cube video


   # pre_numbers_str = "  -0.993118  0.115852 0.0171965         0 -0.113416 -0.987917  0.105627         0 0.0292258   0.10295  0.994257         0   0.17977 0.0208068 0.0431561         1"
 
   # grasp_numbers_str = "   -0.993118   0.115852  0.0171965          0  -0.113416  -0.987917   0.105627          0  0.0292258    0.10295   0.994257          0   0.279081 0.00922168  0.0414364          1"

   # post_numbers_str = " -0.993118   0.115852  0.0171965          0  -0.113416  -0.987917   0.105627          0  0.0292258    0.10295   0.994257          0   0.279081 0.00922168   0.241436          1 "



   






    #Cube big 6mm
    #grasp_numbers_str = " -0.998769 -0.0420025 -0.0263759 0 0.00964518 -0.686135 0.72741 0 -0.0486504 0.72626 0.685696 0 0.271498 -0.00300148 0.0464619 1"

    #post_numbers_str = " -0.998769 -0.0420025 -0.0263759 0 0.00964518 -0.686135 0.72741 0 -0.0486504 0.72626 0.685696 0 0.271498 -0.00300148 0.246462 1"

    #cube big 6mm
    # dont use this pre_numbers_str = " -0.935182 -0.0649461 0.348163 0 0.0358385 -0.99535 -0.0894081 0 0.352351 -0.0711352 0.933161 0 0.183646 -0.0257903 0.114516 1"

    #grasp_numbers_str = " -0.99985 0.00987005 0.0141978 0 -0.0104595 -0.99906 -0.0420613 0 0.0137693 -0.0422035 0.999014 0 0.272039 -0.0111163 0.0469101 1"

    #post_numbers_str = " -0.99985 0.00987005 0.0141978 0 -0.0104595 -0.99906 -0.0420613 0 0.0137693 -0.0422035 0.999014 0 0.272039 -0.0111163 0.24691 1"

    #cube big 4mm
    #grasp_numbers_str = " -0.999759 -0.00841445 0.02028 0 0.00261986 0.871334 0.490683 0 -0.0217995 0.490618 -0.871102 0 0.274751 -0.0176693 0.0464074 1"

    #Cube 10mm
   # pre_numbers_str = " -0.865433 -0.182043 0.466782 0 0.0411618 -0.95434 -0.295872 0 0.49933 -0.236844 0.833411 0 0.198559 -0.0535289 0.141157 1"

   # grasp_numbers_str = " -0.865433 -0.182043 0.466782 0 0.0411618 -0.95434 -0.295872 0 0.49933 -0.236844 0.833411 0 0.285102 -0.0353246 0.0944785 1"

   # post_numbers_str = " -0.865433 -0.182043 0.466782 0 0.0411618 -0.95434 -0.295872 0 0.49933 -0.236844 0.833411 0 0.285102 -0.0353246 0.294478 1"

    #Sylinder 8mm
    #pre_numbers_str = " -0.920133 -0.0595095 0.387058 0 -0.0427777 -0.967197 -0.250399 0 0.389262 -0.246957 0.887405 0 0.19887 -0.0103296 0.137425 1"

    #grasp_numbers_str = " -0.920133 -0.0595095 0.387058 0 -0.0427777 -0.967197 -0.250399 0 0.389262 -0.246957 0.887405 0 0.290883 -0.00437861 0.0987192 1"

    #post_numbers_str = " -0.865433 -0.182043 0.466782 0 0.0411618 -0.95434 -0.295872 0 0.49933 -0.236844 0.833411 0 0.285102 -0.0353246 0.294478 1"

    #Trekant 8mm
    #pre_numbers_str = " -0.978334 -0.0168719 0.206343 0 -0.045078 -0.955402 -0.291848 0 0.202065 -0.294826 0.933942 0 0.176836 -0.0214904 0.0805866 1"

    #grasp_numbers_str = " -0.978334 -0.0168719 0.206343 0 -0.045078 -0.955402 -0.291848 0 0.202065 -0.294826 0.933942 0 0.274669 -0.0198032 0.0599523 1"

    #post_numbers_str = " 0.654912 0.456424 0.602302 0 0.750545 -0.485831 -0.44794 0 0.0881666 0.745416 -0.660743 0 0.42764 0.0410954 0.302685 1"

    #Kule 8mm
    #pre_numbers_str = " -0.664243 0.110048 0.739372 0 0.321885 -0.850598 0.415781 0 0.674664 0.514172 0.529581 0 0.231782 0.00655019 0.209168 1"

    #grasp_numbers_str = " -0.664243 0.110048 0.739372 0 0.321885 -0.850598 0.415781 0 0.674664 0.514172 0.529581 0 0.298206 -0.00445465 0.135231 1  "

    #post_numbers_str = " -0.664243 0.110048 0.739372 0 0.321885 -0.850598 0.415781 0 0.674664 0.514172 0.529581 0 0.298206 -0.00445465 0.335231 1"

    #Ola's trekant
    #pre_numbers_str = " -0.984792 -0.143941 0.0972934 0 0.0997125 -0.926848 -0.361953 0 0.142276 -0.346747 0.927105 0 0.176774 -0.0496153 0.0661024 1"

    #grasp_numbers_str = " -0.984792 -0.143941 0.0972934 0 0.0997125 -0.926848 -0.361953 0 0.142276 -0.346747 0.927105 0 0.275253 -0.0352212 0.0563731 1"

    #post_numbers_str = " -0.664243 0.110048 0.739372 0 0.321885 -0.850598 0.415781 0 0.674664 0.514172 0.529581 0 0.298206 -0.00445465 0.335231 1"


    #EXPERIMENT
    #pre_numbers_str = " -0.661589 -0.482214 0.574255 0 0.417193 0.399644 0.816232 0 -0.623096 0.779586 -0.0632238 0 0.239938 -0.149877 0.190735 1"

    #grasp_numbers_str = " -0.661589 -0.482214 0.574255 0 0.417193 0.399644 0.816232 0 -0.623096 0.779586 -0.0632238 0 0.306097 -0.101656 0.13331 1"

    #post_numbers_str = " -0.661589 -0.482214 0.574255 0 0.417193 0.399644 0.816232 0 -0.623096 0.779586 -0.0632238 0 0.306097 -0.101656 0.33331 1"


    #Cube big 8mm
    pre_numbers_str = " -0.974023 -0.00660199 0.226355 0 -0.00600667 0.999976 0.00331872 0 -0.226371 0.00187287 -0.974039 0 0.177731 -0.013725 0.0868068 1"

    grasp_numbers_str = " -0.974023 -0.00660199 0.226355 0 -0.00600667 0.999976 0.00331872 0 -0.226371 0.00187287 -0.974039 0 0.275134 -0.0130648 0.0641714 1"

    post_numbers_str = " -0.974023 -0.00660199 0.226355 0 -0.00600667 0.999976 0.00331872 0 -0.226371 0.00187287 -0.974039 0 0.275134 -0.0130648 0.264171 1"



    pre_numbers_list = [float(num) for num in pre_numbers_str.split()]

    
    pre_T_sd = np.identity(4)
    pre_T_sd[0,3] = pre_numbers_list[12]
    pre_T_sd[1,3] = pre_numbers_list[13]
    pre_T_sd[2,3] = pre_numbers_list[14] 
    pre_T_sd[0,2] = pre_numbers_list[8]
    pre_T_sd[1,2] = pre_numbers_list[9]
    pre_T_sd[2,2] = pre_numbers_list[10]
    pre_T_sd[0,1] = pre_numbers_list[4]
    pre_T_sd[1,1] = pre_numbers_list[5]
    pre_T_sd[2,1] = pre_numbers_list[6]
    pre_T_sd[0,0] = pre_numbers_list[0]
    pre_T_sd[1,0] = pre_numbers_list[1]
    pre_T_sd[2,0] = pre_numbers_list[2]
    print(pre_T_sd)
    
    
    
    pre_E_sd = np.identity(3)
    pre_E_sd[0,2] = pre_T_sd[0,2]
    pre_E_sd[1,2] = pre_T_sd[1,2]
    pre_E_sd[2,2] = pre_T_sd[2,2]
    pre_E_sd[0,1] = pre_T_sd[0,1]
    pre_E_sd[1,1] = pre_T_sd[1,1]
    pre_E_sd[2,1] = pre_T_sd[2,1]
    pre_E_sd[0,0] = pre_T_sd[0,0]
    pre_E_sd[1,0] = pre_T_sd[1,0]
    pre_E_sd[2,0] = pre_T_sd[2,0]
    print(pre_E_sd)
    
    
    pre_angles = rotation_angles(pre_E_sd, 'xyz')
    print(pre_angles)
    

    grasp_numbers_list = [float(num) for num in grasp_numbers_str.split()]

    
    grasp_T_sd = np.identity(4)
    grasp_T_sd[0,3] = grasp_numbers_list[12] 
    grasp_T_sd[1,3] = grasp_numbers_list[13]
    grasp_T_sd[2,3] = grasp_numbers_list[14] 
    grasp_T_sd[0,2] = grasp_numbers_list[8]
    grasp_T_sd[1,2] = grasp_numbers_list[9]
    grasp_T_sd[2,2] = grasp_numbers_list[10]
    grasp_T_sd[0,1] = grasp_numbers_list[4]
    grasp_T_sd[1,1] = grasp_numbers_list[5]
    grasp_T_sd[2,1] = grasp_numbers_list[6]
    grasp_T_sd[0,0] = grasp_numbers_list[0]
    grasp_T_sd[1,0] = grasp_numbers_list[1]
    grasp_T_sd[2,0] = grasp_numbers_list[2]
    print(grasp_T_sd)
    
    
    
    grasp_E_sd = np.identity(3)
    grasp_E_sd[0,2] = grasp_T_sd[0,2]
    grasp_E_sd[1,2] = grasp_T_sd[1,2]
    grasp_E_sd[2,2] = grasp_T_sd[2,2]
    grasp_E_sd[0,1] = grasp_T_sd[0,1]
    grasp_E_sd[1,1] = grasp_T_sd[1,1]
    grasp_E_sd[2,1] = grasp_T_sd[2,1]
    grasp_E_sd[0,0] = grasp_T_sd[0,0]
    grasp_E_sd[1,0] = grasp_T_sd[1,0]
    grasp_E_sd[2,0] = grasp_T_sd[2,0]
    print(grasp_E_sd)
    
    
    grasp_angles = rotation_angles(grasp_E_sd, 'xyz')
    print(grasp_angles)
    
    
    
    post_numbers_list = [float(num) for num in post_numbers_str.split()]

    
    post_T_sd = np.identity(4)
    post_T_sd[0,3] = post_numbers_list[12] 
    post_T_sd[1,3] = post_numbers_list[13]
    post_T_sd[2,3] = post_numbers_list[14] 
    post_T_sd[0,2] = post_numbers_list[8]
    post_T_sd[1,2] = post_numbers_list[9]
    post_T_sd[2,2] = post_numbers_list[10]
    post_T_sd[0,1] = post_numbers_list[4]
    post_T_sd[1,1] = post_numbers_list[5]
    post_T_sd[2,1] = post_numbers_list[6]
    post_T_sd[0,0] = post_numbers_list[0]
    post_T_sd[1,0] = post_numbers_list[1]
    post_T_sd[2,0] = post_numbers_list[2]
    print(post_T_sd)
    
    
    
    post_E_sd = np.identity(3)
    post_E_sd[0,2] = post_T_sd[0,2] 
    post_E_sd[1,2] = post_T_sd[1,2]
    post_E_sd[2,2] = post_T_sd[2,2]
    post_E_sd[0,1] = post_T_sd[0,1]
    post_E_sd[1,1] = post_T_sd[1,1]
    post_E_sd[2,1] = post_T_sd[2,1]
    post_E_sd[0,0] = post_T_sd[0,0]
    post_E_sd[1,0] = post_T_sd[1,0]
    post_E_sd[2,0] = post_T_sd[2,0]
    print(post_E_sd)
    
    
    post_angles = rotation_angles(post_E_sd, 'xyz')
    print(post_angles)
    


    first_pos = [0, -1, 1, 0, 0]
    




    
    bot = InterbotixManipulatorXS("vx300", "arm", "gripper")
    arm = InterbotixManipulatorXS("vx300", "arm", "gripper")
    bot.arm.go_to_home_pose()
    bot.arm.set_joint_positions(first_pos)
    arm.gripper.open(2.0)
    bot.arm.set_ee_pose_components(x=pre_T_sd[0,3], y=pre_T_sd[1,3], z=pre_T_sd[2,3], roll=pre_angles[0], pitch=pre_angles[1])
    bot.arm.set_ee_pose_components(x=grasp_T_sd[0,3], y=grasp_T_sd[1,3], z=grasp_T_sd[2,3], roll=grasp_angles[0], pitch=grasp_angles[1])
    arm.gripper.close(2.0)
    #arm.gripper.set_pressure(1.0)
    bot.arm.set_ee_pose_components(x=post_T_sd[0,3], y=post_T_sd[1,3], z=post_T_sd[2,3], roll=post_angles[0], pitch=post_angles[1])
    bot.arm.set_ee_pose_components(x=0.36, y=0.3, z=0.25, roll=0, pitch=1.5)
    arm.gripper.open(2.0)
    
    #bot.arm.go_to_home_pose()
    
    bot.arm.go_to_sleep_pose()
    
    
    

    
    
    
    

if __name__=='__main__':
    main()