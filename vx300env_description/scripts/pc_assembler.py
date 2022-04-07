#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2


def assemble_voxel_pc(): #Assembles point clouds recived from voxal grid filter. Topic subscribed to set in "ur5env.launch".
    rospy.wait_for_service("assemble_scans2")
    assemble_PC = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
    pub = rospy.Publisher('/assembled_pc_scan', PointCloud2, queue_size=2)

    try:
        resp = assemble_PC(rospy.Time(0,0), rospy.get_rostime())
        pub.publish(resp.cloud)
        rospy.loginfo('New cloud added to object point cloud')
    except rospy.serviceException as e:
        rospy.loginfo('Didnt manage to assemble cloud...')
    
    return
