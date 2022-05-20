#!/usr/bin/env python
import tf
import rospy
import std_msgs
import numpy as np
from roslib import message
from tf import transformations
import sensor_msgs.point_cloud2 as p_c2
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2


class Tf_camera_PC():

    def __init__(self):

        self.pc_pub = rospy.Publisher('/camera_PC_transposed', PointCloud2, queue_size=1)
        self.pc_sub = rospy.Subscriber('/pc_filter/pointcloud/objects', PointCloud2, self.pc_callback, queue_size=1) #Topics= Simulation: /camera/depth/color/points , physical: /pc_filter/pointcloud/objects

    def pc_callback(self,pc_data):
        assert isinstance(pc_data, PointCloud2)
        listener = tf.TransformListener()
        
        pc = list([x for x in pc2.read_points(pc_data, skip_nans=True, field_names=["x", "y", "z"])]) #Get points from gathered topic

        listener.waitForTransform("world", "/camera_depth_optical_frame", rospy.Time(), rospy.Duration(4.0))

        (trans, quat) = listener.lookupTransform("world", "/camera_depth_optical_frame", rospy.Time(0)) #Get transformation from camera frame to base frame

        Tra = transformations.translation_matrix(trans)
        Rot = transformations.quaternion_matrix(quat)
        RT = Rot + Tra - np.identity(4)
        
        points_transposed = np.zeros((len(pc),3)) #Create empthy list to store transposed points

        for i, point in enumerate(pc):
            point_setup = np.array((point[0],point[1],point[2], 1))
            point_new = np.matmul(RT, point_setup)
            points_transposed[i] = np.array([point_new[0], point_new[1], point_new[2]])

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        PointCloud_transposed = p_c2.create_cloud_xyz32(header, points_transposed)

        rospy.loginfo("Point Cloud are transposed. Ready for publication to topic.")
        self.pc_pub.publish(PointCloud_transposed)
        self.pc_sub.unregister() #Stops subscription for not continue publishing several points

if __name__ == '__main__':

    rospy.init_node('camera_PC_transform', anonymous=True)
    tf_pc_cam = Tf_camera_PC()
    rospy.spin()