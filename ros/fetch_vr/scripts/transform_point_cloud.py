#!/usr/bin/env python

import rospy
import tf2_ros

from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class TransformPointCloud():
    
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pcd_publisher = rospy.Publisher(
            'base_point_cloud', 
            PointCloud2, 
            queue_size=1
            )
        self.pcd_subscriber = rospy.Subscriber(
            '/head_camera/depth_registered/points',
            PointCloud2,
            self.pcd_callback,
            queue_size=1
            )


    def pcd_callback(self, pcd):
        trans = self.tf_buffer.lookup_transform(
            'base_link', 
            'head_camera_rgb_optical_frame', 
            rospy.Time()
            )


        transformed_pcd = do_transform_cloud(pcd, trans)

        self.pcd_publisher.publish(transformed_pcd)

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud')
    transform_point_cloud = TransformPointCloud()
    rospy.spin()