#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
import scipy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromRosToOpen3d, convertCloudFromOpen3dToRos

class Segmentation():

    def __init__(self):
        rospy.init_node('tabletop_segmentation', anonymous=False)

    def point_cloud_listener(self):
        rospy.Subscriber(
            '/head_camera/depth_registered/points',
            PointCloud2,
            self.callback,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.spin()

    def callback(self, pcd_ros):

        pc_frame = pcd_ros.header.frame_id

        pcd = convertCloudFromRosToOpen3d(pcd_ros)

        # Run RANSAC to detect largest plane, assume this is the table
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
        [a, b, c, d] = plane_model
        # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        inlier_cloud = pcd.select_by_index(inliers)         # plane / table
        outlier_cloud = pcd.select_by_index(inliers, invert=True)       # rest of the objects
        
        # Run DBscan on the rest of the point cloud to segment the objects on the table
        labels = np.array(outlier_cloud.cluster_dbscan(eps=0.03, min_points=500))
        max_label = labels.max()

        # Get bounding boxes for each segmented object
        # Initialise jsk BBArray message
        bbox_array = BoundingBoxArray()
        bbox_array.header.frame_id = pc_frame
        bbox_array.header.stamp = rospy.Time.now()

        # Loop through each segmented object
        for i in range(max_label + 1):
            
            # Obtain bounding box for the object
            cluster = o3d.geometry.PointCloud()
            cluster.points = o3d.utility.Vector3dVector(np.asarray(outlier_cloud.points)[np.where(labels==i)])

            bbox = cluster.get_oriented_bounding_box()

            # Construct bounding box message
            bbox_msg = BoundingBox()
            bbox_msg.header.frame_id = pc_frame
            bbox_msg.header.stamp = rospy.Time.now()
            
            quat = scipy.spatial.transform.Rotation.from_matrix(bbox.R).as_quat()
            bbox_msg.pose = Pose(Point(*bbox.center), Quaternion(*quat))

            bbox_msg.dimensions = Vector3(*bbox.extent)

            # Add to array
            bbox_array.boxes.append(bbox_msg)
        
        # print(f"point cloud has {max_label + 1} clusters")

        # Publish segment bounding boxes
        bbox_array_pub = rospy.Publisher(
            'segment_bounding_boxes_array',
            BoundingBoxArray, 
            tcp_nodelay=True, 
            queue_size=10
        )

        bbox_array_pub.publish(bbox_array)

if __name__ == "__main__":

    seg = Segmentation()

    seg.point_cloud_listener()