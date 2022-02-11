#!/usr/bin/env python3

from sklearn.utils import compute_class_weight
import rospy
import open3d as o3d
import numpy as np
import scipy
import time

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromRosToOpen3d, convertCloudFromOpen3dToRos
from open3d_ros_helper import rospc_to_o3dpc, o3dpc_to_rospc

class Segmentation():

    def __init__(self):
        self.input_cloud_topic = rospy.get_param('~input_cloud_topic')
        self.compute_bounding_boxes = rospy.get_param('~compute_bounding_boxes')

        self.point_cloud_listener = rospy.Subscriber(
            self.input_cloud_topic,
            PointCloud2,
            self.callback,
            queue_size=1,
            tcp_nodelay=True
        )
        self.point_cloud_publisher = rospy.Publisher(
            'segmentation/objects_point_cloud', 
            PointCloud2,
            tcp_nodelay=True,
            queue_size=1
        )
        self.bounding_box_publisher = rospy.Publisher(
            'segmentation/bounding_boxes',
            BoundingBoxArray, 
            tcp_nodelay=True, 
            queue_size=1
        )

        

    def callback(self, pcd_ros):

        start_time = time.time()

        pcd_frame = pcd_ros.header.frame_id
        # print(pcd_frame)

        pcd = rospc_to_o3dpc(pcd_ros)
        print('src:', pcd)

        # o3d.io.write_point_cloud("raw.pcd", pcd, print_progress=True)

        # Remove points further away than this threshold
        distance_threshold = 1.2

        #norms = np.linalg.norm(np.asarray(pcd.points), axis=np.argmin(np.asarray(pcd.points).shape))
        dist = np.asarray(pcd.points)[:, 0]

        pcd = pcd.select_by_index([i for i in range(len(dist)) if dist[i] < distance_threshold])
        print('remove far points:', pcd)

        # Remove points lower than this threshold
        height_threshold = 0.8

        heights = np.asarray(pcd.points)[:, 2]

        pcd = pcd.select_by_index([i for i in range(len(heights)) if heights[i] > height_threshold])
        print('remove low points:', pcd)

        # Remove planes with more points than this threshold
        plane_threshold = 10000

        for i in range(5):
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=50)
            plane_cloud = pcd.select_by_index(inliers)         # plane
            plane_size = len(plane_cloud.points)

            if plane_size < plane_threshold:
                continue
            
            
            pcd = pcd.select_by_index(inliers, invert=True)       # rest of the objects

            # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
            print(f"Removed plane with {plane_size} points")
        
        # Clean up point cloud
        # pcd, ind = pcd.remove_radius_outlier(nb_points=500, radius=0.04)
        print('remove radius outliers:', pcd)
        
        if self.compute_bounding_boxes:
            # Run DBscan on the rest of the point cloud to segment the objects on the table
            labels = np.array(pcd.cluster_dbscan(eps=0.04, min_points=500))
            max_label = labels.max()
            print(f"{max_label + 1} objects detected in point cloud")

            # Get bounding boxes for each segmented object
            # Initialise jsk BBArray message
            bbox_array = BoundingBoxArray()
            bbox_array.header.frame_id = pcd_frame
            bbox_array.header.stamp = rospy.Time.now()

            # Loop through each segmented object
            for i in range(max_label + 1):
                
                # Obtain bounding box for the object
                cluster = o3d.geometry.PointCloud()
                cluster.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[np.where(labels==i)])

                bbox = cluster.get_oriented_bounding_box()

                # Construct bounding box message
                bbox_msg = BoundingBox()
                bbox_msg.header.frame_id = pcd_frame
                bbox_msg.header.stamp = rospy.Time.now()
                
                # open3d rotation matrices can have -1 det, improper roation with reflections
                # scipy only works with +1 det, will convert the matrix
                # inverting each element will change det from -1 to +1
                if np.linalg.det(bbox.R) > 0:
                    quat = scipy.spatial.transform.Rotation.from_matrix(bbox.R).as_quat()
                else:
                    quat = scipy.spatial.transform.Rotation.from_matrix(-bbox.R).as_quat()
                
                bbox_msg.pose = Pose(Point(*bbox.center), Quaternion(*quat))

                bbox_msg.dimensions = Vector3(*bbox.extent)

                # Add to array
                bbox_array.boxes.append(bbox_msg)
            
            # print(f"point cloud has {max_label + 1} clusters")

            # Publish segment bounding boxes
            self.bounding_box_publisher.publish(bbox_array)
            print("Publishing bounding boxes...")

        # Publish segmented point cloud
        pcd_msg = o3dpc_to_rospc(pcd, frame_id=pcd_frame)

        self.point_cloud_publisher.publish(pcd_msg)
        print("Publishing objects point cloud...")

        print(f"Time taken: {time.time() - start_time}")

if __name__ == "__main__":

    rospy.init_node('tabletop_segmentation', anonymous=False)

    seg = Segmentation()

    rospy.spin()
