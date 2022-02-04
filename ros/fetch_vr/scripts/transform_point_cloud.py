#!/usr/bin/env python

import rospy
import tf2_ros

from sensor_msgs.msg import PointCloud2

from sensor_msgs.point_cloud2 import read_points, create_cloud
import PyKDL

class TransformPointCloud():
    
    def __init__(self):
        self.input_cloud_topic = rospy.get_param("~input_cloud_topic")
        self.source_frame_topic = rospy.get_param("~source_frame_topic")
        self.transform_frame_topic = rospy.get_param("~transform_frame_topic")
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pcd_publisher = rospy.Publisher(
            'base_frame_point_cloud', 
            PointCloud2, 
            queue_size=1
            )
        self.pcd_subscriber = rospy.Subscriber(
            self.input_cloud_topic,
            PointCloud2,
            self.pcd_callback,
            queue_size=1
            )


    def pcd_callback(self, pcd):
        trans = self.tf_buffer.lookup_transform(
            self.transform_frame_topic, 
            self.source_frame_topic, 
            rospy.Time()
            )


        transformed_pcd = self.do_transform_cloud(pcd, trans)

        self.pcd_publisher.publish(transformed_pcd)

    # ---------------------------------------------------------------------- #
    # Copyright (c) 2008, Willow Garage, Inc.
    # All rights reserved.
    # 
    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions are met:
    # 
    #     * Redistributions of source code must retain the above copyright
    #       notice, this list of conditions and the following disclaimer.
    #     * Redistributions in binary form must reproduce the above copyright
    #       notice, this list of conditions and the following disclaimer in the
    #       documentation and/or other materials provided with the distribution.
    #     * Neither the name of the Willow Garage, Inc. nor the names of its
    #       contributors may be used to endorse or promote products derived from
    #       this software without specific prior written permission.
    # 
    # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    # POSSIBILITY OF SUCH DAMAGE.

    @staticmethod
    def transform_to_kdl(t):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                    t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x, 
                                        t.transform.translation.y, 
                                        t.transform.translation.z))

    # PointStamped
    def do_transform_cloud(self, cloud, transform):
        t_kdl = self.transform_to_kdl(transform)
        points_out = []
        for p_in in read_points(cloud):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append((p_out[0], p_out[1], p_out[2]) + p_in[3:])
        res = create_cloud(transform.header, cloud.fields, points_out)
        return res
    tf2_ros.TransformRegistration().add(PointCloud2, do_transform_cloud)
    # ---------------------------------------------------------------------- #

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud')
    transform_point_cloud = TransformPointCloud()
    print("Publishing point cloud in base frame...")
    rospy.spin()