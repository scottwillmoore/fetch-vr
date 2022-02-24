#!/usr/bin/env python

import aruco_msgs.msg
import geometry_msgs.msg
import rospy
import tf2_ros
from tf import transformations as t

class ExternalCamera():
    def __init__(self):
        
        rospy.init_node("camera_transform_publisher")

        one_time = rospy.get_param("~one_time", False)
        rate = rospy.get_param("~rate", 10)
        time_out = rospy.get_param("~time_out", 10.0)

        assert isinstance(one_time, bool)
        assert isinstance(rate, int)
        assert isinstance(time_out, float)

        rospy.loginfo("one_time: %s", one_time)
        rospy.loginfo("Rate: %f" % rate)
        rospy.loginfo("Time out: %d" % time_out)

        first_time = True
        rate = rospy.Rate(rate)
        self.transform_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.marker_subscriber = rospy.Subscriber("external_camera_marker_publisher/markers", aruco_msgs.msg.MarkerArray, self.callback)
    
    def callback(self, marker_array):
        # while not rospy.is_shutdown():
            # # if not one_time or first_time:
            #     first_time = False

            #     try:
            #         marker_array = rospy.wait_for_message("external_camera_marker_publisher/markers", aruco_msgs.msg.MarkerArray, time_out)
            #     except rospy.ROSException:
            #         rospy.logerr("Timed out!")
            #         raise

            for marker in marker_array.markers:
                if marker.id == 200:
                    transform = geometry_msgs.msg.TransformStamped()
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "marker_200"
                    transform.child_frame_id = "camera_link"
                    trans = (marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z)
                    rot = (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)
                    
                    transform_mat = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                    inv_transform_mat = t.inverse_matrix(transform_mat)

                    inv_trans = t.translation_from_matrix(inv_transform_mat)
                    inv_rot = t.quaternion_from_matrix(inv_transform_mat)

                    transform.transform.translation.x = inv_trans[0]
                    transform.transform.translation.y = inv_trans[1]
                    transform.transform.translation.z = inv_trans[2]
                    transform.transform.rotation.x = inv_rot[0]
                    transform.transform.rotation.y = inv_rot[1]
                    transform.transform.rotation.z = inv_rot[2]
                    transform.transform.rotation.w = inv_rot[3]
            try:
                self.transform_broadcaster.sendTransform(transform)
                print("Published transform from marker to external camera\n")
            except:
                print("Marker not yet detected by camera\n")

            # self.rate.sleep()

if __name__ == "__main__":
    try:
        ExternalCamera()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
