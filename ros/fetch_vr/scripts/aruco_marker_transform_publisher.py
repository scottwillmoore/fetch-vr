#!/usr/bin/env python

import aruco_msgs.msg
import geometry_msgs.msg
import rospy
import tf2_ros

if __name__ == "__main__":
    try:
        rospy.init_node("marker_transform_publisher")

        marker_id = rospy.get_param("~marker_id")
        marker_rate = rospy.get_param("~marker_rate")

        marker_transform = None
        while not marker_transform:
            marker_array = rospy.wait_for_message("marker_publisher/markers", aruco_msgs.msg.MarkerArray)
            for marker in marker_array.markers:
                if marker.id in marker_id:
                    marker_transform = geometry_msgs.msg.TransformStamped()
                    marker_transform.header = marker.header
                    marker_transform.child_frame_id = "marker_" + str(marker.id)
                    marker_transform.transform.translation.x = marker.pose.pose.position.x
                    marker_transform.transform.translation.y = marker.pose.pose.position.y
                    marker_transform.transform.translation.z = marker.pose.pose.position.z
                    marker_transform.transform.rotation = marker.pose.pose.orientation

        rate = rospy.Rate(marker_rate)
        transform_broadcaster = tf2_ros.TransformBroadcaster()
        while not rospy.is_shutdown():
            transform_broadcaster.sendTransform(marker_transform)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
