#!/usr/bin/env python

import aruco_msgs.msg
import geometry_msgs.msg
import rospy
import sys
import tf2_ros

EXIT_CODE_SUCCESS = 0
EXIT_CODE_ERROR = 1

def main():
    rospy.init_node("marker_transform_publisher")

    failure_duration = rospy.get_param("~failure_duration", 10.0)
    assert isinstance(failure_duration, float)
    rospy.loginfo("Failure duration: %f" % failure_duration)
    
    marker_ids = rospy.get_param("~marker_ids")
    marker_ids = [int(marker_id) for marker_id in marker_ids.split(",")]
    assert isinstance(marker_ids, list)
    rospy.loginfo("Marker ids: [%s]" % ", ".join(str(marker_id) for marker_id in marker_ids))

    publish_rate = rospy.get_param("~publish_rate", 10)
    assert isinstance(publish_rate, int)
    rospy.loginfo("Publish rate: %d" % publish_rate)

    rospy.loginfo("Waiting for markers...")

    marker_transforms = {marker_id: None for marker_id in marker_ids}
    while not all(marker_transforms.values()):
        try:
            marker_array = rospy.wait_for_message("marker_publisher/markers", aruco_msgs.msg.MarkerArray, timeout=failure_duration)
        except rospy.ROSException:
            rospy.logerr("Timed out!")
            sys.exit(EXIT_CODE_ERROR)

        for marker in marker_array.markers:
            if marker.id in marker_transforms.keys():
                marker_transform = geometry_msgs.msg.TransformStamped()
                marker_transform.header = marker.header
                marker_transform.child_frame_id = "marker_%d" % marker.id
                marker_transform.transform.translation.x = marker.pose.pose.position.x
                marker_transform.transform.translation.y = marker.pose.pose.position.y
                marker_transform.transform.translation.z = marker.pose.pose.position.z
                marker_transform.transform.rotation = marker.pose.pose.orientation
                
                marker_transforms[marker.id] = marker_transform

    rospy.loginfo("Found all makers!")

    rate = rospy.Rate(publish_rate)
    transform_broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        for marker_transform in marker_transforms.values():
            transform_broadcaster.sendTransform(marker_transform)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass
