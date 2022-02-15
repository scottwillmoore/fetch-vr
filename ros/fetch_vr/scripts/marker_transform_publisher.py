#!/usr/bin/env python

import aruco_msgs.msg
import geometry_msgs.msg
import rospy
import tf2_ros

def main():
    rospy.init_node("marker_transform_publisher")

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
    marker_transforms = {}
    rate = rospy.Rate(rate)
    transform_broadcaster = tf2_ros.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        if not one_time or first_time:
            first_time = False

            try:
                marker_array = rospy.wait_for_message("marker_publisher/markers", aruco_msgs.msg.MarkerArray, time_out)
            except rospy.ROSException:
                rospy.logerr("Timed out!")
                raise

            for marker in marker_array.markers:
                rospy.loginfo(marker.id)
                transform = geometry_msgs.msg.TransformStamped()
                transform.header = marker.header
                transform.child_frame_id = "marker_%d" % marker.id
                transform.transform.translation.x = marker.pose.pose.position.x
                transform.transform.translation.y = marker.pose.pose.position.y
                transform.transform.translation.z = marker.pose.pose.position.z
                transform.transform.rotation = marker.pose.pose.orientation
                marker_transforms[marker.id] = transform

        for transform in marker_transforms.values():
            transform_broadcaster.sendTransform(transform)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
