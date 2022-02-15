#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

EXIT_CODE_SUCCESS = 0
EXIT_CODE_ERROR = 1

DEFAULT_RATE = 1.0
def main():
    rospy.init_node("world_publisher")

    world_transform = geometry_msgs.msg.TransformStamped()
    world_transform.header = marker.header
    world_transform.child_frame_id = "marker_%d" % marker.id
    world_transform.transform.translation.x = marker.pose.pose.position.x
    world_transform.transform.translation.y = marker.pose.pose.position.y
    world_transform.transform.translation.z = marker.pose.pose.position.z
    world_transform.transform.rotation = marker.pose.pose.orientation

    rate = rospy.Rate(DEFAULT_RATE)
    transform_broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        transform_broadcaster.sendTransform(world_transform)
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
