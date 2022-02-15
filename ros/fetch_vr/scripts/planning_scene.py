#!/usr/bin/env python

import geometry_msgs.msg
import moveit_commander
import rospy
import sys
import tf2_ros

def main():
    rospy.init_node("planning_scene")

    marker_frame = "marker_201"
    reference_frame = "base_link"

    one_time = rospy.get_param("~one_time", False)
    rate = rospy.get_param("~rate", 10)
    time_out = rospy.get_param("~time_out", 10.0)

    assert isinstance(one_time, bool)
    assert isinstance(rate, int)
    assert isinstance(time_out, float)

    rospy.loginfo("One time: %s" % one_time)
    rospy.loginfo("Rate: %d" % rate)
    rospy.loginfo("Time out: %f" % time_out)

    transform_buffer = tf2_ros.Buffer()
    transform_listener = tf2_ros.TransformListener(transform_buffer)

    try:
        transform_buffer.lookup_transform(reference_frame, marker_frame, rospy.Time(), rospy.Duration(time_out))
    except:
        rospy.logerr("Failure to find transform!")
        raise
    
    moveit_commander.roscpp_initialize(sys.argv)

    planning_scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    marker_x = -0.140
    marker_y = -0.060
    marker_z = -0.700

    table_depth = 0.800
    table_thickness = 0.030
    table_width = 1.800

    table_pose = geometry_msgs.msg.PoseStamped()
    # table_pose.header.stamp = ...
    table_pose.header.frame_id = marker_frame
    table_pose.position.x = marker_x + table_width / 2
    table_pose.position.y = marker_y + table_depth / 2
    table_pose.position.z = marker_z + table_thickness / 2
    
    table_size = (table_depth, table_thickness, table_width)
    
    planning_scene.add_box("table", table_pose, table_size)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

