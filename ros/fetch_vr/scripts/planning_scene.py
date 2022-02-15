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

    marker_depth = 0.120
    marker_separation = 0.160
    marker_width = 0.120

    table_depth = 0.800
    table_thickness = 0.030
    table_width = 1.800

    world_x = -((marker_width / 2) + (marker_separation / 2))
    world_y = -(marker_depth / 2)

    table_x = world_x
    table_y = world_y + (table_depth / 2)
    table_z = -(table_thickness / 2)
    
    table_pose = geometry_msgs.msg.PoseStamped()
    # table_pose.header.stamp = ...
    table_pose.header.frame_id = marker_frame
    table_pose.pose.position.x = table_x
    table_pose.pose.position.y = table_y
    table_pose.pose.position.z = table_z
    # table_pose.pose.orientation = ...
    
    table_size = (table_width, table_depth, table_thickness)
    
    planning_scene.add_box("table", table_pose, table_size)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

