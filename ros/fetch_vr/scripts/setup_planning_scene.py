#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import copy

from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

class ArucoInit():
    
    def __init__(self):
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # table size: w80cm l180cm h77cm
        self.table_size = [0.82, 1.80, 0.85]
        # obstructions position: 64cm to the right, 42cm to the back
        self.obstruction_side_offset = 0.64
        self.obstruction_back_offset = 0.40

        # self.pcd_subscriber = rospy.Subscriber(
        #     "/aruco_marker_publisher/markers",
        #     aruco_msgs.msg.MarkerArray,
        #     self.add_object_to_scene,
        #     queue_size=1
        #     )

        # self.robot = moveit_commander.RobotCommander()

        self.arm = moveit_commander.MoveGroupCommander("arm")

        # self.gripper = moveit_commander.MoveGroupCommander("gripper")

        self.init_arm = rospy.get_param("~initialise_arm_pose", False)

    def init_moveit(self, aruco_array):
        for marker in aruco_array.markers:
            if marker.id == 201:    # marker on the right
                marker_pose = marker.pose.pose.position
                break
            
        try:
            marker_pose
        except NameError:
            sys.exit("\n\n\nCannot see marker!\n\n\n")
        
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position = marker_pose
        # marker is at top of table, box position needs to be at the centre
        # left marker is 38cm in -x direction 12 in -y direction from the centre of the table
        box_pose.pose.position.x = box_pose.pose.position.x + .33
        box_pose.pose.position.y = box_pose.pose.position.y + .12
        box_pose.pose.position.z = box_pose.pose.position.z - .38 # self.table_size[2]/2
        box_name = "table"
        self.scene.add_box(box_name, box_pose, size=self.table_size)
        
        right_obst_pose = copy.deepcopy(box_pose)
        right_obst_pose.pose.position.y = right_obst_pose.pose.position.y - self.obstruction_side_offset
        self.scene.add_box("right_obstruction", right_obst_pose, size=(2, 0.01, 4))

        left_obst_pose = copy.deepcopy(box_pose)
        left_obst_pose.pose.position.y = left_obst_pose.pose.position.y + self.obstruction_side_offset
        self.scene.add_box("left_obstruction", left_obst_pose, size=(2, 0.01, 4))
        
        back_obst_pose = copy.deepcopy(box_pose)
        back_obst_pose.pose.position.x = back_obst_pose.pose.position.x + self.obstruction_back_offset
        self.scene.add_box("back_obstruction", back_obst_pose, size=(0.01, 2, 4))
        
        print(dir(self.scene))
        # self.scene.apply_planning_scene()

        # while not rospy.is_shutdown():
        #     attached_objects = self.scene.get_attached_objects([box_name])
        #     is_attached = len(attached_objects.keys()) > 0

        #     is_known = box_name in self.scene.get_known_object_names()

        #     if (is_attached) and (is_known):
        #         return
        
        # self.scene.add_box("temp_box", box_pose, size=(self.table_size[0], self.table_size[1], self.table_size[2] + 1))
        # self.scene.remove_world_object("temp_box")

        # print(self.arm.get_joints())
        # print((self.arm.get_current_joint_values()))
        # print(self.gripper.get_joints())
        # print(self.gripper.get_current_joint_values())
        # self.gripper.go([0.0, 0.0], wait=True)
        # self.gripper.stop()

        # start_pose = [-0.6189127329589844, -0.5323255894348145, -0.7583010278918763, -1.276721908996582, -2.165996028260498, -1.2340825009521483, -2.4642663503967284]
        # self.arm.plan(start_pose)
        # if self.init_arm:
        #     self.arm.go(start_pose, wait=True)
        #     self.arm.stop()

#name: [l_wheel_joint, r_wheel_joint, torso_lift_joint, bellows_joint, head_pan_joint, head_tilt_joint, shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint]
#position: [0.2015855312347412, 6.142337322235107, 0.19760264456272125, 0.092, -0.006723165512084961, 0.3862792734008789, -0.6189127329589844, -0.5323255894348145, -0.7583010278918763, -1.276721908996582, -2.165996028260498, -1.2340825009521483, -2.4642663503967284]
#arm joints: ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

#tucked joint states:[1.3211896058319093, 1.400107062084961, -0.19763138222480012, 1.7191431975860596, -1.511132454752942e-05, 1.660155971032715, 7.333383261561369e-05]

if __name__ == "__main__":
    try:
        rospy.init_node("setup_planning_scene")
        moveit_commander.roscpp_initialize(sys.argv)
        aruco_array = rospy.wait_for_message("/marker_publisher/markers", MarkerArray)
        
        box = ArucoInit()
        box.init_moveit(aruco_array)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
