#!/usr/bin/env python

import sys

import rospy
import PyKDL
import tf2_ros
import tf_conversions
import moveit_commander

from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray
from dynamixel_controllers.srv import TorqueEnable


def set_gripper(state):
    if state:
        gripper_command_pub.publish(0.0)
    else:
        gripper_command_pub.publish(-1.0)


def move_by_pose(target_pose):
    arm_group.set_pose_target(target_pose)
    arm_group.go(wait=True)


def pick_place(target_pose):
    pass


def object_cb(data):
    for marker in data.markers:
        if marker.ns == "Axis":
            v = PyKDL.Vector(0, 1, 0)
            p_frame = tf_conversions.posemath.fromMsg(marker.pose)
            p_frame.p = p_frame.p * v
            rospy.loginfo("id %s, %s, %s, %s", marker.id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
            rospy.loginfo("id %s, %s, %s, %s", marker.id, p_frame.p.x(), p_frame.p.y(), p_frame.p.z())
            # rospy.loginfo("id %s, %s, %s, %s", marker.id, p_frame.x(), p_frame.y(), p_frame.z())


arm_group = moveit_commander.MoveGroupCommander('cute_arm')
object_marker_sub = rospy.Subscriber('/object_visual_markers', MarkerArray, object_cb)
gripper_command_pub = rospy.Publisher('/claw_controller/command', Float64, queue_size=10)
gripper_torque_service_proxy = rospy.ServiceProxy('/claw_controller/torque_enable', TorqueEnable)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cute_unit_app_node')

    rospy.loginfo("set gripper torque enable")
    gripper_torque_service_proxy(torque_enable=True)

    rospy.loginfo("open gripper")
    set_gripper(False)
    rospy.sleep(2.0)
    set_gripper(True)

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    main()
