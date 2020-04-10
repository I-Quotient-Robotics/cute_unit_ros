#!/usr/bin/env python

import sys

import rospy
import math3d
import moveit_commander

import geometry_msgs.msg
from std_msgs.msg import Float64
from dynamixel_controllers.srv import TorqueEnable


class PickPlace:
    def __init__(self):
        self.arm_group = moveit_commander.MoveGroupCommander('cute_arm')
        self.gripper_command_pub = rospy.Publisher('/claw_controller/command', Float64, queue_size=10)
        self.gripper_torque_service_proxy = rospy.ServiceProxy('/claw_controller/torque_enable', TorqueEnable)
        rospy.loginfo("arm setup")
        self.gripper_torque_service_proxy(torque_enable=True)
        self.__set_gripper(False)
        rospy.sleep(1.0)
        self.__set_gripper(True)
        rospy.sleep(1.0)
        self.__move_by_name('home')
        rospy.sleep(1.0)

        self.object_pose_sub = rospy.Subscriber('/object_pose', geometry_msgs.msg.PoseArray, self.__object_cb)

        self.__object_pose = None

    def __convert_to_transform(self, pose):
        quaternion = math3d.UnitQuaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        transform = math3d.Transform()
        transform.set_pos((pose.position.x, pose.position.y, pose.position.z))
        transform.set_orient(quaternion.orientation)
        return transform

    def __convert_to_pose(self, t):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = t.pos.x
        pose.pose.position.y = t.pos.y
        pose.pose.position.z = t.pos.z
        pose.pose.orientation.w = t.orient.quaternion[0]
        pose.pose.orientation.x = t.orient.quaternion[1]
        pose.pose.orientation.y = t.orient.quaternion[2]
        pose.pose.orientation.z = t.orient.quaternion[3]
        return pose

    def __set_gripper(self, state):
        if state:
            self.gripper_command_pub.publish(0.0)
        else:
            self.gripper_command_pub.publish(-1.0)

    def __move_by_pose(self, target_pose):
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True)

    def __move_by_name(self, target_name):
        self.arm_group.set_named_target(target_name)
        self.arm_group.go(wait=True)

    def pick_place(self, target_pose):
        pass

    def __object_cb(self, data):
        # rospy.loginfo("get data")
        for pose in data.poses:
            self.__object_pose = pose
            self.__object_frame_id = data.header.frame_id

    def run(self):
        if self.__object_pose is not None:
            rospy.loginfo("has object")
            current_pose = self.__object_pose
            object_pose_stamped = geometry_msgs.msg.PoseStamped()
            object_pose_stamped.header.frame_id = self.__object_frame_id
            object_pose_stamped.pose = current_pose

            object_transform = self.__convert_to_transform(current_pose)

            standby_offset = math3d.Transform()
            standby_offset.pos = math3d.Vector(0, 0, -0.03)
            standby_transform = object_transform * standby_offset
            standby_pose_stamped = self.__convert_to_pose(standby_transform)
            standby_pose_stamped.header.frame_id = self.__object_frame_id

            pick_offset = math3d.Transform()
            pick_offset.pos = math3d.Vector(0, 0, 0)
            pick_transform = object_transform * pick_offset
            pick_pose_stamped = self.__convert_to_pose(pick_transform)
            pick_pose_stamped.header.frame_id = self.__object_frame_id

            rospy.loginfo("%s, %s, %s", object_pose_stamped.pose.position.x, object_pose_stamped.pose.position.y, object_pose_stamped.pose.position.z)
            rospy.loginfo("%s, %s, %s", standby_pose_stamped.pose.position.x, standby_pose_stamped.pose.position.y, standby_pose_stamped.pose.position.z)
            rospy.loginfo("%s, %s, %s", pick_pose_stamped.pose.position.x, pick_pose_stamped.pose.position.y, pick_pose_stamped.pose.position.z)
            # rospy.loginfo("%s, %s, %s, %s", new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w)

            self.__set_gripper(True)
            rospy.sleep(2.0)

            self.__move_by_pose(standby_pose_stamped)
            # rospy.sleep(2.0)

            self.__move_by_pose(pick_pose_stamped)
            # rospy.sleep(2.0)

            self.__set_gripper(False)
            rospy.sleep(2.0)

            self.__move_by_pose(standby_pose_stamped)
            # rospy.sleep(2.0)

            self.__move_by_name('home')
            # rospy.sleep(2.0)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cute_unit_app_node')

    pick_place = PickPlace()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pick_place.run()
        rate.sleep()


if __name__ == '__main__':
    main()
