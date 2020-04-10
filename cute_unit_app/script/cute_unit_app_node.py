#!/usr/bin/env python

import sys

import rospy
import math3d
import moveit_commander

import geometry_msgs.msg
from std_msgs.msg import Float64
from dynamixel_controllers.srv import TorqueEnable

from cute_unit_app.srv import GetObject


class PickPlace:
    def __init__(self):
        self.__arm_group = moveit_commander.MoveGroupCommander('cute_arm')
        self.__gripper_command_pub = rospy.Publisher('/claw_controller/command', Float64, queue_size=10)
        self.__gripper_torque_service_proxy = rospy.ServiceProxy('/claw_controller/torque_enable', TorqueEnable)
        rospy.loginfo("arm setup")
        self.__gripper_torque_service_proxy(torque_enable=True)
        self.__set_gripper(False)
        rospy.sleep(1.0)
        self.__set_gripper(True)
        rospy.sleep(1.0)
        self.__move_by_name('home')
        rospy.sleep(1.0)

        self.__object_service_proxy = rospy.ServiceProxy('/object_detect_node/request_first_object', GetObject)

    def __convert_to_transform(self, pose_stamped):
        quaternion = math3d.UnitQuaternion(pose_stamped.pose.orientation.w, pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z)
        transform = math3d.Transform()
        transform.set_pos((pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z))
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
            self.__gripper_command_pub.publish(0.0)
        else:
            self.__gripper_command_pub.publish(-1.0)

    def __move_by_pose(self, target_pose):
        self.__arm_group.set_pose_target(target_pose)
        return self.__arm_group.go(wait=True)

    def __move_by_name(self, target_name):
        self.__arm_group.set_named_target(target_name)
        return self.__arm_group.go(wait=True)

    def pick_place(self, target_pose):
        pass

    def run(self):
        rospy.loginfo("request object pose...")
        try:
            res = self.__object_service_proxy(request_type=0)
        except rospy.ServiceException:
            rospy.loginfo("no object")
            return

        rospy.loginfo("found object")

        object_transform = self.__convert_to_transform(res.object_pose)

        standby_offset = math3d.Transform()
        standby_offset.pos = math3d.Vector(0, 0, -0.03)
        standby_transform = object_transform * standby_offset
        standby_pose_stamped = self.__convert_to_pose(standby_transform)
        standby_pose_stamped.header.frame_id = res.object_pose.header.frame_id

        pick_offset = math3d.Transform()
        pick_offset.pos = math3d.Vector(0, 0, 0)
        pick_transform = object_transform * pick_offset
        pick_pose_stamped = self.__convert_to_pose(pick_transform)
        pick_pose_stamped.header.frame_id = res.object_pose.header.frame_id

        rospy.loginfo("object pose: %s, %s, %s", res.object_pose.pose.position.x, res.object_pose.pose.position.y, res.object_pose.pose.position.z)
        rospy.loginfo("standby pose: %s, %s, %s", standby_pose_stamped.pose.position.x, standby_pose_stamped.pose.position.y, standby_pose_stamped.pose.position.z)
        # rospy.loginfo("%s, %s, %s", pick_pose_stamped.pose.position.x, pick_pose_stamped.pose.position.y, pick_pose_stamped.pose.position.z)

        self.__set_gripper(True)
        rospy.sleep(2.0)

        if self.__move_by_pose(standby_pose_stamped) is not True:
            rospy.loginfo("unable to get standby pose")
            return
        # rospy.sleep(2.0)

        if self.__move_by_pose(pick_pose_stamped) is not True:
            rospy.loginfo("unable to get object pose")
            return
        # rospy.sleep(2.0)

        self.__set_gripper(False)
        rospy.sleep(2.0)

        if self.__move_by_pose(standby_pose_stamped) is not True:
            rospy.loginfo("unable to get standby pose")
            return
        # rospy.sleep(2.0)

        if self.__move_by_name('home') is not True:
            rospy.loginfo("unable to get home pose")
            return
        # rospy.sleep(2.0)

        rospy.loginfo("finish")


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cute_unit_app_node')

    pick_place = PickPlace()

    rate = rospy.Rate(0.4)
    while not rospy.is_shutdown():
        pick_place.run()
        rate.sleep()


if __name__ == '__main__':
    main()
