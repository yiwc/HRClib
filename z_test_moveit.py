#!/usr/bin/env python


# References
# ----------
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html


import sys
import copy
import rospy
import moveit_commander
from movo_action_clients.gripper_action_client import GripperActionClient

from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

import geometry_msgs.msg

# Importing this just for the .quaternion_from_euler() function
import tf
import math

_upper_body_joints = ["right_shoulder_pan_joint",
                      "right_shoulder_lift_joint",
                      "right_arm_half_joint",
                      "right_elbow_joint",
                      "right_wrist_spherical_1_joint",
                      "right_wrist_spherical_2_joint",
                      "right_wrist_3_joint",
                      "left_shoulder_pan_joint",
                      "left_shoulder_lift_joint",
                      "left_arm_half_joint",
                      "left_elbow_joint",
                      "left_wrist_spherical_1_joint",
                      "left_wrist_spherical_2_joint",
                      "left_wrist_3_joint",
                      "linear_joint",
                      "pan_joint",
                      "tilt_joint"]
default_pose_tucked = [-1.595, -1.5, 0.40, -2.612, 0.0, 0.496, -1.69,
                       1.595, 1.5, -0.4, 2.612, 0.0, -0.496, 1.69,
                       0.14, 0, -0.6]

# pos before screw

# R-arm:
# position:
# x: 0.751439135541
# y: -0.170846250246
# z: 1.17794694979
# orientation:
# x: -0.721548240643
# y: -0.678997389435
# z: 0.0914187200588
# w: 0.099866406705

#Larm:
# pose:
#   position:
#     x: 0.749532245054
#     y: 0.196965046952
#     z: 1.17979921488
#   orientation:
#     x: 0.496355726331
#     y: 0.474604993393
#     z: -0.511413369809
#     w: 0.516563121376


if __name__ == "__main__":
    rospy.init_node('movo_moveit_test',
                    anonymous=False)

    moveit_commander.roscpp_initialize(sys.argv)

    scene = moveit_commander.PlanningSceneInterface()

    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')

    larm_group = moveit_commander.MoveGroupCommander("left_arm")
    rarm_group = moveit_commander.MoveGroupCommander("right_arm")
    upper_body = moveit_commander.MoveGroupCommander("upper_body")

    move_group = MoveGroupInterface("upper_body", "base_link")
    lmove_group = MoveGroupInterface("left_arm", "base_link")
    rmove_group = MoveGroupInterface("right_arm", "base_link")

    # upper_body.go(joints=[])
    # larm_group.

    gripper_closed = 0.00
    gripper_open = 0.165

    print("starting")

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = float(0.749532245054)
    pose_goal.position.y = float(0.35)
    pose_goal.position.z = float(1.3)
    quat = tf.transformations.quaternion_from_euler(float(0), float(0), float(-math.pi / 2))
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    larm_group.set_pose_target(pose_goal)
    plan = larm_group.go(wait=True)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = float(0.749532245054)
    pose_goal.position.y = float(-0.35)
    pose_goal.position.z = float(1.17979921488)
    quat = tf.transformations.quaternion_from_euler(float(0), float(math.pi / 2), float(-math.pi / 2))
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    rarm_group.set_pose_target(pose_goal)
    plan = rarm_group.go(wait=True)


    # larm_group.set_pose_target(pose_goal)

    for i in range(1, 10):
        print("Moving back...")
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.position.x = float(0.535866126205)
        # pose_goal.position.y = float(0.0802986860614)
        # pose_goal.position.z = float(1.22187608755)
        #
        # quat = tf.transformations.quaternion_from_euler(float(0), float(-math.pi / 2), float(-math.pi / 2))
        # pose_goal.orientation.x = quat[0]
        # pose_goal.orientation.y = quat[1]
        # pose_goal.orientation.z = quat[2]
        # pose_goal.orientation.w = quat[3]
        #
        # larm_group.set_pose_target(pose_goal)
        #
        # plan = larm_group.go(wait=True)
        #
        # print("Moving forward...")
        #
        # pose_goal = geometry_msgs.msg.Pose()
        #
        # pose_goal.position.x = float(0.735866126205)
        # pose_goal.position.y = float(0.0802986860614)
        # pose_goal.position.z = float(1.22187608755)
        #
        # quat = tf.transformations.quaternion_from_euler(float(0), float(-math.pi / 2), float(-math.pi / 2))
        # pose_goal.orientation.x = quat[0]
        # pose_goal.orientation.y = quat[1]
        # pose_goal.orientation.z = quat[2]
        # pose_goal.orientation.w = quat[3]
        #
        # larm_group.set_pose_target(pose_goal)
        #
        # plan = larm_group.go(wait=True)

    print("Opening grippers...")
    lgripper.command(gripper_open, block=True)
    rgripper.command(gripper_open, block=True)

    print("Closing grippers...")
    lgripper.command(gripper_closed, block=True)
    rgripper.command(gripper_closed, block=True)

    print("Successful!")

