#!/usr/bin/env python3
# encoding: utf-8

import rospy
import sys
import os
import numpy as np
import random
from math import pi
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import OrientationConstraint, Constraints, JointConstraint
home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from utils import *
from move import *

oc = OrientationConstraint()
oc.header.frame_id = "base_link"
oc.link_name = "Link6"
oc.orientation.w = 0.0
oc.orientation.x = -0.707
oc.orientation.y = -0.707
oc.orientation.z = 0.0
oc.absolute_x_axis_tolerance = 0.2
oc.absolute_y_axis_tolerance = 0.2
oc.absolute_z_axis_tolerance = 0.2
oc.weight = 1.0

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_test', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander(group_arm_name)
    
    joint_state = group_arm.get_current_joint_values()
    
    jc1 = JointConstraint(joint_name="joint1", position = joint_state[0], tolerance_above = 1, tolerance_below = 1, weight = 0.5)
    jc2 = JointConstraint(joint_name="joint2", position = joint_state[1], tolerance_above = 1.5, tolerance_below = 1.5, weight = 1.0)
    jc3 = JointConstraint(joint_name="joint3", position = joint_state[2], tolerance_above = 1, tolerance_below = 1, weight = 1.0)
    jc4 = JointConstraint(joint_name="joint4", position = joint_state[3], tolerance_above = 1, tolerance_below = 1, weight = 1.0)
    jc5 = JointConstraint(joint_name="joint5", position = joint_state[4], tolerance_above = 1, tolerance_below = 1, weight = 1.0)
    jc6 = JointConstraint(joint_name="joint6", position = joint_state[5], tolerance_above = 0.5, tolerance_below = 0.5, weight = 0.5)
    
    constraints = moveit_commander.Constraints()
    constraints.joint_constraints.append(jc1)
    # constraints.joint_constraints.append(jc2)
    # constraints.joint_constraints.append(jc3)
    # constraints.joint_constraints.append(jc4)
    # constraints.joint_constraints.append(jc5)
    constraints.joint_constraints.append(jc6)
    # group_arm.set_path_constraints(constraints)
    # group_arm.clear_path_constraints()
    
    group_arm.set_planner_id(planner_id)
    group_arm.set_max_velocity_scaling_factor(max_velocity_scaling_factor)
    group_arm.set_max_acceleration_scaling_factor(max_acceleration_scaling_factor)
    set_pose_tolerance(group_arm)
    
    pose_home = group_arm.get_current_pose().pose
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.44
    pose_goal.position.y = 0
    pose_goal.position.z = 0.228
    pose_goal.orientation = pose_home.orientation

    pose_move(group_arm, pose_goal, "pose_move_test")
    print(group_arm.get_current_joint_values())
    print(group_arm.get_current_pose().pose)
    
    group_arm.clear_path_constraints()

    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()