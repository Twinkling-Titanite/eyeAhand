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
home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from move import *

# class Pose:
#     def __init__(self, position = np.array([0, 0, 0]), roll = 0, pitch = 0, yaw = 0):
#         self.position = position
#         self.roll = roll
#         self.pitch = pitch
#         self.yaw = yaw

# pose = [[[Pose() for _ in range(depth)] for _ in range(rows)] for _ in range(cols)]

# for i in range(cols):
#     for j in range(rows):
#         for k in range(depth):
#             pose[i][j][k] = Pose(np.array([position_init[0] + ((i-(cols-1)/2) * step_x) * (depth - k),
#                                            position_init[1] + ((j-(rows-1)/2) * step_y) * (depth - k),
#                                            position_init[2] + (k-(depth-1)/2) * step_z]),
#                                            random.randint(-20, 20) * pi/180,
#                                            random.randint(-20, 20) * pi/180 + 3*pi/2,
#                                            random.randint(-20, 20) * pi/180 + pi)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_test', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander(group_arm_name)
    
    group_arm.set_planner_id(planner_id)
    group_arm.set_max_velocity_scaling_factor(max_velocity_scaling_factor)
    group_arm.set_max_acceleration_scaling_factor(max_acceleration_scaling_factor)
    set_pose_tolerance(group_arm)
    
    # joint_move(group_arm, [0, 0, 0, 0, 0, 0], "move_to_zero")
    print(group_arm.get_current_pose().pose)
    pose_home = group_arm.get_current_pose().pose
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.29
    pose_goal.position.y = 0.1652074
    pose_goal.position.z = 0.62005802
    pose_goal.orientation = pose_home.orientation
    

    pose_move(group_arm, pose_goal, "pose_move_test")
    pose_move(group_arm, pose_goal, "pose_move_test")
    print(group_arm.get_current_pose().pose)
    
    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()