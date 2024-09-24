#!/usr/bin/env python3
# encoding: utf-8

import rospy
import sys
import os
import moveit_commander
import geometry_msgs.msg
from math import pi

sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from move import *

gripper_open = [0.04, 0.04]
gripper_close = [0.007, 0.007]

pose_red = None

pose_home = geometry_msgs.msg.Pose()
pose_home.orientation = rpy2quaternion(0, 3.14, 0.785)
pose_home.position.x = 0.4
pose_home.position.y = 0.2
pose_home.position.z = 0.4
    
obstacle_id = "box"

obstacle_pose = geometry_msgs.msg.PoseStamped()
obstacle_pose.header.frame_id = "world"
obstacle_pose.pose.orientation = rpy2quaternion(0, 0, 0)
obstacle_pose.pose.position.x = 0.0
obstacle_pose.pose.position.y = 0.0
obstacle_pose.pose.position.z = -0.05

obstalce_size = [2, 2, 0.01]

def callback(data):
    global pose_red
    pose_red = data.pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_track', anonymous=True)
    
    rospy.Subscriber(world_coord_topic, geometry_msgs.msg.PoseStamped, callback)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    scene.add_box(obstacle_id, obstacle_pose, size=obstalce_size)
    rospy.sleep(0.5)
    group_arm = moveit_commander.MoveGroupCommander(group_arm_name)
    group_hand = moveit_commander.MoveGroupCommander(group_hand_name)
    
    group_arm.set_planner_id(planner_id)
    group_arm.set_max_velocity_scaling_factor(max_velocity_scaling_factor)
    group_arm.set_max_acceleration_scaling_factor(max_acceleration_scaling_factor)
    set_pose_tolerance(group_arm)
    pose_move(group_arm, pose_home, 'home')
    joint_move(group_hand, gripper_open, 'open gripper')
    rospy.sleep(2)
    
    global pose_red
    while pose_red is None:
        pass
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation = pose_red.orientation
    pose_goal.position.x = pose_red.position.x
    pose_goal.position.y = pose_red.position.y
    pose_goal.position.z = pose_red.position.z + 0.1
    
    pose_move(group_arm, pose_goal, 'go to red')
    joint_move(group_hand, gripper_close, 'close gripper')
    # pose_move(group_arm, pose_home, 'home')
    
    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()