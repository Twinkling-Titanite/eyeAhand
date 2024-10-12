#!/usr/bin/env python3
# encoding: utf-8

import rospy
import sys
import os
import moveit_commander
import geometry_msgs.msg
from math import pi
home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from move import *

pose_red = None

pose_home = geometry_msgs.msg.Pose()
pose_home.orientation.x = 0.0
pose_home.orientation.y = 0.707
pose_home.orientation.z = -0.707
pose_home.orientation.w = 0.0
pose_home.position.x = 0.08
pose_home.position.y = 0.09
pose_home.position.z = 0.66
    
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
    
    group_arm.set_planner_id(planner_id)
    group_arm.set_max_velocity_scaling_factor(max_velocity_scaling_factor)
    group_arm.set_max_acceleration_scaling_factor(max_acceleration_scaling_factor)
    # set_pose_tolerance(group_arm)
    # pose_move(group_arm, pose_home, 'home')

    rospy.sleep(2)
    
    global pose_red
    while pose_red is None:
        pass
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation = pose_red.orientation
    pose_goal.position.x = pose_red.position.x
    pose_goal.position.y = pose_red.position.y
    pose_goal.position.z = pose_red.position.z
    
    pose_move(group_arm, pose_goal, 'go to red')
    
    # pose_move(group_arm, pose_home, 'home')
    
    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()