#!/usr/bin/env python3
# encoding: utf-8

import rospy
import sys
import roslib
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint
from std_srvs.srv import SetBool
from math import pi

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from utils import *
from move import *

pose_red = None

def callback(data):
    global pose_red
    pose_red = data.pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_track', anonymous=True)
    
    rospy.Subscriber(world_coord_topic, geometry_msgs.msg.PoseStamped, callback)
    gripper_pick = rospy.ServiceProxy('gripper_pick', SetBool)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander(group_arm_name)
    
    group_arm.set_planner_id(planner_id)
    group_arm.set_max_velocity_scaling_factor(max_velocity_scaling_factor)
    group_arm.set_max_acceleration_scaling_factor(max_acceleration_scaling_factor)
    set_pose_tolerance(group_arm)
    
    joint_state = group_arm.get_current_joint_values()
    jc1 = JointConstraint(joint_name="joint1", position = joint_state[0], tolerance_above = 1.5, tolerance_below = 1.5, weight = 0.5)
    jc2 = JointConstraint(joint_name="joint2", position = joint_state[1], tolerance_above = 1.5, tolerance_below = 1.5, weight = 1.0)
    jc3 = JointConstraint(joint_name="joint3", position = joint_state[2], tolerance_above = 1, tolerance_below = 1, weight = 1.0)
    jc4 = JointConstraint(joint_name="joint4", position = joint_state[3], tolerance_above = 1, tolerance_below = 1, weight = 1.0)
    jc5 = JointConstraint(joint_name="joint5", position = joint_state[4], tolerance_above = 1, tolerance_below = 1, weight = 1.0)
    jc6 = JointConstraint(joint_name="joint6", position = joint_state[5], tolerance_above = 1.5, tolerance_below = 1.5, weight = 0.5)

    constraints = moveit_commander.Constraints()
    constraints.joint_constraints.append(jc1)
    # constraints.joint_constraints.append(jc2)
    # constraints.joint_constraints.append(jc3)
    # constraints.joint_constraints.append(jc4)
    # constraints.joint_constraints.append(jc5)
    constraints.joint_constraints.append(jc6)
    group_arm.set_path_constraints(constraints)

    pose_home = group_arm.get_current_pose().pose
    joint_home = group_arm.get_current_joint_values()

    rospy.sleep(2)
    
    global pose_red
    while pose_red is None:
        pass
    
    while not rospy.is_shutdown():
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = pose_home.orientation
        pose_goal.position.x = pose_red.position.x
        pose_goal.position.y = pose_red.position.y
        pose_goal.position.z = 0.27
    
        if pose_move(group_arm, pose_goal, 'go to red'):
            gripper_pick(True)
            pose_goal.position.z = 0.233
            if pose_move(group_arm, pose_goal, 'pick'):
                gripper_pick(False)
                pose_goal.position.z = 0.27
                pose_goal.position.y = 0
                pose_goal.position.x = 0.45
            if pose_move(group_arm, pose_goal, 'go to red'):
                gripper_pick(True)
        joint_move(group_arm, joint_home, 'home')
        rospy.sleep(3)
    
    # pose_move(group_arm, pose_home, 'home')
    
    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()