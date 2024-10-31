#!/usr/bin/env python3
# encoding: utf-8

import rospy
import sys
import roslib
import numpy as np
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint
from std_srvs.srv import SetBool
from math import pi

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')
sys.path.append(package_path + '/scripts/srv/')

from utils import *
from move import *

from GetPose import *

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_track', anonymous=True)
    
    rospy.wait_for_service('get_pose')
    rospy.wait_for_service('gripper_pick')
    get_pose = rospy.ServiceProxy('get_pose', GetPose)
    gripper_pick = rospy.ServiceProxy('gripper_pick', SetBool)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = moveit_commander.MoveGroupCommander(group_arm_name)
    
    group_arm.set_planner_id(planner_id)
    group_arm.set_max_velocity_scaling_factor(max_velocity_scaling_factor)
    group_arm.set_max_acceleration_scaling_factor(max_acceleration_scaling_factor)
    set_pose_tolerance(group_arm)
    
    joint_state = group_arm.get_current_joint_values()
    jc1 = JointConstraint(joint_name="joint1", position = joint_state[0], tolerance_above = 2, tolerance_below = 2, weight = 1.0)
    jc2 = JointConstraint(joint_name="joint2", position = joint_state[1], tolerance_above = 2, tolerance_below = 2, weight = 1.0)
    jc3 = JointConstraint(joint_name="joint3", position = joint_state[2], tolerance_above = 1, tolerance_below = 1, weight = 1.0)
    jc4 = JointConstraint(joint_name="joint4", position = joint_state[3], tolerance_above = 1, tolerance_below = 1, weight = 1.0)
    jc5 = JointConstraint(joint_name="joint5", position = joint_state[4], tolerance_above = 1, tolerance_below = 1, weight = 0.5)
    jc6 = JointConstraint(joint_name="joint6", position = joint_state[5], tolerance_above = np.pi, tolerance_below = np.pi, weight = 0.5)
    
    constraints = moveit_commander.Constraints()
    # constraints.joint_constraints.append(jc1)
    # constraints.joint_constraints.append(jc2)
    # constraints.joint_constraints.append(jc3)
    # constraints.joint_constraints.append(jc4)
    constraints.joint_constraints.append(jc5)
    constraints.joint_constraints.append(jc6)
    group_arm.set_path_constraints(constraints)

    # pose_home = group_arm.get_current_pose().pose
    # pose_init = pose_home
    # pose_init.orientation = rpy2quaternion(0, np.pi, np.pi)
    # pose_move(group_arm, pose_init, 'init')
    
    # group_arm.clear_path_constraints()
    
    rospy.sleep(2)
    
    joint_home = group_arm.get_current_joint_values()
    pose_home = group_arm.get_current_pose().pose
    
    while not rospy.is_shutdown():
        response = get_pose()
        pose_goal = response.pose
        success = response.success
        pose_goal.position.z += 0.05
        if not success :
            continue
        if pose_move(group_arm, pose_goal, 'go to red'):
            gripper_pick(True)
            pose_goal.position.z -= 0.05
            if pose_move(group_arm, pose_goal, 'pick'):
                gripper_pick(False)

        joint_move(group_arm, joint_home, 'home')
        gripper_pick(True)
        rospy.sleep(1)
        
    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()