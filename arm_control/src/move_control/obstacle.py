#!/usr/bin/env python3
# encoding: utf-8

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import sys
import os
home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from utils import *
from move import *

class Obstacle:
    def __init__(self, scene, id, posestamped, size):
        self.scene = scene
        self.id = id
        self.posestamped = posestamped
        self.size = size

    def add_obstacle(self):
        self.scene.add_box(self.id, self.posestamped, size=self.size)

    def remove_obstacle(self):
        self.scene.remove_world_object(self.id)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('obstacle', anonymous=True)
    scene = moveit_commander.PlanningSceneInterface()
    pose_box = PoseStamped()
    pose_box.header.frame_id = "world"
    pose_box.pose.position.x = 0.0
    pose_box.pose.position.y = 0.0
    pose_box.pose.position.z = -0.05
    pose_box.pose.orientation.x = 0.0
    pose_box.pose.orientation.y = 0.0
    pose_box.pose.orientation.z = 0.0
    pose_box.pose.orientation.w = 1.0
    size_box = [2, 2, 0.01]

    box = Obstacle(scene, "box", pose_box, size_box)
    box.add_obstacle()
        