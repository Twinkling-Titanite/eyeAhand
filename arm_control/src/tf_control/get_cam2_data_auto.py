#!/usr/bin/env python3
# encoding: utf-8

import rospy
import os
import cv2
import sys
import yaml
import random
import message_filters
import moveit_commander
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point

home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from uilts_in_color import *
from uilts_to_color import *
from uilts_in_depth import *
from uilts_to_depth import *
from move import *
from files_op import *

if handeye == 'in':
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    camera_max_positions_path = os.getcwd() + camera_in_max_positions_path
    camera_color_img_path = os.getcwd() + camera_in_color_img_path
    camera_ir_img_path = os.getcwd() + camera_in_ir_img_path
    camera_poses_end_path = os.getcwd() + camera_in_poses_end_path
    
else:
    if handeye == 'to':
        camera_color_img_corrected_topic = camera_to_color_img_corrected_topic
        camera_max_positions_path = os.getcwd() + camera_to_max_positions_path
        camera_color_img_path = os.getcwd() + camera_to_color_img_path
        camera_ir_img_path = os.getcwd() + camera_to_ir_img_path
        camera_poses_end_path = os.getcwd() + camera_to_poses_end_path
        
    else:
        print("Please choose handeye in or to")

image_gray_color = None
image_gray_ir = None

def get_move_poses_target(path):
    with open(path, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    position_center = Point()
    position_center.x = 0
    position_center.y = 0
    position_center.z = 0
    
    counter = 0
    poses = []
    for position in data['max_positions']:
        position_center.x += position['position' + str(counter)]['x']
        position_center.y += position['position' + str(counter)]['y']
        position_center.z += position['position' + str(counter)]['z']
        for i in range(2):
            pose = Pose()
            pose.position.x = position['position' + str(counter)]['x']
            pose.position.y = position['position' + str(counter)]['y']
            pose.position.z = position['position' + str(counter)]['z']
            pose.orientation = rpy2quaternion(30/180*np.pi + random.randint(-10,10)/180*np.pi,
                                                210/180*np.pi + random.randint(-10,10)/180*np.pi,
                                                135/180*np.pi + random.randint(-10,10)/180*np.pi)
            poses.append(pose)
        counter += 1
    position_center.x /= len(data['max_positions']) 
    position_center.y /= len(data['max_positions']) 
    position_center.z /= len(data['max_positions'])
    for i in range(2):
        pose = Pose()
        pose.position = position_center
        pose.orientation = rpy2quaternion(30/180*np.pi + random.randint(-10,10)/180*np.pi,
                                            210/180*np.pi + random.randint(-10,10)/180*np.pi,
                                            135/180*np.pi + random.randint(-10,10)/180*np.pi)
        poses.append(pose)
    
    return poses

def img_callback(color_msg, ir_msg):
    global image_gray_color, image_gray_ir
    image_gray_color = cv2.cvtColor(np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1), cv2.COLOR_RGB2GRAY)
    image_gray_ir = cv2.convertScaleAbs(np.frombuffer(ir_msg.data, dtype=np.uint16).reshape(ir_msg.height, ir_msg.width, -1), alpha=(255.0/65535.0))
    
    cv2.imshow("camera_rgb", image_gray_color)
    cv2.imshow("camera_ir", image_gray_ir)
    cv2.waitKey(1)

def main():
    rospy.init_node('get_cam2_data_auto', anonymous=True)
    color_sub = message_filters.Subscriber(camera_color_img_corrected_topic, Image)
    ir_sub = message_filters.Subscriber(camera_ir_img_path, Image)
    
    ts = message_filters.ApproximateTimeSynchronizer([color_sub, ir_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(img_callback)
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    group.set_max_velocity_scaling_factor(max_velocity_scaling_factor)
    group.set_max_acceleration_scaling_factor(max_acceleration_scaling_factor)
    group.set_planner_id(planner_id)
    
    poses_goal = get_move_poses_target(camera_max_positions_path) 
    
    counter = 0
    global image_gray
    
    clear_folder(camera_color_img_path)
    clear_folder(camera_ir_img_path)
    
    data = {'poses_end': []}
    for pose_goal in poses_goal:
        pose_move(group, pose_goal, 'take photo')
        rospy.sleep(0.5)
        cv2.imwrite(camera_color_img_path + str(counter) + '.jpg', image_gray)
        pose_now = group.get_current_pose().pose 

        data['poses_end'].append( {'pose' + str(counter): 
                {'position': {'x': pose_now.position.x, 'y': pose_now.position.y, 'z': pose_now.position.z},
                'orientation': {'x': pose_now.orientation.x, 'y': pose_now.orientation.y, 'z': pose_now.orientation.z, 'w': pose_now.orientation.w}}})
        rospy.sleep(0.5)
        counter += 1
        
    with open(camera_poses_end_path, 'w') as f:
        yaml.dump(data, f)
    
    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()
    
    