#!/usr/bin/env python3
# encoding: utf-8

import rospy
import sys
import os
import yaml
import numpy as np
import moveit_commander
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import cv2
import math
home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from uilts_in_color import *
from uilts_to_color import *
from uilts_in_depth import *
from uilts_to_depth import *

if handeye == 'in':
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    camera_ir_img_corrected_topic = camera_in_ir_img_corrected_topic
    camera_max_positions_path = os.getcwd() + camera_in_max_positions_path
    
else:
    if handeye == 'to':
        camera_color_img_corrected_topic = camera_to_color_img_corrected_topic
        camera_ir_img_corrected_topic = camera_to_ir_img_corrected_topic
        camera_max_positions_path = os.getcwd() + camera_to_max_positions_path
        
    else:
        print("Please choose handeye in or to")

def img_color_callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)

def img_ir_callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('get_max_position', anonymous=True)
    rospy.Subscriber(camera_color_img_corrected_topic, Image, img_color_callback)
    rospy.Subscriber(camera_ir_img_corrected_topic, Image, img_ir_callback)
    
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    
    data = {'max_positions': []}
    with open(camera_max_positions_path, 'w') as f:
        yaml.dump(data, f)
        
    counter = 0
    
    while True:
        if counter == 8:
            print("Max positions saved to file")
            with open(camera_max_positions_path, 'w') as f:
                yaml.dump(data, f)
            break
        user_input = input("Enter 1 to save the max position, 0 to exit: ")
        if user_input == '1':
            position = group.get_current_pose().pose.position
            data['max_positions'].append({'position' + str(counter):
                {'x': position.x, 'y': position.y, 'z': position.z}})
            counter += 1
        elif user_input == '0':
            break
        else:
            print("Invalid input")

if __name__ == '__main__':
    main()