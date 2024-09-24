#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import sys
import os
import message_filters
import numpy as np
import yaml
import moveit_commander
from sensor_msgs.msg import Image

sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from uilts_to_depth import *
from uilts_to_color import *
from uilts_in_color import *
from uilts_in_depth import *
import files_op

if handeye == 'in':
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    camera_ir_img_corrected_topic = camera_in_ir_img_corrected_topic
    
    camera_poses_end_path = os.getcwd() + camera_in_poses_end_path
    
    camera_color_img_path = os.getcwd() + camera_in_color_img_path
    camera_ir_img_path = os.getcwd() + camera_in_ir_img_path
    
else:
    if handeye == 'to':
        camera_color_img_corrected_topic = camera_to_color_img_corrected_topic
        camera_ir_img_corrected_topic = camera_to_ir_img_corrected_topic
        
        camera_poses_end_path = os.getcwd() + camera_to_poses_end_path
        
        camera_color_img_path = os.getcwd() + camera_to_color_img_path
        camera_ir_img_path = os.getcwd() + camera_to_ir_img_path
        
    else:
        print("Please choose handeye in or to")

image_gray_color = None
image_gray_ir = None

def image_callback(color_msg, ir_msg):
    global image_gray_color, image_gray_ir
    image_gray_color = cv2.cvtColor(np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1), cv2.COLOR_RGB2GRAY)
    image_gray_ir = cv2.convertScaleAbs(np.frombuffer(ir_msg.data, dtype=np.uint16).reshape(ir_msg.height, ir_msg.width, -1), alpha=(255.0/65535.0))
    
    cv2.imshow('camera_color', image_gray_color)
    cv2.imshow('camera_ir', image_gray_ir)

    cv2.waitKey(1)

def image_save(img, path, num):
    cv2.imwrite(path + num + '.jpg', img)

def get_img_num(path):
    img_ext = ['.jpg', '.png', '.jpeg']
    img_num = 0
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.lower().endswith(tuple(img_ext)):
                img_num += 1
    return img_num

def main():
    rospy.init_node('get_cam2world_data', anonymous=True)
    
    color_sub = message_filters.Subscriber(camera_color_img_corrected_topic, Image)
    ir_sub = message_filters.Subscriber(camera_ir_img_corrected_topic, Image)
    
    ts = message_filters.ApproximateTimeSynchronizer([color_sub, ir_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(image_callback)
    
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    
    with open(camera_poses_end_path, 'r') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)
        
    global image_gray_color, image_gray_ir

    count_color = get_img_num(camera_color_img_path)
    count_ir = get_img_num(camera_ir_img_path)
    
    if count_color == count_ir:
        count = count_color
    else:
        print("something wrong with the number of images")
    
    while True:
        user_input = input("Enter 1 to get data, 0 to exit, 2 to clear folder:")
        
        if user_input == '0':
            with open(camera_poses_end_path, 'w') as file:
                yaml.dump(data, file)
            break
        
        elif user_input == '1':
            pose_now = group.get_current_pose().pose
            
            image_save(image_gray_color, camera_color_img_path, str(count))
            image_save(image_gray_ir, camera_ir_img_path, str(count))
            
            if data ==None:
                data = {'poses_end': []}
            data['poses_end'].append( {'pose' + str(count): 
                {'position': {'x': pose_now.position.x, 'y': pose_now.position.y, 'z': pose_now.position.z},
                 'orientation': {'x': pose_now.orientation.x, 'y': pose_now.orientation.y, 'z': pose_now.orientation.z, 'w': pose_now.orientation.w}}})
            
            count += 1
            
        elif user_input == '2':
            files_op.clear_folder(camera_color_img_path)
            files_op.clear_folder(camera_ir_img_path)
            count = get_img_num(camera_color_img_path)
            data = {'poses_end': []}
        else:
            print("Invalid input!")

if __name__ == '__main__':
    main()