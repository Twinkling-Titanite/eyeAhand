#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import sys
import message_filters
import numpy as np
import yaml
import roslib
import moveit_commander
from sensor_msgs.msg import Image

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from utils import *
from utils_to_depth import *
from utils_to_color import *
from utils_in_color import *
from utils_in_depth import *
import files_op
import get_tf

if handeye == 'in':
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    camera_ir_img_corrected_topic = camera_in_ir_img_corrected_topic
    
    camera_poses_end_path = package_path + camera_in_poses_end_path
    
    camera_color_img_path = package_path + camera_in_color_img_path
    camera_ir_img_path = package_path + camera_in_ir_img_path
    
    camera_color_info_path = package_path + camera_in_color_info_path
    camera_ir_info_path = package_path + camera_in_ir_info_path
    
    camera_color_result_path = package_path + camera_in_color_result_path
    camera_ir_result_path = package_path + camera_in_ir_result_path
    
    cam2_color_tf_path = package_path + cam2end_color_tf_path
    cam2_ir_tf_path = package_path + cam2end_ir_tf_path
else:
    if handeye == 'to':
        camera_color_img_corrected_topic = camera_to_color_img_corrected_topic
        camera_ir_img_corrected_topic = camera_to_ir_img_corrected_topic
        
        camera_poses_end_path = package_path + camera_to_poses_end_path
        
        camera_color_img_path = package_path + camera_to_color_img_path
        camera_ir_img_path = package_path + camera_to_ir_img_path
        
        camera_color_info_path = package_path + camera_to_color_info_path
        camera_ir_info_path = package_path + camera_to_ir_info_path
        
        camera_color_result_path = package_path + camera_to_color_result_path
        camera_ir_result_path = package_path + camera_to_ir_result_path
        
        cam2_color_tf_path = package_path + cam2world_color_tf_path
        cam2_ir_tf_path = package_path + cam2world_ir_tf_path
    else:
        print("Please choose handeye in or to")

with open(camera_color_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_color_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_color = np.array(data['camera_matrix']).reshape((3,3))

with open(camera_ir_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_ir_img_raw_topic.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_ir = np.array(data['camera_matrix']).reshape((3,3))

# 准备棋盘格的3D点，例如 (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
object_points = np.zeros((np.prod(chessboard_size), 3), np.float32)
object_points[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2)
object_points *= square_size

image_gray_color = None
image_gray_ir = None

def image_callback(color_msg, ir_msg):
    global image_gray_color, image_gray_ir
    image_gray_color = cv2.cvtColor(np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1), cv2.COLOR_RGB2GRAY)
    image_gray_ir = cv2.convertScaleAbs(np.frombuffer(ir_msg.data, dtype=np.uint16).reshape(ir_msg.height, ir_msg.width, -1), alpha=(255.0/65535.0))
    
    # cv2.imshow('camera_color', image_gray_color)
    # cv2.imshow('camera_ir', image_gray_ir)

    # cv2.waitKey(100)

def image_callback_rgb(color_msg):
    global image_gray_color
    image_gray_color = cv2.cvtColor(np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1), cv2.COLOR_RGB2GRAY)
    
    # image_gray_color_filtered = cv2.bilateralFilter(image_gray_color, 5, 75, 75)
    # laplacian = cv2.Laplacian(image_gray_color_filtered, cv2.CV_64F)
    # image_gray_color_sharpened = cv2.convertScaleAbs(laplacian)
    # image_gray_color = cv2.addWeighted(image_gray_color, 1.5, image_gray_color_sharpened, -0.5, 0)
    
    # cv2.imshow('camera_color', image_gray_color)
    # cv2.waitKey(5)

def image_save(img, path, num):
    cv2.imwrite(path + num + '.jpg', img)

def main():
    rospy.init_node('get_cam2world_data', anonymous=True)
    
    color_sub = message_filters.Subscriber(camera_color_img_corrected_topic, Image)
    ir_sub = message_filters.Subscriber(camera_ir_img_corrected_topic, Image)
    
    if correct_depth_camera:
        color_sub = message_filters.Subscriber(camera_color_img_corrected_topic, Image)
        ir_sub = message_filters.Subscriber(camera_ir_img_corrected_topic, Image)
    
        ts = message_filters.ApproximateTimeSynchronizer([color_sub, ir_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(image_callback)      
    else:
        color_sub = rospy.Subscriber(camera_color_img_corrected_topic, Image, image_callback_rgb)
    
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    
    with open(camera_poses_end_path, 'r') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)
        
    global image_gray_color, image_gray_ir

    count_color = files_op.get_img_num(camera_color_img_path)
    count = count_color

    while not rospy.is_shutdown():
        user_input = input("Enter s to save data, q to quit, d to delete all data, c to calibrate: ")
        
        if user_input == 'q':
            break
        
        elif user_input == 's':
            pose_now = group.get_current_pose().pose
            image_save(image_gray_color, camera_color_img_path, str(count))
            if correct_depth_camera:
                image_save(image_gray_ir, camera_ir_img_path, str(count))
            if data ==None:
                data = {'poses_end': []}
            data['poses_end'].append( {'pose' + str(count): 
                {'position': {'x': pose_now.position.x, 'y': pose_now.position.y, 'z': pose_now.position.z},
                 'orientation': {'x': pose_now.orientation.x, 'y': pose_now.orientation.y, 'z': pose_now.orientation.z, 'w': pose_now.orientation.w}}})
            with open(camera_poses_end_path, 'w') as file:
                yaml.dump(data, file)
            count += 1
            
        elif user_input == 'd':
            files_op.clear_folder(camera_color_img_path)
            files_op.clear_folder(camera_ir_img_path)
            count = files_op.get_img_num(camera_color_img_path)
            data = {'poses_end': []}
            
        elif user_input == 'c':
            files_op.clear_folder(camera_color_result_path)
            files_op.clear_folder(camera_ir_result_path)
            img_color_num = files_op.get_img_num(camera_color_img_path)
            img_ir_num = files_op.get_img_num(camera_ir_img_path)
            get_tf.get_cam2_tf(
                camera_color_img_path,
                camera_poses_end_path,
                img_color_num,
                camera_color_result_path,
                object_points,
                chessboard_size,
                camera_matrix_color,
                cam2_color_tf_path,
                handeye
            )
            if correct_depth_camera:
                get_tf.get_cam2_tf(
                    camera_ir_img_path,
                    camera_poses_end_path,
                    img_ir_num,
                    camera_ir_result_path,
                    object_points,
                    chessboard_size,
                    camera_matrix_ir,
                    cam2_ir_tf_path,
                    handeye
                )
        else:
            print("Invalid input!")

if __name__ == '__main__':
    main()