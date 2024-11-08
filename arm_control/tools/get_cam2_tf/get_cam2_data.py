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
import files_op
import get_tf

while not rospy.is_shutdown():
    eyehand = input("Please choose eyehand in or to: ")
    if eyehand not in ['in', 'to']:
        print("Invalid input!")
        continue
    break

if eyehand == 'in':
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    camera_color_info_path = package_path + camera_in_color_info_path
    cam2_color_tf_path = package_path + cam2end_color_tf_path
else:
    if eyehand == 'to':
        camera_color_img_corrected_topic = camera_to_color_img_corrected_topic
        camera_color_info_path = package_path + camera_to_color_info_path
        cam2_color_tf_path = package_path + cam2world_color_tf_path
        
camera_poses_end_path = package_path + '/tools/get_cam2_tf/poses_end/camera_'+ eyehand + '/poses_end.yaml'
camera_color_img_path = package_path + '/tools/get_cam2_tf/img/camera_' + eyehand + '/'
camera_color_result_path = package_path + '/tools/get_cam2_tf/result/camera_' + eyehand + '/'

with open(camera_color_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_color_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_color = np.array(data['camera_matrix']).reshape((3,3))

# 准备棋盘格的3D点，例如 (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
object_points = np.zeros((np.prod(chessboard_size), 3), np.float32)
object_points[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2)
object_points *= square_size

image_gray_color = None
image_gray_ir = None

def image_callback_rgb(color_msg):
    global image_gray_color
    image_gray_color = cv2.cvtColor(np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1), cv2.COLOR_RGB2GRAY)

def image_save(img, path, num):
    cv2.imwrite(path + num + '.jpg', img)

def main():
    rospy.init_node('get_cam2world_data', anonymous=True)
    

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
            count = files_op.get_img_num(camera_color_img_path)
            data = {'poses_end': []}
            
        elif user_input == 'c':
            files_op.clear_folder(camera_color_result_path)
            img_color_num = files_op.get_img_num(camera_color_img_path)
            
            get_tf.get_cam2_tf(
                camera_color_img_path,
                camera_poses_end_path,
                img_color_num,
                camera_color_result_path,
                object_points,
                chessboard_size,
                camera_matrix_color,
                cam2_color_tf_path,
                eyehand
            )
            
        else:
            print("Invalid input!")

if __name__ == '__main__':
    main()