#!/usr/bin/env python3
# encoding: utf-8

import rospy
import yaml
import sys
import os
import numpy as np
from sensor_msgs.msg import CameraInfo
home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from uilts_to_color import *
from uilts_to_depth import *

import files_op
import get_tf

camera_matrix_color = None
dist_coeffs_color = None
camera_matrix_ir = None
dist_coeffs_ir = None

# 准备棋盘格的3D点，例如 (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
object_points = np.zeros((np.prod(chessboard_size), 3), np.float32)
object_points[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2)
object_points *= square_size


def get_img_num(path):
    img_ext = ['.jpg', '.png', '.jpeg']
    img_num = 0
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.lower().endswith(tuple(img_ext)):
                img_num += 1
    return img_num

img_color_num = get_img_num(os.getcwd() + camera_to_color_img_path)
img_ir_num = get_img_num(os.getcwd() + camera_to_ir_img_path)

with open(os.getcwd() + camera_to_color_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_color_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_color = np.array(data['camera_matrix']).reshape((3,3))
        if 'dist_coeffs' in data and data['dist_coeffs']!= []:
            dist_coeffs_color = np.array(data['dist_coeffs']).reshape((-1,1))

with open(os.getcwd() + camera_to_ir_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_ir_img_raw_topic.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_ir = np.array(data['camera_matrix']).reshape((3,3))
        if 'dist_coeffs' in data and data['dist_coeffs']!= []:
            dist_coeffs_ir = np.array(data['dist_coeffs']).reshape((-1,1))

def main():
    rospy.init_node('to_get_cam2world_tf', anonymous=True)
    
    files_op.clear_folder(os.getcwd() + camera_to_color_result_path)
    files_op.clear_folder(os.getcwd() + camera_to_ir_result_path)
    
    get_tf.get_cam2_tf(os.getcwd() + camera_to_color_img_path,
                     os.getcwd() + camera_to_poses_end_path,
                     img_color_num,
                     os.getcwd() + camera_to_color_result_path,
                     object_points,
                     chessboard_size,
                     camera_matrix_color,
                     os.getcwd() + cam2world_color_tf_path,
                     'to')
    
    if correct_depth_camera:
        get_tf.get_cam2_tf(os.getcwd() + camera_to_ir_img_path,
                           os.getcwd() + camera_to_poses_end_path,
                           img_ir_num,
                           os.getcwd() + camera_to_ir_result_path,
                           object_points,
                           chessboard_size,
                           camera_matrix_ir,
                           os.getcwd() + cam2world_ir_tf_path,
                           'to')

if __name__ == '__main__':
    main()