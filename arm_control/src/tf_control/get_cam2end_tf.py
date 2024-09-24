#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import sys
import os
import numpy as np
from sensor_msgs.msg import CameraInfo

sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from uilts_in_color import *
from uilts_in_depth import *
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

# 加载Charuco板字典和参数
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
charuco_board = cv2.aruco.CharucoBoard_create(7, 7, 0.02, 0.014, aruco_dict)

def get_img_num(path):
    img_ext = ['.jpg', '.png', '.jpeg']
    img_num = 0
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.lower().endswith(tuple(img_ext)):
                img_num += 1
    return img_num

img_color_num = get_img_num(os.getcwd() + camera_in_color_img_path)
img_ir_num = get_img_num(os.getcwd() + camera_in_ir_img_path)

def camera_color_info_callback(msg):
    global camera_matrix_color, dist_coeffs_color
    # 读取相机内参
    camera_matrix_color = np.array(msg.K).reshape(3, 3)
    # 读取畸变系数
    # dist_coeffs_color = np.array(msg.D)

def camera_ir_info_callback(msg):
    global camera_matrix_ir, dist_coeffs_ir
    # 读取相机内参
    camera_matrix_ir = np.array(msg.K).reshape(3, 3)
    # 读取畸变系数
    # dist_coeffs_ir = np.array(msg.D)

def main():
    rospy.init_node('to_get_cam2world_tf', anonymous=True)

    rospy.Subscriber(camera_in_color_info_topic, CameraInfo, camera_color_info_callback)
    rospy.Subscriber(camera_in_ir_info_topic, CameraInfo, camera_ir_info_callback)
    
    files_op.clear_folder(os.getcwd() + camera_in_color_result_path)
    files_op.clear_folder(os.getcwd() + camera_in_ir_result_path)

    while camera_matrix_color is None or camera_matrix_ir is None:
        pass
    
    get_tf.get_cam2_tf(os.getcwd() + camera_in_color_img_path,
                     os.getcwd() + camera_in_poses_end_path,
                     img_color_num,
                     os.getcwd() + camera_in_color_result_path,
                     object_points,
                     chessboard_size,
                     camera_matrix_color,
                     os.getcwd() + cam2end_color_tf_path,
                     'in')

    get_tf.get_cam2_tf(os.getcwd() + camera_in_ir_img_path,
                     os.getcwd() + camera_in_poses_end_path,
                     img_ir_num,
                     os.getcwd() + camera_in_ir_result_path,
                     object_points,
                     chessboard_size,
                     camera_matrix_ir,
                     os.getcwd() + cam2end_ir_tf_path,
                     'in')

if __name__ == '__main__':
    main()