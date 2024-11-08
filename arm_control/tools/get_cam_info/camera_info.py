#!/usr/bin/env python3
# encoding: utf-8

import rospy
import yaml
import sys
import roslib
import numpy as np
from sensor_msgs.msg import CameraInfo
from orbbec_camera.msg import Extrinsics

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from utils import *

def camera_in_color_info_callback(camera_info):
    camera_color_info = {
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist()
    }
    
    with open(package_path + camera_in_color_info_path, 'w') as f:
        yaml.dump(camera_color_info, f)

def camera_in_depth_info_callback(camera_info):
    camera_depth_info = {
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist()
    }
    
    with open(package_path + camera_in_depth_info_path, 'w') as f:
        yaml.dump(camera_depth_info, f)

def camera_to_color_info_callback(camera_info):
    camera_to_color_info = {
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist()
    }
    
    with open(package_path + camera_to_color_info_path, 'w') as f:
        yaml.dump(camera_to_color_info, f)

def camera_to_depth_info_callback(camera_info):
    camera_to_depth_info = {
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist()
    }
    
    with open(package_path + camera_to_depth_info_path, 'w') as f:
        yaml.dump(camera_to_depth_info, f)

def camera_in_depth_to_color_callback(extrinsics):
    rotation = np.array(extrinsics.rotation).reshape(3, 3).tolist()
    translation = np.array(extrinsics.translation).reshape(3, 1).tolist()
    #合并旋转矩阵和平移向量
    extrinsics_matrix = np.hstack((rotation, translation))
    #转换为齐次矩阵
    extrinsics_matrix = np.vstack((extrinsics_matrix, [0, 0, 0, 1]))
    d2c_matrix = {
        'd2c_matrix': extrinsics_matrix.tolist()
    }
    #保存到yaml文件
    with open(package_path + camera_to_depth_to_color_path, 'w') as f:
        yaml.dump(d2c_matrix, f)

def camera_to_depth_to_color_callback(extrinsics):
    rotation = np.array(extrinsics.rotation).reshape(3, 3).tolist()
    translation = np.array(extrinsics.translation).reshape(3, 1).tolist()
    #合并旋转矩阵和平移向量
    extrinsics_matrix = np.hstack((rotation, translation))
    #转换为齐次矩阵
    extrinsics_matrix = np.vstack((extrinsics_matrix, [0, 0, 0, 1]))
    d2c_matrix = {
        'd2c_matrix': extrinsics_matrix.tolist()
    }
    #保存到yaml文件
    with open(package_path + camera_in_depth_to_color_path, 'w') as f:
        yaml.dump(d2c_matrix, f)

def main():
    rospy.init_node('camera_info', anonymous=True)
    rospy.Subscriber(camera_in_color_info_topic, CameraInfo, camera_in_color_info_callback)
    rospy.Subscriber(camera_in_depth_info_topic, CameraInfo, camera_in_depth_info_callback)
    rospy.Subscriber(camera_to_color_info_topic, CameraInfo, camera_to_color_info_callback)
    rospy.Subscriber(camera_to_depth_info_topic, CameraInfo, camera_to_depth_info_callback)
    rospy.Subscriber(camera_in_d2c_topic, Extrinsics, camera_to_depth_to_color_callback)
    rospy.Subscriber(camera_to_d2c_topic, Extrinsics, camera_in_depth_to_color_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()