#!/usr/bin/env python3
# encoding: utf-8

import rospy
import yaml
import sys
import numpy as np
from sensor_msgs.msg import CameraInfo
import os

sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts_to_depth import *
from uilts_to_color import *
from uilts_in_depth import *
from uilts_in_color import *

def camera_in_color_info_callback(camera_info):
    camera_color_info = {
        'frame_id': camera_info.header.frame_id,
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist(),
        'width': camera_info.width,
        'height': camera_info.height
    }
    
    with open(os.getcwd() + camera_in_color_info_path, 'w') as f:
        yaml.dump(camera_color_info, f)

def camera_in_depth_info_callback(camera_info):
    camera_depth_info = {
        'frame_id': camera_info.header.frame_id,
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist(),
        'width': camera_info.width,
        'height': camera_info.height
    }
    
    with open(os.getcwd() + camera_in_depth_info_path, 'w') as f:
        yaml.dump(camera_depth_info, f)
        
def camera_in_ir_info_callback(camera_info):
    camera_ir_info = {
        'frame_id': camera_info.header.frame_id,
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist(),
        'width': camera_info.width,
        'height': camera_info.height
    }
    
    with open(os.getcwd() + camera_in_ir_info_path, 'w') as f:
        yaml.dump(camera_ir_info, f)

def camera_to_color_info_callback(camera_info):
    camera_to_color_info = {
        'frame_id': camera_info.header.frame_id,
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist(),
        'width': camera_info.width,
        'height': camera_info.height
    }
    
    with open(os.getcwd() + camera_to_color_info_path, 'w') as f:
        yaml.dump(camera_to_color_info, f)

def camera_to_depth_info_callback(camera_info):
    camera_to_depth_info = {
        'frame_id': camera_info.header.frame_id,
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist(),
        'width': camera_info.width,
        'height': camera_info.height
    }
    
    with open(os.getcwd() + camera_to_depth_info_path, 'w') as f:
        yaml.dump(camera_to_depth_info, f)

def camera_to_ir_info_callback(camera_info):
    camera_to_ir_info = {
        'frame_id': camera_info.header.frame_id,
        'camera_matrix': np.array(camera_info.K).reshape(3, 3).tolist(),
        'dist_coeffs': np.array(camera_info.D).reshape(-1, 1).tolist(),
        'width': camera_info.width,
        'height': camera_info.height
    }
    
    with open(os.getcwd() + camera_to_ir_info_path, 'w') as f:
        yaml.dump(camera_to_ir_info, f)

def main():
    rospy.init_node('camera_info', anonymous=True)
    rospy.Subscriber(camera_in_color_info_topic, CameraInfo, camera_in_color_info_callback)
    rospy.Subscriber(camera_in_depth_info_topic, CameraInfo, camera_in_depth_info_callback)
    rospy.Subscriber(camera_in_ir_info_topic, CameraInfo, camera_in_ir_info_callback)
    rospy.Subscriber(camera_to_color_info_topic, CameraInfo, camera_to_color_info_callback)
    rospy.Subscriber(camera_to_depth_info_topic, CameraInfo, camera_to_depth_info_callback)
    rospy.Subscriber(camera_to_ir_info_topic, CameraInfo, camera_to_ir_info_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()