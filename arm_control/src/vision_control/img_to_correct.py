#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import os
import sys
import numpy as np
import yaml
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts_to_color import *
from uilts_to_depth import *

dist_coeffs_color = np.zeros((5,1))
dist_coeffs_depth = np.zeros((5,1))
dist_coeffs_ir = np.zeros((5,1))

with open(os.getcwd() + camera_to_color_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_color_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_color = np.array(data['camera_matrix']).reshape((3,3))
        if 'dist_coeffs' in data and data['dist_coeffs']!= []:
            dist_coeffs_color = np.array(data['dist_coeffs']).reshape((5,1))

with open(os.getcwd() + camera_to_depth_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_depth_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_depth = np.array(data['camera_matrix']).reshape((3,3))
        if 'dist_coeffs' in data and data['dist_coeffs']!= []:
            dist_coeffs_depth = np.array(data['dist_coeffs']).reshape((5,1))

with open(os.getcwd() + camera_to_ir_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_ir_img_raw_topic.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_ir = np.array(data['camera_matrix']).reshape((3,3))
        if 'dist_coeffs' in data and data['dist_coeffs']!= []:
            dist_coeffs_ir = np.array(data['dist_coeffs']).reshape((5,1))

def callback(color_img, depth_img, ir_img):
    bridge = CvBridge()
    try:
        color_cv = bridge.imgmsg_to_cv2(color_img, "bgr8")
        depth_cv = bridge.imgmsg_to_cv2(depth_img, "16UC1")
        ir_cv = bridge.imgmsg_to_cv2(ir_img, "16UC1")
    except CvBridgeError as e:
        print(e)
        
    color_cv = cv2.undistort(color_cv, camera_matrix_color, dist_coeffs_color)
    depth_cv = cv2.undistort(depth_cv, camera_matrix_depth, dist_coeffs_depth)
    ir_cv = cv2.undistort(ir_cv, camera_matrix_ir, dist_coeffs_ir)

    color_pub = rospy.Publisher(camera_to_color_img_corrected_topic, Image, queue_size=1)
    depth_pub = rospy.Publisher(camera_to_depth_img_corrected_topic, Image, queue_size=1)
    ir_pub = rospy.Publisher(camera_to_ir_img_corrected_topic, Image, queue_size=1)
    
    color_msg = bridge.cv2_to_imgmsg(color_cv, "bgr8")
    depth_msg = bridge.cv2_to_imgmsg(depth_cv, "16UC1")
    ir_msg = bridge.cv2_to_imgmsg(ir_cv, "16UC1")
    
    color_pub.publish(color_msg)
    depth_pub.publish(depth_msg)
    ir_pub.publish(ir_msg)


def main():
    rospy.init_node('img_to_correct', anonymous=True)
    color_sub = message_filters.Subscriber(camera_to_color_img_raw_topic, Image)
    depth_sub = message_filters.Subscriber(camera_to_depth_img_raw_topic, Image)
    ir_sub = message_filters.Subscriber(camera_to_ir_img_raw_topic, Image)

    ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub, ir_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    
    rospy.spin()
    
if __name__ == '__main__':
    main()