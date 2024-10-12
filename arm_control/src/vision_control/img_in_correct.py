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

home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')

sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts_in_color import *
from uilts_in_depth import *

dist_coeffs_color = np.zeros((5,1))
dist_coeffs_depth = np.zeros((5,1))
dist_coeffs_ir = np.zeros((5,1))

with open(os.getcwd() + camera_in_color_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_color_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_color = np.array(data['camera_matrix']).reshape((3,3))
        if 'dist_coeffs' in data and data['dist_coeffs']!= []:
            dist_coeffs_color = np.array(data['dist_coeffs']).reshape((-1,1))

with open(os.getcwd() + camera_in_depth_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_depth_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_depth = np.array(data['camera_matrix']).reshape((3,3))
        if 'dist_coeffs' in data and data['dist_coeffs']!= []:
            dist_coeffs_depth = np.array(data['dist_coeffs']).reshape((-1,1))

with open(os.getcwd() + camera_in_ir_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_ir_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_ir = np.array(data['camera_matrix']).reshape((3,3))
        if 'dist_coeffs' in data and data['dist_coeffs']!= []:
            dist_coeffs_ir = np.array(data['dist_coeffs']).reshape((-1,1))

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
    
    # cv2.imshow("color_cv", color_cv)
    # cv2.imshow("depth_cv", depth_cv)
    # cv2.imshow("ir_cv", ir_cv)
    # cv2.waitKey(1)
    
    color_pub = rospy.Publisher(camera_in_color_img_corrected_topic, Image, queue_size=1)
    depth_pub = rospy.Publisher(camera_in_depth_img_corrected_topic, Image, queue_size=1)
    ir_pub = rospy.Publisher(camera_in_ir_img_corrected_topic, Image, queue_size=1)
    
    color_msg = bridge.cv2_to_imgmsg(color_cv, "bgr8")
    depth_msg = bridge.cv2_to_imgmsg(depth_cv, "16UC1")
    ir_msg = bridge.cv2_to_imgmsg(ir_cv, "16UC1")
    
    color_pub.publish(color_msg)
    depth_pub.publish(depth_msg)
    ir_pub.publish(ir_msg)


def main():
    rospy.init_node('img_in_correct', anonymous=True)
    color_sub = message_filters.Subscriber(camera_in_color_img_raw_topic, Image)
    depth_sub = message_filters.Subscriber(camera_in_depth_img_raw_topic, Image)
    ir_sub = message_filters.Subscriber(camera_in_ir_img_raw_topic, Image)
    ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub, ir_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()
    
if __name__ == '__main__':
    main()