#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import os
import sys
import yaml
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from uilts import *
from uilts_to_color import *
from uilts_in_color import *

if handeye == 'in':
    camera_color_info_path = os.getcwd() + camera_in_color_info_path
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    
else:
    if handeye == 'to':
        camera_color_info_path = os.getcwd() + camera_to_color_info_path
        camera_color_img_corrected_topic = camera_to_color_img_corrected_topic
    else:
        print("Please choose handeye in or to")

point_pub = None

with open(camera_color_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_color_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_color = np.array(data['camera_matrix']).reshape((3,3))

def image_callback(color_msg, depth_msg):
    global  point_pub
    bridge = CvBridge()
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_depth = bridge.imgmsg_to_cv2(depth_msg, "16UC1")
    except CvBridgeError as e:
        print(e)
        
    # 将图像从BGR转换为HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 定义红色的HSV范围
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

    lower_red = np.array([170, 120, 70])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

    # 合并两个掩码
    mask = mask1 + mask2
    
    # 查找红色物体的轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        # 忽略小的轮廓
        if cv2.contourArea(contour) > 50:
            # 获取物体的中心点
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # 发布器，用于发布红色物体的像素坐标
                point_pub = rospy.Publisher(red_object_position_topic, PointStamped, queue_size=1)
                    
                # 创建并发布 PointStamped 消息
                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = 'red_object_frame'
                point_msg.point.x = cX
                point_msg.point.y = cY
                point_msg.point.z = 0 

                point_pub.publish(point_msg)

    cv2.imshow("Image window", mask)
    cv2.waitKey(1)

def main():
    global point_pub
    rospy.init_node('red_detect', anonymous=True)
    color_sub = message_filters.Subscriber(camera_color_img_corrected_topic, Image, image_callback, queue_size=1)
    
    rospy.loginfo("Red object detector node started.")
    rospy.spin()

if __name__ == "__main__":
    main()
