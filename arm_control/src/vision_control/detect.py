#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import os
import sys
import yaml
import roslib
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from utils import *
from utils_to_color import *
from utils_in_color import *

if handeye == 'in':
    camera_color_info_path = package_path + camera_in_color_info_path
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    
else:
    if handeye == 'to':
        camera_color_info_path = package_path + camera_to_color_info_path
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

def image_callback(color_msg):
    global  point_pub
    bridge = CvBridge()
    try:
        # 将ROS图像消息转换为OpenCV图像
        image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # 转换为灰度图像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 使用高斯模糊去除噪声
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 使用自适应阈值法进行二值化
    _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    # kernel_open = np.ones((5, 5), np.uint8)
    # kernel_close = np.ones((7, 7), np.uint8)
    # open = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel_open)
    # close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel_close)

    # 边缘检测
    # edges = cv2.Canny(close, 20, 100)
    # edges_colored = np.zeros_like(image)
    # edges_colored[edges != 0] = [0, 255, 0]
    # overlay = cv2.addWeighted(image, 1, edges_colored, 1, 0)

    # 查找红色物体的轮廓
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        # 忽略小的轮廓
        if cv2.contourArea(contour) > 50 and cv2.contourArea(contour) < 1000:           
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0, 255, 255), 2)
            
            distance_1 = np.linalg.norm(box[1] - box[0])
            distance_2 = np.linalg.norm(box[2] - box[1])    
            if distance_1 > distance_2:
                cX_1 = int((box[0][0] + box[3][0]) / 2)
                cY_1 = int((box[0][1] + box[3][1]) / 2)
                cX_2 = int((box[1][0] + box[2][0]) / 2)
                cY_2 = int((box[1][1] + box[2][1]) / 2)
                
                x_1 = int(cX_1*0.9 + cX_2*0.1)
                y_1 = int(cY_1*0.9 + cY_2*0.1)
                x_2 = int(cX_1*0.1 + cX_2*0.9)
                y_2 = int(cY_1*0.1 + cY_2*0.9)
            else:
                cX_1 = int((box[0][0] + box[1][0]) / 2)
                cY_1 = int((box[0][1] + box[1][1]) / 2)
                cX_2 = int((box[2][0] + box[3][0]) / 2)
                cY_2 = int((box[2][1] + box[3][1]) / 2)
                
                x_1 = int(cX_1*0.9 + cX_2*0.1)
                y_1 = int(cY_1*0.9 + cY_2*0.1)
                x_2 = int(cX_1*0.1 + cX_2*0.9)
                y_2 = int(cY_1*0.1 + cY_2*0.9)
            
            cv2.circle(image, (x_1, y_1), 5, (0, 0, 0), -1)
            cv2.circle(image, (x_2, y_2), 5, (0, 0, 0), -1)

            # 发布器，用于发布红色物体的像素坐标
            point_pub_1 = rospy.Publisher(red_poistion_1_topic, PointStamped, queue_size=1)
            point_pub_2 = rospy.Publisher(red_poistion_2_topic, PointStamped, queue_size=1)
                    
            # 创建并发布 PointStamped 消息
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = 'red_object_1_frame'
            point_msg.point.x = x_1
            point_msg.point.y = y_1
            point_msg.point.z = 0

            point_pub_1.publish(point_msg)

            point_msg.header.frame_id = 'red_object_2_frame'
            point_msg.point.x = x_2
            point_msg.point.y = y_2
            point_msg.point.z = 0
            
            point_pub_2.publish(point_msg)

    cv2.imshow("image", image)
    cv2.waitKey(5)
    
def main():
    global point_pub
    rospy.init_node('red_object_pose_find', anonymous=True)
    rospy.Subscriber(camera_to_color_img_raw_topic, Image, image_callback)
    
    rospy.loginfo("Red object pose find node started.")
    rospy.spin()

if __name__ == "__main__":
    main()
