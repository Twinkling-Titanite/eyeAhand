#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
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
from get_tf import *
from motor_API import *

while not rospy.is_shutdown():
    eyehand = input("Please choose eyehand in or to: ")
    if eyehand not in ['in', 'to']:
        print("Invalid input!")
        continue
    break

if eyehand == 'in':
    camera_color_info_path = package_path + camera_in_color_info_path
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    
else:
    if eyehand == 'to':
        camera_color_info_path = package_path + camera_to_color_info_path
        camera_color_img_corrected_topic = camera_to_color_img_corrected_topic

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

cv_image = None
depth_img = None
p_x = None
p_y = None
     
def find_chessboard():
    global depth_img, p_x, p_y
    pixel = np.array([p_x, p_y, 1]).reshape((3,1))
    chessboard_in_cam = np.dot(np.linalg.inv(camera_matrix_color), pixel) * depth_img[p_y, p_x] / 1000
    return chessboard_in_cam
        
def image_callback(msg):
    global cv_image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
    except CvBridgeError as e:
        print(e)

def point_callback(msg):
    global p_x, p_y
    p_x = int(msg.point.x)
    p_y = int(msg.point.y)

def depth_img_callback(depth_msg):
    global depth_img
    try:
        bridge = CvBridge()
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)
        return

def main():
    global p_x, p_y, depth_img
    rospy.init_node('xyz_eAh', anonymous=True)
    rospy.Subscriber(camera_color_img_corrected_topic, Image, image_callback)
    rospy.Subscriber(object_poistion_1_topic, PointStamped, point_callback)
    rospy.Subscriber(camera_in_depth_img_raw_topic, Image, depth_img_callback)
    x = SerialConnection(id=1)
    x.connect()
    x.clean_error()
    y = SerialConnection(id=2)
    y.connect()
    y.clean_error()
    z = SerialConnection(id=3)
    z.connect()
    z.clean_error()
    
    motors_init_mode(x, y, z)
    motors_enable(x, y, z)
    # motors_init(x, y, z, wait_time=5)
    # print(motors_position(x, y, z))
    
    p1_x = 0.05
    p1_y = 0.0
    p1_z = 0.0
    p2_x = 0.0
    p2_y = 0.05
    p2_z = 0.0
    
    rmat = np.array([[-1, 0, 0],[0, -1, 0],[0, 0, 1]])
    
    while not rospy.is_shutdown():
        user_input = input("press q to quit, r to get rmat, t to get tvec: ")
        if user_input == 'q':
            break
        elif user_input =='r':
            motors_move(x, y, z, [0.2, 0.2, 0.15])
            chessboard_in_cam_1 = find_chessboard()
            if chessboard_in_cam_1 is not None:
                motors_move(x, y, z, [0.2 + p1_x, 0.2 + p1_y, 0.15 + p1_z])
                chessboard_in_cam_2 = find_chessboard()
                if chessboard_in_cam_2 is not None:
                    motors_move(x, y, z, [0.2, 0.2, 0.15])
                    motors_move(x, y, z, [0.2 + p2_x, 0.2 + p2_y, 0.15 + p2_z])
                    chessboard_in_cam_3 = find_chessboard()
                    if chessboard_in_cam_3 is not None:
                        # 叉乘得到旋转矩阵
                        M_world = np.array([[p1_x, p1_y, p1_z]/np.linalg.norm([p1_x, p1_y, p1_z]),[p2_x, p2_y, p2_z]/np.linalg.norm([p2_x, p2_y, p2_z]),np.cross([p1_x, p1_y, p1_z]/np.linalg.norm([p1_x, p1_y, p1_z]),[p2_x, p2_y, p2_z]/np.linalg.norm([p2_x, p2_y, p2_z]))])
                        print("M_world:", M_world)
                        M_world = M_world.T
                        Pc_1 = (chessboard_in_cam_2 - chessboard_in_cam_1)/np.linalg.norm(chessboard_in_cam_2 - chessboard_in_cam_1)
                        Pc_2 = (chessboard_in_cam_3 - chessboard_in_cam_1)/np.linalg.norm(chessboard_in_cam_3 - chessboard_in_cam_1)
                        Pc_3 = np.cross(Pc_1, Pc_2)/np.linalg.norm(np.cross(Pc_1, Pc_2))
                        Pc_1 = Pc_1.reshape((3,1))
                        Pc_2 = Pc_2.reshape((3,1))
                        Pc_3 = Pc_3.reshape((3,1))
                        M_cam = np.hstack((Pc_1, Pc_2, Pc_3))
                        rmat = np.dot(M_world, np.linalg.inv(M_cam))
                        print("rmat:", rmat)
                    else:
                        print("chessboard not found!")
                else:
                    print("chessboard not found!")
            else:
                print("chessboard not found!")
        elif user_input == 't':
            motors_move(x, y, z, [0.2, 0.2, 0.01])
            user_input = input("press 1 to conutinue")
            if user_input == '1':
                motors_move(x, y, z, [0.2, 0.2, 0.2])
                chessboard_in_cam = find_chessboard()
                print("chessboard_in_cam:", chessboard_in_cam)
                chessboard_in_tool = np.array([0.0, 0.0, 0.19]).reshape((3,1))
                tvec = chessboard_in_tool - np.dot(rmat, chessboard_in_cam)
                print("tvec:", tvec)
        
        elif user_input == 'p':
            motors_move(x, y, z, [0.3, 0.15, 0.2])
            print("p_x:", p_x, "p_y:", p_y)
            pixel = np.array([p_x, p_y, 1]).reshape((3,1))
            print("pixel:", pixel)
            chessboard_in_cam = np.dot(np.linalg.inv(camera_matrix_color), pixel) * depth_img[p_y, p_x] / 1000
            chessboard_in_tool = np.dot(rmat, chessboard_in_cam).T + np.array([0.005, 0.02711269, -0.11])
            chessboard_in_world = (np.array(motors_position(x, y, z)) - chessboard_in_tool).reshape(-1,3)
            print("chessboard_in_world:", chessboard_in_world)
            motors_move(x, y, z, [chessboard_in_world[0][0], chessboard_in_world[0][1], chessboard_in_world[0][2]])
        
        else:
            print("Invalid input!")
        
            
        
    motors_disable(x, y, z)
    x.close()
    y.close()
    z.close()

if __name__ == '__main__':
    main()