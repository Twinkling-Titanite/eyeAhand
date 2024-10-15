#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
import sys
import yaml
import roslib
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from utils import *
from utils_to_color import *
from utils_in_color import *
from get_tf import *

if handeye == 'in':
    camera_color_info_path = package_path + camera_in_color_info_path
    camera_color_img_corrected_topic = camera_in_color_img_corrected_topic
    
else:
    if handeye == 'to':
        camera_color_info_path = package_path + camera_to_color_info_path
        camera_color_img_corrected_topic = camera_to_color_img_corrected_topic
    else:
        print("Please choose handeye in or to")

with open(camera_color_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_color_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_matrix_color = np.array(data['camera_matrix']).reshape((3,3))
            
with open(package_path + cam2world_color_tf_path, 'r') as f:
    T_cam_color_to_world = np.array(yaml.load(f)['cam2world_tf_matrix'])
 
# 准备棋盘格的3D点，例如 (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
object_points = np.zeros((np.prod(chessboard_size), 3), np.float32)
object_points[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2)
object_points *= square_size

cv_image = None

def main():
    rospy.init_node('chessboard_find', anonymous=True)
    
    rospy.Subscriber(camera_color_img_corrected_topic, Image, image_callback, queue_size=10)
    
    all_corners = []
    all_chessboard_in_world = []
    all_cam_idx_size = []
    
    res_corner = []
    res_chessboard_in_world = []
    res_cam_idx_size = []
    
    while not rospy.is_shutdown():
        user_input = input("press q to quit, press s to save the chessboard position: ")
        if user_input == 'q':
            break
        elif user_input =='s':

            for i in range(1, 100):
                one_corner, chessboard_in_world, cam_idx_size = find_chessboard()
                if one_corner is None:
                    print("chessboard not found")
                    break
                all_corners.append(one_corner)
                all_chessboard_in_world.append(chessboard_in_world)
                all_cam_idx_size.append(cam_idx_size)
            
            if all_corners == []:
                continue
                            
            res_corner.append(np.mean(all_corners, axis=0))
            res_chessboard_in_world.append(np.mean(all_chessboard_in_world, axis=0))
            res_cam_idx_size.append(np.mean(all_cam_idx_size, axis=0))
            
            all_corners = []
            all_chessboard_in_world = []
            all_cam_idx_size = []
            
            print("one corner:" + str(res_corner[len(res_corner)-1]))
            print("chessboard in world:" + str(res_chessboard_in_world[len(res_chessboard_in_world)-1]))
            print("cam_idx_size:" + str(res_cam_idx_size[len(res_cam_idx_size)-1]))       
                
            if len(res_corner) > 1:
                print("corner diff to last: " + str(np.linalg.norm(res_corner[len(res_corner)-1] - res_corner[len(res_corner)-2])))
                print("chessboard in world diff to last (m): " + str(np.linalg.norm(res_chessboard_in_world[len(res_chessboard_in_world)-1] - res_chessboard_in_world[len(res_chessboard_in_world)-2])))
                print("corner diff to last (mm): " + str(np.linalg.norm(res_corner[len(res_corner)-1] - res_corner[len(res_corner)-2]) * res_cam_idx_size[len(res_cam_idx_size)-1]))
                
def find_chessboard():
    global cv_image
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        #细化角点
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        luck_one = 1
        luck_two = 2
        one_corner = corners2[luck_one][0]
        
        cam_idx_size = square_size/((corners2[luck_one][0][0] - corners2[luck_two][0][0])**2 + (corners2[luck_one][0][1] - corners2[luck_two][0][1])**2)**0.5
        
        #棋盘格到相机坐标系的转换
        dist_coeffs = np.zeros((5, 1))
        ret, rvec, tvec = cv2.solvePnP(object_points, corners2, camera_matrix_color, dist_coeffs)
        #旋转向量转化为旋转矩阵
        rmat, jac = cv2.Rodrigues(rvec)
        # 齐次坐标矩阵
        T_chessboard_to_cam = np.hstack((rmat, tvec * 0.001))
        T_chessboard_to_cam = np.vstack((T_chessboard_to_cam, [0, 0, 0, 1]))
        
        T_chessboard_to_world = np.dot(T_cam_color_to_world, T_chessboard_to_cam)
        
        # 计算棋盘格在世界坐标系中的位置
        chessboard_in_world = np.dot(T_chessboard_to_world, np.array([0, 0, 0, 1]))[:3]

        return one_corner, chessboard_in_world, cam_idx_size
    else:
        return None, None, None
        
def image_callback(msg):
    global cv_image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
    except CvBridgeError as e:
        print(e)

    
        
if __name__ == '__main__':
    main()

