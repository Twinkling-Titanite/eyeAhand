#!/usr/bin/env python3
# encoding: utf-8

import rospy
import roslib
import sys
import yaml
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from utils import *
from utils_in_color import *
from utils_to_color import *
import files_op

if handeye == 'to':
    camera_color_img_raw_topic = camera_to_color_img_raw_topic
    camera_color_info_path = package_path + camera_to_color_info_path
    camera_color_calib_img_path = package_path + camera_to_color_calib_img_path
    camera_color_calib_result_path = package_path + camera_to_color_calib_result_path
else:
    if handeye == 'in':
        camera_color_img_raw_topic = camera_in_color_img_raw_topic
        camera_color_info_path = package_path + camera_in_color_info_path
        camera_color_calib_img_path = package_path + camera_in_color_calib_img_path
        camera_color_calib_result_path = package_path + camera_in_color_calib_result_path
    else:
        print("Please choose handeye in or to")

# 准备棋盘格的3D点，如 (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

# 用来存储所有图像的3D点和2D点
objpoints = []  # 3D点
imgpoints = []  # 2D点
cv_image = None

files_op.clear_folder(camera_color_calib_result_path)

def callback(data):
    global cv_image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('get_camera_info', anonymous=True)
    rospy.Subscriber(camera_color_img_raw_topic, Image, callback)
    global cv_image
    counter = 0
    while not rospy.is_shutdown():
        user_input = input("Press s to save image, c to calibrate, d to delete all images, or q to quit: ")
        
        if user_input == 'q':
            break
        
        elif user_input == 's':
            if cv_image is not None:
                cv2.imwrite(camera_color_calib_img_path + str(counter) + '.jpg', cv_image)
                counter += 1
                
        elif user_input == 'd':
            files_op.clear_folder(camera_color_calib_img_path)
            counter = 0
            
        elif user_input == 'c':
            img_num = files_op.get_img_num(camera_color_calib_img_path)
            for i in range(0, img_num):
                img = cv2.imread(camera_color_calib_img_path + str(i) + '.jpg')
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
                
                if ret:
                    objpoints.append(objp)
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    imgpoints.append(corners)
                    img_result = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
                    cv2.imwrite(camera_color_calib_result_path + str(i) + '.jpg', img_result)
                else:
                    print("Can't find chessboard in image " + str(i) + ".jpg")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            print("Camera matrix:\n", mtx)
            print("Distortion coefficients:\n", dist)
            camera_color_info = {
                'camera_matrix': np.array(mtx).reshape(3, 3).tolist(),
                'dist_coeffs': np.array(dist).reshape(-1, 1).tolist()
            }
            
            with open(camera_color_info_path, 'w') as f:
                yaml.dump(camera_color_info, f)
            
            
if __name__ == '__main__':
    main()