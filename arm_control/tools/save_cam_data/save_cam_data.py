#!/usr/bin/env python3
# encoding: utf-8

import cv2
import open3d as o3d
import sys
import os
import glob
import numpy as np
import rospy
import roslib
import yaml
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from files_op import get_img_num

with open(package_path + '/config/camera_in/color/camera_color_info.yaml', 'r') as f:
    camera_info = yaml.safe_load(f)
    camera_matrix = np.array(camera_info['camera_matrix']).reshape(3, 3)

image = None
depth = None
cloud = None

save_path = package_path + '/tools/save_cam_data'

def main():
    rospy.init_node('save_cam_data', anonymous=True)
    
    global image, depth, cloud
    
    img_sub = message_filters.Subscriber('/camera_in/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera_in/aligned_depth_to_color/image_raw', Image)
    ts = message_filters.ApproximateTimeSynchronizer([img_sub, depth_sub], 1, 0.01, allow_headerless=True)
    ts.registerCallback(callback)
    
    count = get_img_num(save_path + '/color/')
    
    while not rospy.is_shutdown():
        user_input = input("Press q to quit, s to save data, d to delete data, D to delete all data: ")
        if user_input == 'q':
            break
        elif user_input =='s':
            cv2.imwrite(save_path + '/color/color' + str(count) + '.jpg', image)
            cv2.imwrite(save_path + '/depth/depth' + str(count) + '.png', depth)
            height, width = depth.shape[:2]
            points = []
            for v in range(height):
                for u in range(width):
                    Z = depth[v, u] / 1000.0
                    if Z == 0:
                        continue
                    X = (u - camera_matrix[0, 2]) * Z / camera_matrix[0, 0]
                    Y = (v - camera_matrix[1, 2]) * Z / camera_matrix[1, 1]
                    points.append([X, Y, Z])
                
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(save_path + '/cloud/cloud' + str(count) + '.pcd', cloud)
            count += 1
        elif user_input == 'd':
            if count > 0:
                count -= 1
                os.remove(save_path + '/color/color' + str(count) + '.jpg')
                os.remove(save_path + '/depth/depth' + str(count) + '.png')
                os.remove(save_path + '/cloud/cloud' + str(count) + '.pcd')
                print("Deleted data " + str(count))
            else:
                print("No data to delete")
        elif user_input == 'D':
            jpg_files = glob.glob(save_path + '/color/*.jpg')
            for file in jpg_files:
                os.remove(file)
            png_files = glob.glob(save_path + '/depth/*.png')
            for file in png_files:
                os.remove(file)
            pcd_files = glob.glob(save_path + '/cloud/*.pcd')
            for file in pcd_files:
                os.remove(file)
            count = 0
            print("Deleted all data")
        else:
            print("Invalid input")

def callback(img_msg, depth_msg):
    global image, depth, cloud
    try:
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    
        
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    main()