#!/usr/bin/env python3
# encoding: utf-8

import rospy
import numpy as np
import yaml
import sys
import roslib
import message_filters
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import moveit_commander
import tf

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from utils import *
from utils_in_color import *
from utils_in_depth import *

from get_tf import get_world_coordinates, get_depth

group = moveit_commander.MoveGroupCommander(group_arm_name)

with open(package_path + camera_in_color_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_color_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_to_matrix_color = np.array(data['camera_matrix']).reshape((3,3))

with open(package_path + camera_in_depth_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_ir_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_to_matrix_depth = np.array(data['camera_matrix']).reshape((3,3))

with open(package_path + camera_in_ir_info_path, 'r') as f:
    data = yaml.safe_load(f)
    if data is None:
        print("camera_ir_info.yaml is empty")
    else:
        if 'camera_matrix' in data:
            camera_to_matrix_ir = np.array(data['camera_matrix']).reshape((3,3))

# 发布器，用于发布世界坐标
pose_pub = rospy.Publisher(world_coord_topic, PoseStamped, queue_size=1)

def img2world_callback(position_1_msg, position_2_msg):
    global depth_img
    pixel_1_x = int(position_1_msg.point.x)
    pixel_1_y = int(position_1_msg.point.y)
    pixel_2_x = int(position_2_msg.point.x)
    pixel_2_y = int(position_2_msg.point.y)
    
    if correct_depth_camera:
    # 打开相机到机械臂末端坐标系的转换矩阵
        with open(package_path + cam2end_color_tf_path, 'r') as f:
            T_cam_color_to_end = np.array(yaml.load(f)['cam2end_tf_matrix'])
        with open(package_path + cam2end_ir_tf_path, 'r') as f:
            T_cam_ir_to_end = np.array(yaml.load(f)['cam2end_tf_matrix'])
    else:
        # 打开相机到世界坐标系的转换矩阵
        with open(package_path + cam2end_color_tf_path, 'r') as f:
            T_cam_color_to_end = np.array(yaml.load(f)['cam2end_tf_matrix'])
        with open(package_path + camera_in_depth_to_color_path, 'r') as f:
            T_cam_depth_to_color = np.array(yaml.load(f)['d2c_matrix'])
        T_cam_ir_to_end = T_cam_color_to_end.dot(T_cam_depth_to_color)
    
    end_pose = group.get_current_pose().pose
    end_R_matrix = tf.transformations.quaternion_matrix([end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w])[:3,:3]
    end_t_vector = np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z])
    T_end_to_world = np.vstack((np.hstack((end_R_matrix, end_t_vector.reshape((3,1)))), np.array([0,0,0,1])))
    
    T_cam_color_to_world = T_end_to_world.dot(T_cam_color_to_end)
    T_cam_ir_to_world = T_end_to_world.dot(T_cam_ir_to_end)
    
    depth_1, p_depth_x_1, p_depth_y_1 = get_depth(pixel_1_x, pixel_1_y, depth_img, camera_to_matrix_color, camera_to_matrix_depth, T_cam_color_to_world, T_cam_ir_to_world)
    depth_2, p_depth_x_2, p_depth_y_2 = get_depth(pixel_2_x, pixel_2_y, depth_img, camera_to_matrix_color, camera_to_matrix_depth, T_cam_color_to_world, T_cam_ir_to_world)
    print("depth_1: ", depth_1)
    print("depth_2: ", depth_2)
    
    
    world_coords_1 = get_world_coordinates(p_depth_x_1, p_depth_y_1, depth_1, camera_to_matrix_depth, T_cam_ir_to_world)
    world_coords_2 = get_world_coordinates(p_depth_x_2, p_depth_y_2, depth_2, camera_to_matrix_depth, T_cam_ir_to_world)
    
    world_coords = (world_coords_1 + world_coords_2) / 2
    
    d_12 = (world_coords_2 - world_coords_1) / np.linalg.norm(world_coords_2 - world_coords_1)
    angle_rad = np.arccos(np.dot(d_12, np.array([1, 0, 0])))
    print("angle: ", angle_rad/np.pi*180)
    
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "world"
    pose_msg.pose.position.x = world_coords[0]
    pose_msg.pose.position.y = world_coords[1] + 0.14
    pose_msg.pose.position.z = world_coords[2]
    pose_msg.pose.orientation.x = 0
    pose_msg.pose.orientation.y = 0.707
    pose_msg.pose.orientation.z = -0.707
    pose_msg.pose.orientation.w = 0
    
    pose_pub.publish(pose_msg)
    
    rospy.loginfo("World coordinates: {}".format(world_coords))

def depth_img_callback(depth_msg):
    global depth_img
    try:
        bridge = CvBridge()
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)
        return

def main():
    rospy.init_node('hand_to_eye_img2world')
    # 订阅红色物体的像素坐标
    position_sub_1 = message_filters.Subscriber(red_poistion_1_topic, PointStamped, queue_size=1)
    position_sub_2 = message_filters.Subscriber(red_poistion_2_topic, PointStamped, queue_size=1)
    rospy.wait_for_message(red_poistion_1_topic, PointStamped, timeout=None)
    rospy.wait_for_message(red_poistion_2_topic, PointStamped, timeout=None)
    rospy.Subscriber(camera_in_depth_img_corrected_topic, Image, depth_img_callback, queue_size=1)
    
    ts = message_filters.ApproximateTimeSynchronizer([position_sub_1, position_sub_2], 10, 0.1, allow_headerless=True)
    ts.registerCallback(img2world_callback)
    
    # 保持节点运行
    rospy.spin()  

if __name__ == "__main__":
    main()
