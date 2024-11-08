#!/usr/bin/env python3
# encoding: utf-8

import rospy
import numpy as np
import yaml
import sys
import roslib
import message_filters
from geometry_msgs.msg import PointStamped, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import moveit_commander
import tf

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')
sys.path.append(package_path + '/scripts/srv/')

from utils import *

from GetPose import *

from get_tf import get_world_coordinates
from move import rpy2quaternion

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

depth_img = None
pixel_1_x = None
pixel_2_x = None
pixel_1_y = None
pixel_2_y = None

object_exists = False
pose_no = Pose()
pose_no.position.x = 0
pose_no.position.y = 0
pose_no.position.z = 0
pose_no.orientation.x = 0
pose_no.orientation.y = 0
pose_no.orientation.z = 0
pose_no.orientation.w = 1

def get_pose_service_handler(req):
    global pixel_1_x, pixel_2_x, pixel_1_y, pixel_2_y, depth_img, object_exists

    if not object_exists:
        success = False
        return GetPoseResponse(success, pose_no)
    
    depth_1 = depth_img[pixel_1_y, pixel_1_x]/1000
    depth_2 = depth_img[pixel_2_y, pixel_2_x]/1000
    
    if depth_1 == 0 or depth_2 == 0:
        success = False
        return GetPoseResponse(success, pose_no)
    
    print("depth_1: ", depth_1)
    print("depth_2: ", depth_2)
    
    # 打开相机到机械臂末端坐标系的转换矩阵
    with open(package_path + cam2end_color_tf_path, 'r') as f:
        T_cam_color_to_end = np.array(yaml.safe_load(f)['cam2end_tf_matrix'])
        
    end_pose = group.get_current_pose().pose
    end_R_matrix = tf.transformations.quaternion_matrix([end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w])[:3,:3]
    end_t_vector = np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z])
    T_end_to_world = np.vstack((np.hstack((end_R_matrix, end_t_vector.reshape((3,1)))), np.array([0,0,0,1])))
    
    T_cam_color_to_world = T_end_to_world.dot(T_cam_color_to_end)
    
    world_coords_1 = get_world_coordinates(pixel_1_x, pixel_1_y, depth_1, camera_to_matrix_color, T_cam_color_to_world)
    world_coords_2 = get_world_coordinates(pixel_2_x, pixel_2_y, depth_2, camera_to_matrix_color, T_cam_color_to_world)
    
    world_coords = (world_coords_1 + world_coords_2) / 2
    
    d_12 = (world_coords_2 - world_coords_1) / np.linalg.norm(world_coords_2 - world_coords_1)
    angle_rad = np.arccos(np.dot(d_12, np.array([1, 0, 0])))
    print("angle: ", angle_rad/np.pi*180)
    
    pose = Pose()
    pose.position.x = world_coords[0]
    pose.position.y = world_coords[1]
    pose.position.z = world_coords[2]
    pose.orientation = rpy2quaternion(0, np.pi, (np.pi - angle_rad))
    
    print("World coordinates: {}".format(pose))
    
    success = object_exists
    object_exists = False
    
    return GetPoseResponse(success, pose)

def img2world_callback(position_1_msg, position_2_msg):
    global pixel_1_x, pixel_2_x, pixel_1_y, pixel_2_y, object_exists
    pixel_1_x = int(position_1_msg.point.x)
    pixel_1_y = int(position_1_msg.point.y)
    pixel_2_x = int(position_2_msg.point.x)
    pixel_2_y = int(position_2_msg.point.y)
    object_exists = True

def depth_img_callback(depth_msg):
    global depth_img
    try:
        bridge = CvBridge()
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)
        return

def main():
    rospy.init_node('eye_in_hand_img2world')
    
    rospy.wait_for_message(object_poistion_1_topic, PointStamped, timeout=None)
    rospy.wait_for_message(object_poistion_2_topic, PointStamped, timeout=None)
    rospy.wait_for_message(camera_in_depth_img_corrected_topic, Image, timeout=None)
    # 订阅红色物体的像素坐标
    position_sub_1 = message_filters.Subscriber(object_poistion_1_topic, PointStamped, queue_size=1)
    position_sub_2 = message_filters.Subscriber(object_poistion_2_topic, PointStamped, queue_size=1)
    depth_sub = rospy.Subscriber(camera_in_depth_img_corrected_topic, Image, depth_img_callback, queue_size=1)
    
    ts = message_filters.ApproximateTimeSynchronizer([position_sub_1, position_sub_2], 1, 1e-5, allow_headerless=False)
    ts.registerCallback(img2world_callback)
    
    rospy.Service('get_pose', GetPose, get_pose_service_handler)
    
    # 保持节点运行
    rospy.spin()  

if __name__ == "__main__":
    main()
