#!/usr/bin/env python3
# encoding: utf-8

import rospy
import roslib
import sys
from geometry_msgs.msg import Pose

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/srv/')

from GetPose import *

def handle_pose_request(req):
    # 创建一个Pose对象
    pose = Pose()
    
    # 设置位置 (x, y, z)
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    
    # 设置方向 (quaternion)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    
    # 打印并返回Pose对象
    rospy.loginfo("Sending pose: position(%f, %f, %f), orientation(%f, %f, %f, %f)",
                  pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    
    return GetPoseResponse(pose)

def pose_service_server():
    rospy.init_node('pose_service_server')
    
    # 创建服务 'get_pose'，使用自定义服务类型 GetPose
    s = rospy.Service('get_pose', GetPose, handle_pose_request)
    
    rospy.loginfo("Pose service is ready.")
    rospy.spin()

if __name__ == "__main__":
    pose_service_server()