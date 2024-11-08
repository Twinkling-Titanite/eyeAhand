#!/usr/bin/env python3
# encoding: utf-8

import rospy
import roslib
import sys
package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/srv/')

from GetPose import *


def request_pose():
    rospy.init_node('pose_service_client')
    
    # 等待服务可用
    rospy.wait_for_service('get_pose')
    
    try:
        # 创建服务代理
        get_pose = rospy.ServiceProxy('get_pose', GetPose)
        
        # 发送请求并获取响应
        response = get_pose()
        
        pose = response.pose
        
        rospy.loginfo("Received pose: position(%f, %f, %f), orientation(%f, %f, %f, %f)",
                      pose.position.x, pose.position.y, pose.position.z,
                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    request_pose()
