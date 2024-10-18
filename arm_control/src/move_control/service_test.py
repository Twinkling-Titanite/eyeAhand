#!/usr/bin/env python3
# encoding: utf-8

import rospy
from std_srvs.srv import SetBool

if __name__ == "__main__":
    rospy.init_node("service_test")
    service = rospy.ServiceProxy('gripper_pick', SetBool)
    response = service(False)
    rospy.spin()