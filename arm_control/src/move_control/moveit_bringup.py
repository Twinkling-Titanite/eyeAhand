#!/usr/bin/env python3
# encoding: utf-8

import rospy
import sys
import re
import roslib
from numpy import pi
import actionlib
from std_srvs.srv import SetBool, SetBoolResponse
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState

package_path = roslib.packages.get_pkg_dir('arm_control')
sys.path.append(package_path + '/scripts/')

from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType

class ArmControllerActionServer:
    def __init__(self):
        self.dashboard, self.move, self.feed = self.connect_robot()
        # self.dashboard.ClearError() 
        # self.dashboard.DisableRobot() 
 
        # 创建 Action 服务端
        self.server = actionlib.SimpleActionServer('/arm_position_controllers/follow_joint_trajectory',
                                                    FollowJointTrajectoryAction,
                                                    self.execute_callback,
                                                    False)

        # 发布 JointState 消息的发布者，用于反馈机械臂状态
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # 机械臂关节的名字和初始状态
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.joint_positions = self.get_joint_states(self.dashboard)
        
        self.gripper_pick = rospy.Service('gripper_pick', SetBool, self.gripper_pick_request)

        # 启动服务器
        self.server.start()
        self.dashboard.EnableRobot() 
        # self.dashboard.SetTerminal485(115200)
        self.dashboard.ModbusCreate("127.0.0.1",60000,1,1)
        self.dashboard.SetHoldRegs(1, 256, 1, "{165}", "U16")
        self.dashboard.SetHoldRegs(1, 259, 1, "{1000}", "U16")

    def execute_callback(self, goal):
        rospy.loginfo("Received trajectory goal from MoveIt")

        # 创建 Feedback 和 Result 消息
        feedback = FollowJointTrajectoryFeedback()
        feedback.joint_names = self.joint_names

        # 遍历轨迹中的每一个轨迹点
        for i in range(1, len(goal.trajectory.points)):
            point = goal.trajectory.points[i]
            point_prev = goal.trajectory.points[i-1]
            # rospy.loginfo("joint_position:" + str(point.positions))
            time = point.time_from_start.secs + point.time_from_start.nsecs / 1e9
            time_prev = point_prev.time_from_start.secs + point_prev.time_from_start.nsecs / 1e9
            time_diff = time - time_prev
            # rospy.loginfo("time_from_start:" + str(time_diff))

            # 调用 Dobot API 控制机械臂运动
            self.move.ServoJ(point.positions[0]/pi*180,
                             point.positions[1]/pi*180,
                             point.positions[2]/pi*180,
                             point.positions[3]/pi*180,
                             point.positions[4]/pi*180,
                             point.positions[5]/pi*180,
                             time_diff,
                             50,
                             500)
            if (i == len(goal.trajectory.points)-1):
                self.move.Sync()
            else:
                rospy.sleep(time_diff)
            # 更新关节位置为当前值
            self.joint_positions = self.get_joint_states(self.dashboard)
            # self.joint_positions = list(point.positions)
            
            # 填充 Feedback 消息
            feedback.actual.positions = self.joint_positions
            feedback.desired.positions = list(point.positions)
            feedback.error.positions = list(map(lambda a, b: a - b, feedback.desired.positions, feedback.actual.positions))

            # 发布当前的关节状态到 /joint_states 话题
            self.publish_joint_state()

            # 发送反馈给 MoveIt
            self.server.publish_feedback(feedback)

        # 轨迹执行完成，返回成功结果
        result = FollowJointTrajectoryResult()
        self.server.set_succeeded(result)

    def gripper_pick_request(self, req):
        if req.data:
            self.dashboard.SetHoldRegs(1, 259, 1, "{1000}", "U16")
            return SetBoolResponse(True, "Open Gripper")
        else:
            self.dashboard.SetHoldRegs(1, 259, 1, "{140}", "U16")
            return SetBoolResponse(False, "Close Gripper")
    
    def publish_joint_state(self):
        """发布机械臂当前的关节状态到 /Joint_State 话题"""
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions

        # 发布关节状态
        self.joint_state_pub.publish(joint_state)

    def connect_robot(self):
        try:
            ip = "192.168.31.100"
            dashboard_p = 29999
            move_p = 30003
            feed_p = 30004
            print("正在建立连接...")
            dashboard = DobotApiDashboard(ip, dashboard_p)
            move = DobotApiMove(ip, move_p)
            feed = DobotApi(ip, feed_p)
            print(">.<连接成功>!<")
            return dashboard, move, feed
        except Exception as e:
            print(":(连接失败:(")
            raise e
        
    def get_joint_states(self, dashboard):
        joint_states = dashboard.GetAngle()
        # 使用正则表达式提取角度
        pattern = r"\{([^}]+)\}"
        match = re.search(pattern, joint_states)
    
        if match:
            # 提取到的角度数据
            angles = match.group(1).split(',')
            # 将角度转换为浮点数
            angles = [float(angle) for angle in angles]
            angles = [angle/180*pi for angle in angles]
            return angles
        else:
            return None
        
if __name__ == '__main__':
    rospy.init_node('moveit_bringup')
    # 创建 Action 服务端
    server = ArmControllerActionServer()
    rate = rospy.Rate(33)  # 33Hz 发送关节状态反馈
    while not rospy.is_shutdown():
        server.publish_joint_state()
        rate.sleep()

    rospy.loginfo("Arm controller action server is ready to receive goals.")
    rospy.spin()
