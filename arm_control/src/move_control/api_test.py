import sys
import os
import re
import time
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType

def connect_robot():
    try:
        ip = "192.168.5.1"
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

def get_joint_states(dashboard):
    joint_states = dashboard.GetAngle()
    # 使用正则表达式提取角度
    pattern = r"\{([^}]+)\}"
    match = re.search(pattern, joint_states)
    
    if match:
        # 提取到的角度数据
        angles = match.group(1).split(',')
        # 将角度转换为浮点数
        angles = [float(angle) for angle in angles]
        return angles
    else:
        return None

if __name__ == '__main__':

    dashboard, move, feed = connect_robot()
    
    dashboard.DisableRobot()
    dashboard.EnableRobot()
    dashboard.RobotMode()

    move.MovL(400, -100, 500, -89, -1, -88)


    # start = 0
    # end = 0
    # if joint_states[0] == '0':
    #     print("获取机械臂角度")
    #     for i in range(0, len(joint_states)):
    #         if joint_states[i] == '{':
    #             start = i + 1
    #         if joint_states[i] == '}':
    #             end = i - 1
    #             break
                
    # angle = 0
    # while True:
    #     move.ServoJ(0, 0, angle, 0, 0, 0,t=0.1,lookahead_time=50, gain=500)
    #     angle += 0.1
    #     if angle > 30:
    #         break

    