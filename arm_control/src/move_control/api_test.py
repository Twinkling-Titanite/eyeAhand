
import sys
import os
import re
import time
home_dir = os.path.expanduser('~')
os.chdir(home_dir + '/eyeAhand')
sys.path.insert(0, os.getcwd() + "/src/arm_control/scripts")
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType

def connect_robot():
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
    dashboard.SetHoldRegs(1, 259, 1, '{0}', "U16")
    dashboard.RobotMode()
    
    
    
