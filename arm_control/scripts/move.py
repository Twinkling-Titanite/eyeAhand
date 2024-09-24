import geometry_msgs.msg
import numpy as np
import rospy

def rpy2quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    
    q = geometry_msgs.msg.Quaternion(x, y, z, w)
    return q

def pose_move_rpy(group, pose_goal):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = pose_goal.position[0]
    pose.position.y = pose_goal.position[1]
    pose.position.z = pose_goal.position[2]
    pose.orientation = rpy2quaternion(pose_goal.roll, pose_goal.pitch, pose_goal.yaw)
    
    group.set_pose_target(pose)
    plan = group.plan()
    group.execute(plan[1])
    group.clear_pose_targets()
    
def set_pose_tolerance(group, tolerance = 0.001):
    group.set_goal_position_tolerance(tolerance)
    group.set_goal_orientation_tolerance(tolerance)

def pose_move(group, pose_goal, str="move"):
    group.set_pose_target(pose_goal)
    plan = group.plan()
    if plan:
        rospy.loginfo(str + " planning successful!")
    else:
        rospy.logerr(str + " planning failed!")
        return
    group.execute(plan[1])
    group.clear_pose_targets()

def joint_move(group, joint_goal, str="move"):
    plan = group.go(joint_goal, wait=True)
    if plan:
        rospy.loginfo(str + " planning successful!")
    else:
        rospy.logerr(str + " planning failed!")
    group.stop()
    group.clear_pose_targets()