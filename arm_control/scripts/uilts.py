# 棋盘格尺寸和方块尺寸
chessboard_size = (11, 8)
square_size = 15  # 单位mm

# 矫正参数手眼系统模式
handeye = 'to'  # 'in' or 'to'

#是否矫正深度相机
correct_depth_camera = False

# 深度相机到彩色相机转移矩阵话题
camera_depth_to_color_topic = '/camera_to/depth_to_color'

# 深度相机到彩色相机转移矩阵文件路径
camera_depth_to_color_path = '/src/arm_control/config/camera_depth_to_color.yaml'

# 录制机械臂末端位姿路径
camera_in_poses_end_path = '/src/arm_control/config/camera_in/poses_end.yaml'
camera_to_poses_end_path = '/src/arm_control/config/camera_to/poses_end.yaml'

# 最大移动范围路径
camera_in_max_positions_path = '/src/arm_control/config/camera_in/max_position.yaml'
camera_to_max_positions_path = '/src/arm_control/config/camera_to/max_position.yaml'

# 红色物体识别像素坐标与深度信息话题
red_object_position_topic = '/arm_control/position_object_red'
red_poistion_1_topic = '/arm_control/red_position_1'
red_poistion_2_topic = '/arm_control/red_position_2'

# 世界坐标发布话题
world_coord_topic = '/img2world/world_coordinates'

# moveit设置参数
group_arm_name = 'dobot_arm'
hand = False
group_hand_name = 'dobot_hand'
planner_id = 'RRTConnect'
max_velocity_scaling_factor = 0.05
max_acceleration_scaling_factor = 0.01
tolerance = 1e-5
