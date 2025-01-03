# 棋盘格尺寸和方块尺寸
chessboard_size = (11, 8)
square_size = 15  # 单位mm

#是否矫正深度相机
correct_depth_camera = False

# 深度相机到彩色相机转移矩阵话题
camera_to_d2c_topic = '/camera_to/depth_to_color'
camera_in_d2c_topic = '/camera_in/depth_to_color'

# 深度相机到彩色相机转移矩阵文件路径
camera_to_depth_to_color_path = '/config/camera_to/camera_depth_to_color.yaml'
camera_in_depth_to_color_path = '/config/camera_in/camera_depth_to_color.yaml'

# 录制机械臂末端位姿路径
camera_in_poses_end_path = '/tools/get_cam2_tf/camera_in/poses_end.yaml'
camera_to_poses_end_path = '/tools/get_cam2_tf/camera_to/poses_end.yaml'

# 物体识别像素坐标话题
object_poistion_1_topic = '/arm_control/red_position_1'
object_poistion_2_topic = '/arm_control/red_position_2'

# 世界坐标发布话题
world_coord_topic = '/arm_control/world_coordinate'

# moveit设置参数
group_arm_name = 'panda_arm'
hand = False
group_hand_name = 'panda_hand'
planner_id = 'RRTConnect'
max_velocity_scaling_factor = 0.03
max_acceleration_scaling_factor = 0.01
tolerance = 1e-5

# ———————————————————————in模式彩色相机参数———————————————————————————

# 相机内参路径
camera_in_color_info_path = '/config/camera_in/color/camera_color_info.yaml'
# 相机位姿变换矩阵路径
cam2end_color_tf_path = '/config/camera_in/color/cam2end_tf_matrix.yaml'
# 相机内参话题
camera_in_color_info_topic = '/camera_in/color/camera_info'
# 相机矫正前图片话题
camera_in_color_img_raw_topic = '/camera_in/color/image_raw'
# 相机矫正后图片话题
camera_in_color_img_corrected_topic = '/camera_in/color/image_raw'

# ———————————————————————to模式彩色相机参数————————————————————————————

# 相机内参路径
camera_to_color_info_path = '/config/camera_to/color/camera_color_info.yaml'
# 相机位姿变换矩阵路径
cam2world_color_tf_path = '/config/camera_to/color/cam2world_tf_matrix.yaml'
# 相机内参话题
camera_to_color_info_topic = '/camera_to/color/camera_info'
# 相机矫正前图片话题
camera_to_color_img_raw_topic = '/camera_to/color/image_raw'
# 相机矫正后图片话题
camera_to_color_img_corrected_topic = '/camera_to/color/image_raw'

# ———————————————————————in模式深度相机参数——————————————————————————————

# 相机内参路径
camera_in_depth_info_path = '/config/camera_in/depth/camera_depth_info.yaml'
# 相机内参话题
camera_in_depth_info_topic = '/camera_in/depth/camera_info'
# 相机矫正前图片话题
camera_in_depth_img_raw_topic = '/camera_in/aligned_depth_to_color/image_raw'
# 相机矫正后图片话题
camera_in_depth_img_corrected_topic = '/camera_in/aligned_depth_to_color/image_raw'

# ———————————————————————to模式深度相机参数———————————————————————————————

# 相机内参路径
camera_to_depth_info_path = '/config/camera_to/depth/camera_depth_info.yaml'
# 相机内参话题
camera_to_depth_info_topic = '/camera_to/depth/camera_info'
# 相机矫正前图片话题
camera_to_depth_img_raw_topic = '/camera_to/depth/image_raw'
# 相机矫正后图片话题
camera_to_depth_img_corrected_topic = '/camera_to/depth/image_corrected'