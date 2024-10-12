# 相机内参路径
camera_to_depth_info_path = '/src/arm_control/config/camera_to/depth/camera_depth_info.yaml'
camera_to_ir_info_path = '/src/arm_control/config/camera_to/depth/camera_ir_info.yaml'

# 录制图像路径
camera_to_ir_img_path = '/src/arm_control/img/camera_to/depth/'

# 结果保存路径
camera_to_ir_result_path = '/src/arm_control/result/camera_to/depth/'

# 相机位姿变换矩阵路径
cam2world_ir_tf_path = '/src/arm_control/config/camera_to/depth/cam2world_tf_matrix.yaml'

# 相机内参话题
camera_to_depth_info_topic = '/camera_to/depth/camera_info'
camera_to_ir_info_topic = '/camera_to/ir/camera_info'

# 相机矫正前图片话题
camera_to_depth_img_raw_topic = '/camera_to/depth/image_raw'
camera_to_ir_img_raw_topic = '/camera_to/ir/image_raw'

# 相机矫正后图片话题
camera_to_depth_img_corrected_topic = '/camera_to/depth/image_corrected'
camera_to_ir_img_corrected_topic = '/camera_to/ir/image_corrected'