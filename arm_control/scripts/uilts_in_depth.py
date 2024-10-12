# 相机内参路径
camera_in_depth_info_path = '/src/arm_control/config/camera_in/depth/camera_depth_info.yaml'
camera_in_ir_info_path = '/src/arm_control/config/camera_in/depth/camera_ir_info.yaml'

# 录制图像路径
camera_in_ir_img_path = '/src/arm_control/img/camera_in/depth/'

# 结果保存路径
camera_in_ir_result_path = '/src/arm_control/result/camera_in/depth/'

# 相机位姿变换矩阵路径
cam2end_ir_tf_path = '/src/arm_control/config/camera_in/depth/cam2world_tf_matrix.yaml'

# 相机内参话题
camera_in_depth_info_topic = '/camera_in/depth/camera_info'
camera_in_ir_info_topic = '/camera_in/ir/camera_info'

# 相机矫正前图片话题
camera_in_depth_img_raw_topic = '/camera_in/depth/image_raw'
camera_in_ir_img_raw_topic = '/camera_in/ir/image_raw'

# 相机矫正后图片话题
camera_in_depth_img_corrected_topic = '/camera_in/depth/image_corrected'
camera_in_ir_img_corrected_topic = '/camera_in/ir/image_corrected'
