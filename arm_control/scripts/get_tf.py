import numpy as np
import cv2
import yaml

def get_corners_tf(image, chessboard_size):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        # 细化角点坐标
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        corners = np.array(corners, dtype=np.float32)
        # print(len(corners))
    else:
        print("chessboard not found")
    return corners

def get_world_coordinates(pixel_x, pixel_y, depth, camera_matrix, T_cam_to_world):
    # 从像素坐标和深度反投影到相机坐标系
    pixel = np.array([pixel_x, pixel_y, 1.0])
    camera_coords = np.linalg.inv(camera_matrix).dot(pixel) * depth

    # 齐次坐标表示
    camera_coords_homogeneous = np.append(camera_coords, 1.0)

    # 转换到世界坐标系
    world_coords_homogeneous = T_cam_to_world.dot(camera_coords_homogeneous)
    world_coords = world_coords_homogeneous[:3]

    return world_coords

def get_corners2world(rgb_image, depth_image, camera_matrix, T_cam_to_world, chessboard_size):
    corners_tf = get_corners_tf(rgb_image, chessboard_size)
    world_coords = np.zeros((len(corners_tf), 3))
    for i in range(len(corners_tf)):
        # 将浮点数索引转换为整数
        x = int(corners_tf[i][0][0])
        y = int(corners_tf[i][0][1])
        
        # 获取深度值
        depth = depth_image[y, x] * 0.001  # 单位为米
        # rospy.loginfo("depth: {}".format(depth))
        
        world_coords[i] = get_world_coordinates(x, y, depth, camera_matrix, T_cam_to_world)
        
    return world_coords

def get_board2cam_tf(image, img_num, result_path, object_points, chessboard_size, camera_matrix, dist_coeffs = np.zeros(5)):
    flag = False
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # 查找棋盘格的角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        # 细化角点坐标
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        # 假设没有畸变
        # 计算标定板相对于相机的位姿
        _, rvec, tvec = cv2.solvePnP(object_points, corners2, camera_matrix, dist_coeffs)

        # 将旋转向量转换为旋转矩阵
        rmat, _ = cv2.Rodrigues(rvec)
        
        # 组合旋转矩阵和平移向量为位姿矩阵
        pose_matrices = []
        pose_matrix = np.hstack((rmat, tvec))
        pose_matrices.append(pose_matrix)
        
        # 在图像上绘制坐标轴用于可视化
        axis = np.float32([[20, 0, 0], [0, 20, 0], [0, 0, -20]]).reshape(-1, 3)
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
 
        img = cv2.drawChessboardCorners(image, chessboard_size, corners, ret)
        corner = tuple(corners[0].ravel().astype(int))
 
        # 确保坐标点是以正确的格式传递给 cv2.line 函数
        img = cv2.line(img, corner, tuple(imgpts[0].ravel().astype(int)), (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel().astype(int)), (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel().astype(int)), (0, 0, 255), 5)
 
        cv2.imwrite(result_path + str(img_num) + '.jpg', img)

        flag = True
        tvec = tvec.reshape(3, 1)
        return rmat, tvec*0.001, flag
    
    else:
        return None, None, flag

def get_board2cam_tf_charuco(image, img_num, result_path, camera_matrix, aruco_dict, charuco_board, dist_coeffs = np.zeros(5)):
    flag = False
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)
    
    # 初始化旋转向量和平移向量
    rvec = np.zeros((3, 1))
    tvec = np.zeros((3, 1))

    # 如果检测到角点
    if len(corners) > 0:
        # 检测Charuco板上的角点
        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board)
    
        # 如果检测到足够的Charuco角点
        if charuco_corners is not None and len(charuco_corners) > 0:

            # 假设没有畸变
            # 估计姿态（外参）
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, charuco_board, camera_matrix, dist_coeffs, rvec, tvec)

            if retval:
                # 绘制坐标轴
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                flag = True
                
                rmat, _ = cv2.Rodrigues(rvec)
                
                cv2.imwrite(result_path + str(img_num) + '.jpg', image)
                
            else:
                print("姿态估计失败")
        else:
            print("没有检测到足够的Charuco角点")
    else:
        print("没有检测到ArUco标志")
    return rmat, tvec, flag
        
def get_end2world_tf(path, img_num):
    with open(path, 'r') as f:
        poses_end = yaml.safe_load(f)
    
    position_temp = poses_end['poses_end'][img_num]['pose'+str(img_num)]['position']
    t = np.array([[position_temp['x']], [position_temp['y']], [position_temp['z']]])
    orientation_temp = poses_end['poses_end'][img_num]['pose'+str(img_num)]['orientation']
    orientation = np.array([orientation_temp['w'], orientation_temp['x'], orientation_temp['y'], orientation_temp['z']])
    R = quaternion_to_rotation_matrix(orientation)
    
    return t, R
        
def quaternion_to_rotation_matrix(q):
    """
    将四元数转换为旋转矩阵。
    :param q: 四元数 [w, x, y, z]
    :return: 3x3 旋转矩阵
    """
    w, x, y, z = q
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])
    return R    

def get_cam2_tf(image_path, yaml_path, img_num, result_path, object_points, chessboard_size, camera_matrix, tf_path, hand_eye = 'to', dist_coeffs = np.zeros(5), method = cv2.CALIB_HAND_EYE_TSAI):
        TfMatrix = []
        # for i in random.sample(range(1, img_num), img_num - 1):
        for i in range(0, img_num):
            # if i == 6:
            #     continue
            image = cv2.imread(image_path + str(i) + '.jpg')
            rmat_board2cam, tvec_board2cam, flag = get_board2cam_tf(image, i, result_path, object_points, chessboard_size, camera_matrix, dist_coeffs)
            # rmat_board2cam, tvec_board2cam, flag = get_board2cam_tf_charuco(image, i)
            if flag:
                print('img_' + str(i) + '.jpg')
                tvec_end2world, rmat_end2world = get_end2world_tf(yaml_path, i)

                rmat_world2end = np.linalg.inv(rmat_end2world)
                tvec_world2end = -np.dot(rmat_world2end, tvec_end2world)
                if hand_eye == 'to':
                    TfMatrix .append(TransformsMatrix(tvec_board2cam, rmat_board2cam, tvec_world2end, rmat_world2end))
                else: 
                    if hand_eye == 'in':
                        TfMatrix .append(TransformsMatrix(tvec_board2cam, rmat_board2cam, tvec_end2world, rmat_end2world))
            else:
                print('img_' + str(i) + '.jpg chessboard corners not found.')
    
        R_cam2world = np.eye(3)
        t_cam2world = np.zeros(3)
        # 执行手眼标定
        cv2.calibrateHandEye(
                            [m.rmat_world2end for m in TfMatrix],
                            [m.tvec_world2end for m in TfMatrix],
                            [m.rmat_board2cam for m in TfMatrix],
                            [m.tvec_board2cam for m in TfMatrix],
                            R_cam2world, t_cam2world,
                            method)

        T_cam2world = np.eye(4)
        T_cam2world[:3, :3] = R_cam2world
        T_cam2world[:3, 3] = t_cam2world.reshape(3)
        print('T_cam2:\n', T_cam2world)
        
        # 保存相机到世界坐标系的变换矩阵
        if hand_eye == 'to':
            data = {'cam2world_tf_matrix': T_cam2world.tolist()}
        else:
            data = {'cam2end_tf_matrix': T_cam2world.tolist()}
            
        with open(tf_path, 'w') as f:
            yaml.dump(data, f)

class TransformsMatrix:
    def __init__(self, tvec_board2cam, rmat_board2cam, tvec_world2end, rmat_world2end):
        self.tvec_board2cam = tvec_board2cam
        self.rmat_board2cam = rmat_board2cam
        self.tvec_world2end = tvec_world2end
        self.rmat_world2end = rmat_world2end
        
def get_depth(p_x, p_y, depth_image, camera_matrix_color, camera_matrix_depth, T_cam_color_to_world, T_cam_depth_to_world):
    # 读取深度图像
    depth_image = depth_image * 0.001  # 将深度转换为米
    # 获取图像的尺寸
    height, width = depth_image.shape

    # 生成所有像素点的坐标
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    u = u.flatten()
    v = v.flatten()

    # 将像素坐标转换为相机坐标 (加入深度信息)
    ones = np.ones_like(u)
    pixels_depth = np.stack((u * depth_image.flatten(), v * depth_image.flatten(), depth_image.flatten()), axis=0)  # [3, height*width]

    # 计算相机坐标（深度相机）
    inv_camera_matrix_depth = np.linalg.inv(camera_matrix_depth)
    cam_coords_depth = inv_camera_matrix_depth @ pixels_depth  # [3, height*width]

    # 计算世界坐标
    cam_coords_depth = np.vstack((cam_coords_depth, np.ones((1, cam_coords_depth.shape[1]))))  # [4, height*width]
    world_coords = T_cam_depth_to_world @ cam_coords_depth  # [4, height*width]

    # 转换为彩色相机坐标
    cam_coords_color = np.linalg.inv(T_cam_color_to_world) @ world_coords  # [4, height*width]
    cam_coords_color = cam_coords_color[:3, :] / cam_coords_color[2, :]  # [3, height*width]

    # 投影到彩色相机平面
    pixel_coords_color = camera_matrix_color @ cam_coords_color  # [3, height*width]
    pixel_coords_color = pixel_coords_color[:2, :] / pixel_coords_color[2, :]  # [2, height*width]

    # 计算与目标像素 (p_x, p_y) 的距离
    distances = np.sqrt((pixel_coords_color[0, :] - p_x)**2 + (pixel_coords_color[1, :] - p_y)**2)

    # 找到距离小于5的像素
    valid_depths = depth_image.flatten()[distances < 2]
    pixel_x = u[distances < 2]
    pixel_x = np.mean(pixel_x)
    pixel_y = v[distances < 2]
    pixel_y = np.mean(pixel_y)

    if len(valid_depths) > 0:
        return np.mean(valid_depths), pixel_x, pixel_y
    else:
        return 0, 0, 0  # 或者返回其他合适的值
