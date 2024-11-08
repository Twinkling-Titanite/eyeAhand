# 手眼协同工具包
## 1. 安装
### 1.1 环境系统

- 操作系统：Ubuntu 20.04
- ROS版本：noetic

### 1.2 环境配置

> 工具包中的third_party中包含与硬件连接的ros驱动以及仿真环境的ros驱动。其中硬件驱动需要安装相应厂家的SDK，不建议使用虚拟机进行硬件连接。如果您使用虚拟机或不进行硬件调试建议删除third_party中的硬件驱动内容，只保留sim_ros文件夹下的仿真驱动。

- 创建工作空间

```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

- 安装硬件SDK

- - intel realsense sdk: [intel realsense](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide?_ga=2.72240400.2044111929.1731042713-1675918350.1729668236)
- - orbbec sdk: [orbbec](https://note.youdao.com/)
- - hikrobot mvs camera sdk: [hikrobot](https://note.youdao.com/)

- 使用rosdep配置环境

> 中国用户如果rosdep超时可以使用rosdepc，使用方法搜索rosdepc

```
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
---
## 2. 功能介绍与使用

> 参数更改在/arm_control/scripts/uilts.py中，主要包括：
棋盘格参数、moveit规划组名称、相机相关话题

### 2.1 获取相机内参

- 通过话题获取

```
rosrun arm_control camera_info.py
```

- 手动标定获取（需要准备棋盘格，并拍摄不同角度的棋盘格照片）

```
rosrun arm_control get_camera_info.py
```

### 2.2 标定相机外参

- 手动标定（需要准备棋盘格，有in和to两种模式）

```
rosrun arm_control get_cam2_data.py
```

### 2.3 保存图像和点云数据

- 数据保存在/arm_control/tools/save_cam_data/文件夹下

```
rosrun arm_control save_cam_data.py
```

### 2.4 机械臂重定位误差检测

- 使用moveit手动调整机械臂运动后回到原位置，拍摄图像并对比棋盘格位置差异（精准度与相机像素大小相关）

```
rosrun arm_control arm_error_meter.py
```

### 2.5 生成仿真棋盘格

- 生成棋盘格图片，请在准备好的.dae文件中更改模型大小以便适应新的图片

```
rosrun arm_control get_sim_chessboard.py
```

- 在仿真模型的xacro文件中添加，以便使棋盘格生效

```
<xacro:include filename="$(find arm_control)/tools/get_sim_chessboard/chessboard.xacro"/>
```

### 2.6 图像检测
- 检测红色物体，发布轮廓像素点话题

```
rosrun arm_control red_object_pose_find.py
```

- 检测燕毛，发布轮廓像素点话题

```
rosrun arm_control detect.py
```

### 2.7 坐标系转换

- in模式下转换（深度图像默认已经与彩色图像对齐）

```
rosrun arm_control img2world_in.py
```

- to模式下转换（手动对齐深度和彩色图像，需要depth_to_color转移矩阵）

```
rosrun arm_control img2world_to.py
```

### 2.8 dobot机械臂moveit控制接口

- 使用api连接dobot硬件机械臂与moveit控制器

```
rosrun arm_control moveit_bringup.py
```

