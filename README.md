# Mini工业互联网产线


## 模块介绍
目前v1.0版包含以下包及模块

**ROS相关**

- 公共Msg/Srv/Action（[itheima_msgs](src/itheima_msgs)）
- 产品装配流水线（[assembly_line](src/assembly_line)）
  - 红外信号信息获取
  - 传送带启停控制
- 相机驱动模块
  - USB相机驱动
  - Astra奥比中光相机驱动
  - Kinect2相机驱动
- 机械臂控制（[aubo_ctrl](src/aubo_ctrl)）
  - Aubo机械臂控制
  - DH大环夹爪控制
  - 因时夹爪控制（计划）
  - 吸盘控制（计划）
- 视觉同步（[ui_sync](src/ui_sync)）
- 统一启动包（[lab_bringup](src/lab_bringup)）
- 无人小车模块
- 激光打标机模块

**Unity相关**

实现装配线数字孪生，用于实时同步展示流水线各个环节状态

unity_mini_industry

**前端显示**

实现直观的前端数据可视化

ui_mini_industry


## 环境准备Requirements

- 安装USB相机依赖
```bash
sudo apt install ros-$ROS_DISTRO-libuvc-camera
```

- 根据ROS项目需要自动安装ros依赖包
```bash
cd ros_mini_industry/src
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

## 如何使用Usage

先在项目根目录执行一次编译及环境初始化
```bash
cd ros_mini_industry/src
catkin_make
source devel/setup.bash
```

### 完整流程
```bash
roslaunch lab_bringup startup.launch
```

### 独立单元流程
- **流水线**
```bash
roslaunch lab_bringup test_env_assembly.launch
```

- **机械臂上下料**

在独立的terminal启动一个服务端长期运行
```bash
roslaunch lab_bringup test_env_aubo_ctrl.launch
```

在新的terminal可多次执行以下上下料测试
```bash
# 上料
roslaunch lab_bringup test_run_aubo_feeding.launch
# 下料
roslaunch lab_bringup test_run_aubo_blanding.launch
```

- **激光打标**

在独立的terminal启动一个服务端长期运行
```bash
roslaunch lab_bringup test_env_laser_mark.launch
```

在新的terminal可多次执行以下打标测试
```bash
roslaunch lab_bringup test_run_laser_mark.launch
```

- **夹爪开合**

开启服务

```bash
roslaunch dh_hand_driver dh_hand_controller.launch
```

测试开合

```bash
rosrun dh_hand_driver hand_controller_client 1 50 80
```

