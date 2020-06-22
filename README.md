# Mini工业互联网产线

目前v1.0版包含以下包及模块

**ROS相关**

- 公共Msg/Srv/Action[itheima_msgs](src/itheima_msgs)
- 产品装配流水线[assembly_line](src/assembly_line)
  - 红外信号信息获取
  - 传送带启停控制
- 机械臂控制[aubo_ctrl](src/aubo_ctrl)
  - Aubo机械臂控制
  - DH大环夹爪控制
- 视觉同步[ui_sync](src/ui_sync)
- 统一启动包[lab_bringup](src/lab_bringup)

- 无人小车模块
- 激光打标机模块

**Unity相关**

实现装配线数字孪生，用于实时同步展示流水线各个环节状态

unity_mini_industry

**前端显示**

ui_mini_industry

