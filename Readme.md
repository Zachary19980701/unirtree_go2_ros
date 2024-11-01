# unitree_go2_ros
宇树 Go2 ROS1驱动包
## 安装
安装环境为Ubuntu 20.04 + ROS Noetic，unitree_sdk编译安装。
clone仓库到catkin_ws/src目录下，编译，安装。

## 运行
启动roscore
```bash
roscore
``` 
启动go2本体驱动程序
```bash
rosrun go2_ros go2_ros_node
```
启动go2摄像头程序
```bash
rosrun go2_ros go2_images_node
```

## 接口说明
### 本体控制话题
速度控制接口：/cmd_vel
机器人运动步态控制接口：/sport_mode, 参数类型为std_msgs/Int32，范围[0, 1, 2, 3, 4]，对应不同步态。
参数说明：
- stop_move: 0，停止运动
- stand: 1, 站立
- walk: 2, 走路模式
- running: 3, 跑步模式
- climb: 4,爬楼梯步态