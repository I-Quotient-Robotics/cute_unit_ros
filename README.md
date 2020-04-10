# cute_unit_ros
ROS package for pick-place training unit, based on cute arm

## ROS包

- [cute_unit_app](cute_unit_app): 抓取Demo
- [cute_unit_base](cute_unit_base): 机械臂训练单元的驱动和Realsense驱动
- [cute_unit_description](cute_unit_description): 机械臂训练单元的ROS模型
- [cute_unit_moveit_config](cute_unit_moveit_config): MoveIt的配置
- [cute_unit_viz](cute_unit_viz): Rviz可视化配置文件

## 程序安装

1. 新建Catkin工程文件夹，并下载相关的ROS包

   ```shell
   mkdir -p ~/arm_ws/src
   cd ~/arm_ws/src
   git clone https://github.com/IntelRealSense/realsense-ros.git
   git clone https://github.com/hans-robot/cute_robot.git
   git clone https://github.com/I-Quotient-Robotics/cute_unit_ros.git
   ```

2. 安装相关依赖程序

   ```shell
   cd ~/arm_ws
   rosdep install --from-paths src --ignore-src -r -y
   pip2 install math3d
   ```

3. 增加串口权限配置文件

   ```shell
   cd ~/arm_ws/src/cute_unit_ros/cute_unit_base/rules
   sudo cp 51-cute-robot.rules /etc/udev/rules.d
   sudo service udev reload
   sudo service udev restart
   sudo udevadm trigger
   ```

4. 编译，并索引ROS包

   ```shell
   cd ~/arm_ws
   catkin_make
   source devel/setup.bash
   ```

## 如何使用

1. 修改串口权限（不一定是ttyUSB0，根据实际情况更改设备符）

   ```shell
   sudo chmod 777 /dev/ttyUSB0
   ```

2. 启动机械臂训练单元的驱动

   ```shell
   roslaunch cute_unit_base base.launch
   ```

3. 将机械臂设置为初始位，机械臂竖起

   ```shell
   rosservice call /cute_go_home "data: true" 
   ```

4. 启动MoveIt节点，用于机械臂运动规划和控制

   ```shell
   roslaunch cute_unit_moveit_config moveit_planning_execution.launch
   ```

5. 将在实验平台上放置物块

6. 启动机械臂抓取节点程序

   ```shell
   roslaunch cute_unit_app cute_unit_app.launch
   ```

7. 完成抓取后，可以在实验平台上重新放置物体，程序重新执行抓取

## 夹爪使用

1. 按照上文的方式，启动机械臂训练单元的驱动

2. 开启夹爪舵机的力控制

   ```shell
   rosservice call /claw_controller/torque_enable "torque_enable: true"
   ```

3. 使用如下指令进行夹爪的开合控制

   ```shell
    # 张开
    rostopic pub /claw_controller/command std_msgs/Float64 "data: 0.0"
    # 闭合
    rostopic pub /claw_controller/command std_msgs/Float64 "data: -2.0"
   ```

## 机械臂其他使用方式

请参考[cute_robot](https://github.com/I-Quotient-Robotics/cute_robot)的README.md文件

## 注意

1. 在启动`base.launch`后，应当现将机械臂初始化，不然舵机处于释放状态，无法使用
2. 关节舵机在长时间承受大扭力时，会发生关节解锁，机械臂突然掉落，此时机械臂需要重新上电才能恢复使用
3. Cute机械臂无机械锁止装置，掉电后，会自由下垂。所以在关机时，一定要扶好机械臂，避免砸到东西

## 已知问题

1. 因为机械臂关节的刚度不够，所以末端位置不可靠，抓取成功率低
2. 目前只能抓取小的方形物块或类似的物件（方糖）

------
© 2020 Beijing I-Quotient Robot Technology Co., Ltd. All Rights Reserved