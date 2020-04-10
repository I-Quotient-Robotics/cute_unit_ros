# cute_unit_ros
ROS package for pick-place training unit, based on cute arm

## ROS Package

- [cute_unit_app](cute_unit_app): Application, including pick&place program
- [cute_unit_base](cute_unit_base): Hardware interface
- [cute_unit_description](cute_unit_description): Contains robot description (URDF)
- [cute_unit_moveit_config](cute_unit_moveit_config): Arm MoveIt config
- [cute_unit_viz](cute_unit_viz): Visualization(RVIZ) configuration and launch file

## How To Use

1. Change arm serial port permission

   ```shell
   sudo chmod 777 /dev/ttyUSB0
   ```

2. Bring up arm and Realsense driver

   ```shell
   roslaunch cute_unit_base base.launch
   ```

3. Set arm to home pose

   ```shell
   rosservice call /cute_go_home "data: true" 
   ```

4. Bring up arm MoveIt node

   ```shell
   roslaunch cute_unit_moveit_config moveit_planning_execution.launch
   ```

5. Bring up pick&place node

   ```shell
   roslaunch cute_unit_app cute_unit_app.launch
   ```

------
© 2019 Beijing I-Quotient Robot Technology Co., Ltd. All Rights Reserved