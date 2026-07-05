!/bin/bash

echo "===== 启动 PX4 SITL ====="

gnome-terminal -- bash -c "
cd ~/PX4-Autopilot
make px4_sitl gz_x500
exec bash
"

sleep 8

echo "===== 启动 MicroXRCEAgent ====="

gnome-terminal -- bash -c "
MicroXRCEAgent udp4 -p 8888
exec bash
"

sleep 3

echo "===== 启动 ROS Bridge + TF ====="

gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source /home/wei/px4_gazebo/ws_ros_gz/install/setup.bash
source /home/wei/px4_gazebo/px4_gazebo_ws/install/setup.bash
ros2 launch offboard_control bridge_tf.launch.py
exec bash
"

sleep 3

echo "===== 启动 RViz2 ====="

gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source ~/ws_ros_gz/install/setup.bash


ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
exec bash
"

