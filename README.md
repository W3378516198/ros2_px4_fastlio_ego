[UAV_MID360S_ROS2_PX4_README.md](https://github.com/user-attachments/files/29639594/UAV_MID360S_ROS2_PX4_README.md)
# Mid360s 无人机避障飞行项目说明

> 当前项目使用 **Livox Mid360s 雷达**、**ROS 2 Humble**、**PX4 1.16**、**Gazebo Harmonic**。  
> 仿真侧用于验证 PX4、Gazebo、ROS2 通信、TF、RViz 和控制链路；真机侧使用 Mid360s + FAST-LIO2 定位，再接 Ego Planner 规划和 PX4 外部视觉/Offboard 控制。  
> 不明白的地方加 QQ：**3378516198**。

---

## 1. 项目总体架构

### 1.1 仿真架构

```text
Gazebo Harmonic 仿真环境
        │
        │  雷达 / IMU / 飞机模型数据
        ▼
ros_gz_bridge / bridge_tf.launch.py
        │
        ├── 点云 / IMU / TF / clock
        │
        ▼
ROS 2 Humble
        │
        ├── RViz2 显示
        ├── Ego Planner 规划，按实际 launch 启动
        └── PX4 Offboard 控制节点 / 轨迹桥接节点
        │
        ▼
PX4 1.16 SITL
```

仿真里主要检查：

1. PX4 SITL 能否正常启动。
2. MicroXRCEAgent 能否连上 PX4。
3. Gazebo 里的传感器数据能否进入 ROS2。
4. RViz2 能否看到点云、TF、无人机位姿。
5. 后续 Ego Planner 规划出的轨迹能否通过桥接节点转换成 PX4 可执行的控制指令。

说明：仿真中的雷达和 IMU 是 Gazebo 模型模拟出来的，不一定和真实 Mid360s 完全一致。因此如果仿真中 FAST-LIO2 漂移严重，可以先用 PX4 仿真里自带的位姿作为规划输入，把重点放在控制链路和话题链路验证上。真机上再使用真实 Mid360s + FAST-LIO2。

---

### 1.2 真机架构

```text
Livox Mid360s
   │
   ├── /livox/lidar     点云
   └── /livox/imu       IMU
        │
        ▼
FAST-LIO2
        │
        ├── /Odometry              定位结果
        ├── /cloud_registered      配准后的点云
        └── /Laser_map             局部/全局地图显示
        │
        ├──────────────► Ego Planner
        │                    │
        │                    ├── /drone_0_planning/bspline
        │                    └── /drone_0_planning/pos_cmd
        │
        ▼
fastlio_px4_bridge
        │
        ▼
PX4 1.16 外部视觉定位 / Offboard 控制
        │
        ▼
飞控执行
```

真机里主要检查：

1. Mid360s 能否被香橙派识别并稳定发布点云和 IMU。
2. FAST-LIO2 能否输出稳定 `/Odometry`。
3. Ego Planner 能否拿到定位和点云，并生成规划轨迹。
4. 桥接节点能否把 FAST-LIO2 的定位结果送给 PX4。
5. PX4 是否正确使用外部视觉定位。
6. Offboard 控制前必须先做安全检查，螺旋桨测试必须卸桨。

---

## 2. 软件和硬件环境

### 2.1 软件环境

| 项目 | 版本 / 说明 |
|---|---|
| 系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 仿真器 | Gazebo Harmonic |
| 飞控固件 | PX4 1.16 |
| 通信代理 | MicroXRCEAgent |
| 雷达驱动 | livox_ros_driver2 |
| 定位 | FAST-LIO2 ROS2 |
| 规划 | Ego Planner ROS2 |
| 可视化 | RViz2 / QGroundControl |

### 2.2 硬件环境

| 硬件 | 说明 |
|---|---|
| 机载电脑 | OrangePi 5 Max |
| 飞控 | Pixhawk 6C mini |
| 雷达 | Livox Mid360s |
| 定位输入 | Mid360s 点云 + Mid360s 内置 IMU |
| 仿真模型 | Gazebo Harmonic 中的 x500 / Mid360s 模型 |

---

## 3. 仿真启动流程

### 3.1 一键启动脚本

项目中的仿真启动脚本逻辑如下：

1. 启动 PX4 SITL。
2. 启动 MicroXRCEAgent，使用 UDP 端口 `8888`。
3. 启动 ROS Bridge 和 TF。
4. 启动 RViz2，并启用仿真时间。

建议脚本名：

```bash
start_uav.sh
```

给脚本增加执行权限：

```bash
chmod +x start_uav.sh
```

启动：

```bash
./start_uav.sh
```

注意：脚本第一行应该是：

```bash
#!/bin/bash
```

如果写成：

```bash
!/bin/bash
```

可能会导致脚本不能被系统正确识别。

---

### 3.2 手动分终端启动

如果一键脚本启动失败，可以按下面方式手动启动。

#### 终端 1：启动 PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

这里的 `gz_x500` 表示使用 Gazebo Harmonic 的 x500 无人机模型。

---

#### 终端 2：启动 MicroXRCEAgent

```bash
MicroXRCEAgent udp4 -p 8888
```

它的作用是让 PX4 和 ROS2 可以通过 DDS/XRCE 通信。简单理解：它是 PX4 和 ROS2 中间的通信转接器。

---

#### 终端 3：启动 ROS Bridge 和 TF

```bash
source /opt/ros/humble/setup.bash
source /home/wei/px4_gazebo/ws_ros_gz/install/setup.bash
source /home/wei/px4_gazebo/px4_gazebo_ws/install/setup.bash
ros2 launch offboard_control bridge_tf.launch.py
```

这一部分通常负责：

1. 把 Gazebo 中的传感器数据桥接到 ROS2。
2. 发布雷达、机体、世界坐标之间的 TF。
3. 给 RViz2 和后续算法提供统一的数据入口。

---

#### 终端 4：启动 RViz2

```bash
source /opt/ros/humble/setup.bash
source /home/wei/px4_gazebo/ws_ros_gz/install/setup.bash
source /home/wei/px4_gazebo/px4_gazebo_ws/install/setup.bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

如果脚本里写的是：

```bash
source ~/ws_ros_gz/install/setup.bash
```

但实际工作空间在：

```bash
/home/wei/px4_gazebo/ws_ros_gz
```

需要把路径改成真实路径。

---

### 3.3 仿真启动后检查

查看 ROS2 话题：

```bash
ros2 topic list
```

检查 PX4 话题：

```bash
ros2 topic list | grep fmu
```

检查 TF：

```bash
ros2 topic echo /tf
```

检查点云：

```bash
ros2 topic list | grep point
ros2 topic list | grep cloud
```

检查时间：

```bash
ros2 topic echo /clock --once
```

RViz2 中建议：

1. Fixed Frame 先选 `map`、`odom_world`、`camera_init` 或项目里实际存在的世界坐标系。
2. 如果世界坐标系看不到点云，可以先切到雷达自己的 frame，确认雷达数据是否存在。
3. 如果雷达 frame 能看到，世界 frame 看不到，通常是 TF 没连上。

---

## 4. 真机启动流程

真机推荐在香橙派上分多个终端启动。PC 只负责远程 RViz2 显示。

---

### 4.1 配置 Mid360s 网口 IP

#### 临时配置

每次重启或重新插拔网线后，如果 IP 没有保存，可以执行：

```bash
sudo ip addr replace 192.168.1.50/24 dev enP3p49s0
sudo ip link set enP3p49s0 up
```

其中 `enP3p49s0` 是香橙派连接 Mid360s 的有线网卡名。不同设备可能不一样，可以用下面命令查看：

```bash
ip addr
nmcli dev status
```

---

#### 永久配置

先查看 NetworkManager 是否管理这个网卡：

```bash
nmcli dev status
```

如果能看到 `enP3p49s0`，执行：

```bash
sudo nmcli con add type ethernet ifname enP3p49s0 con-name livox-static \
  ipv4.method manual \
  ipv4.addresses 192.168.1.50/24 \
  ipv4.never-default yes \
  ipv6.method ignore \
  connection.autoconnect yes
```

启动这个连接：

```bash
sudo nmcli con up livox-static
```

---

### 4.2 香橙派终端 1：启动 Mid360s 驱动

```bash
source /opt/ros/humble/setup.bash
source ~/plane_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

正常情况下应该能看到 Livox 驱动初始化成功，并发布雷达点云和 IMU。

常用检查命令：

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

---

### 4.3 香橙派终端 2：启动 FAST-LIO2

```bash
source /opt/ros/humble/setup.bash
source ~/plane_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

这里 `rviz:=false` 是为了减轻香橙派负担。RViz2 建议放到电脑端运行。

FAST-LIO2 正常运行后，常见输出话题：

```bash
/Odometry
/cloud_registered
/cloud_registered_body
/Laser_map
```

检查定位是否有输出：

```bash
ros2 topic hz /Odometry
ros2 topic echo /Odometry --once
```

---

### 4.4 香橙派终端 3：启动 Ego Planner

```bash
source /opt/ros/humble/setup.bash
source ~/plane_ws/install/setup.bash
ros2 launch ego_planner real_mid360s.launch.py
```

Ego Planner 需要拿到：

1. 当前无人机位置。
2. 当前环境点云。
3. 目标点。

常见规划输出话题：

```bash
/drone_0_planning/bspline
/drone_0_planning/pos_cmd
```

说明：

- `/drone_0_planning/bspline` 更像是规划器生成的“整条平滑轨迹信息”。
- `/drone_0_planning/pos_cmd` 更像是某一时刻要执行的“当前位置目标命令”。
- 如果要控制 PX4，通常不能直接把 `bspline` 丢给 PX4，而是需要桥接节点按时间取出轨迹上的点，再转换成 PX4 能执行的 setpoint。

---

### 4.5 香橙派终端 4：启动 MicroXRCEAgent

```bash
MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600
```

说明：

- `/dev/ttyS1` 是当前帮助文件中的飞控串口。
- 如果实际接线换了串口，可能是 `/dev/ttyS7` 或其他设备名。
- 波特率当前使用 `921600`。

检查串口设备：

```bash
ls /dev/ttyS*
ls /dev/ttyUSB*
```

如果没有权限，可以尝试：

```bash
sudo usermod -aG dialout $USER
```

执行后需要重新登录系统。

---

### 4.6 香橙派终端 5：启动 FAST-LIO2 到 PX4 的定位桥接

方案 A：不使用位置差分速度

```bash
source ~/plane_ws/install/setup.bash
ros2 run fastlio_px4_bridge fastlio_to_px4_visual_odometry \
  --ros-args \
  -p input_odom_topic:=/Odometry \
  -p publish_orientation:=false \
  -p publish_velocity_from_delta:=false
```

方案 B：使用位置差分速度

```bash
source ~/plane_ws/install/setup.bash
ros2 run fastlio_px4_bridge fastlio_to_px4_visual_odometry \
  --ros-args \
  -p input_odom_topic:=/Odometry \
  -p publish_orientation:=false \
  -p publish_velocity_from_delta:=true
```

参数解释：

| 参数 | 含义 |
|---|---|
| `input_odom_topic:=/Odometry` | 输入 FAST-LIO2 的定位结果 |
| `publish_orientation:=false` | 暂时不把 FAST-LIO2 姿态发给 PX4，只发位置相关信息 |
| `publish_velocity_from_delta:=false` | 不根据位置变化估计速度 |
| `publish_velocity_from_delta:=true` | 根据前后位置变化估计速度，并一起发给 PX4 |

如果 PX4 只需要外部视觉位置，可以先用方案 A。  
如果 EKF 需要速度辅助，或者位置融合不稳定，再尝试方案 B。

---

## 5. 电脑端远程 RViz2 显示

香橙派负责跑算法，电脑负责显示。

电脑端执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic list
rviz2
```

注意：

1. 香橙派和电脑必须在同一个网络里。
2. 两边 `ROS_DOMAIN_ID` 要一致。
3. 两边都要设置：

```bash
export ROS_LOCALHOST_ONLY=0
```

如果电脑端看不到香橙派的话题，优先检查：

```bash
echo $ROS_DOMAIN_ID
printenv ROS_LOCALHOST_ONLY
ping <香橙派IP>
ros2 topic list
```

---

## 6. PX4 外部视觉参数设置

进入 PX4 shell 或 QGroundControl 的 MAVLink Console 后执行：

```bash
param set EKF2_EV_CTRL 3
param save
reboot
```

作用：让 PX4 的 EKF 使用外部视觉定位信息。这里的外部视觉，在本项目里就是 FAST-LIO2 通过桥接节点发给 PX4 的定位数据。

设置完成后需要重启飞控。

---

## 7. QGroundControl 启动

在电脑端执行：

```bash
cd ~/Downloads
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

QGroundControl 用于：

1. 查看飞控是否连接。
2. 查看飞控模式。
3. 修改 PX4 参数。
4. 观察 EKF、传感器和状态提示。
5. 必要时进行解锁、模式切换和安全检查。

---

## 8. 螺旋桨测试

> 安全要求：测试前必须卸下螺旋桨。不要带桨运行电机测试。

测试命令：

```bash
ros2 run velocity_offboard velocity_offboard_test \
  --ros-args \
  -p vx:=0.0 \
  -p vy:=0.0 \
  -p vz:=0.0 \
  -p auto_offboard:=true \
  -p auto_arm:=true
```

这个命令只用于检查 Offboard、解锁和电机响应链路，不代表可以直接飞行。

---

## 9. 常用话题说明

| 话题 | 来源 | 作用 |
|---|---|---|
| `/livox/lidar` | livox_ros_driver2 | Mid360s 点云 |
| `/livox/imu` | livox_ros_driver2 | Mid360s 内置 IMU |
| `/Odometry` | FAST-LIO2 | 雷达惯性定位结果 |
| `/cloud_registered` | FAST-LIO2 | 配准后的点云 |
| `/Laser_map` | FAST-LIO2 | 地图显示数据 |
| `/drone_0_planning/bspline` | Ego Planner | 规划出的 B 样条轨迹 |
| `/drone_0_planning/pos_cmd` | Ego Planner | 轨迹采样后的控制目标 |
| `/fmu/in/...` | ROS2 到 PX4 | 发给 PX4 的控制/定位输入 |
| `/fmu/out/...` | PX4 到 ROS2 | PX4 发出的状态、里程计等数据 |
| `/tf` | 多个节点 | 坐标关系 |
| `/clock` | Gazebo 仿真 | 仿真时间 |

---

## 10. 推荐启动顺序

### 10.1 仿真推荐顺序

```text
1. PX4 SITL
2. MicroXRCEAgent UDP
3. ROS Bridge + TF
4. RViz2
5. Ego Planner，按需要启动
6. 轨迹桥接 / Offboard 控制节点，按需要启动
```

### 10.2 真机推荐顺序

```text
1. 配置 Mid360s 网口 IP
2. 启动 livox_ros_driver2
3. 确认 /livox/lidar 和 /livox/imu 正常
4. 启动 FAST-LIO2
5. 确认 /Odometry 稳定
6. 启动 Ego Planner
7. 启动 MicroXRCEAgent 串口通信
8. 启动 fastlio_px4_bridge
9. 打开 QGroundControl 检查飞控状态
10. 卸桨后再测试 Offboard / 电机链路
```

---

## 11. 常见问题排查

### 11.1 Mid360s 没有点云

检查网口 IP：

```bash
ip addr
nmcli dev status
```

重新设置：

```bash
sudo ip addr replace 192.168.1.50/24 dev enP3p49s0
sudo ip link set enP3p49s0 up
```

检查话题：

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
```

如果仍然没有，检查：

1. 网线是否连接正确。
2. 雷达供电是否正常。
3. 香橙派有线网卡名是否真的是 `enP3p49s0`。
4. Livox 配置文件中雷达型号是否为 Mid360s。

---

### 11.2 FAST-LIO2 没有输出 `/Odometry`

检查输入：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

检查启动命令是否使用 Mid360s 配置：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

如果只有点云没有 IMU，FAST-LIO2 通常无法稳定定位。

---

### 11.3 FAST-LIO2 漂移严重

重点检查：

1. 点云和 IMU 时间戳是否正常。
2. 雷达和机体的安装方向是否配置正确。
3. FAST-LIO2 的 `mid360.yaml` 是否匹配当前雷达话题。
4. 雷达是否固定牢靠。
5. 环境是否太空旷、特征太少。

仿真中漂移不一定代表真机也漂移。Gazebo 里的雷达数据格式、点时间、IMU 噪声模型可能和真实 Mid360s 不一致。

---

### 11.4 Ego Planner 报 “drone is in obstacle”

含义：规划器认为无人机当前位置在障碍物里面。

常见原因：

1. 点云坐标系和定位坐标系没有对齐。
2. 雷达点云里包含了机体自身结构。
3. 膨胀半径设置过大。
4. 无人机刚启动时离墙、地面或障碍物太近。
5. FAST-LIO2 的 odom 和点云不是同一个坐标系。

处理方向：

1. 在 RViz2 中同时显示 `/Odometry`、`/cloud_registered` 和 TF。
2. 确认无人机模型位置不在点云障碍物内部。
3. 适当调整 Ego Planner 的障碍物膨胀参数。
4. 先在开阔环境测试。

---

### 11.5 Ego Planner 报 Astar 初始点或目标点错误

常见原因：

1. 起点在障碍物内。
2. 目标点在障碍物内。
3. 目标点超出局部地图范围。
4. 地图分辨率或局部地图大小不合适。

处理方向：

1. 把目标点设近一点。
2. 确认目标点高度合理，例如 `z=1.0` 或 `z=1.5`。
3. 确认当前地图里确实有可通行空间。
4. 先不要设置太远的航点。

---

### 11.6 MicroXRCEAgent 连不上飞控

检查：

```bash
ls /dev/ttyS*
ls /dev/ttyUSB*
```

确认启动命令中的串口是否正确：

```bash
MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600
```

如果仿真中使用 UDP，则应该是：

```bash
MicroXRCEAgent udp4 -p 8888
```

真机和仿真不要混用。

---

### 11.7 电脑端 RViz2 看不到香橙派话题

两边都执行：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

检查网络：

```bash
ping <对方IP>
```

如果能 ping 通但 `ros2 topic list` 看不到，检查是否有防火墙、DDS 网络发现问题，或者两边是否真的在同一个局域网。

---

## 12. 当前项目开发建议

当前项目建议按下面顺序推进：

1. 先保证真机 Mid360s 点云稳定。
2. 再保证 FAST-LIO2 `/Odometry` 稳定。
3. 再让 Ego Planner 只负责生成轨迹，不急着接飞控。
4. 再写或完善 `bspline_px4_bridge`，把规划轨迹转换成 PX4 setpoint。
5. 最后再做 Offboard 真机飞行测试。

不要一开始就把雷达、定位、规划、PX4、Offboard、QGC 全部接在一起调。这样出问题时很难判断是哪一层的问题。

---

## 13. 联系方式

如果对项目启动、参数、话题、桥接节点或飞控配置不明白，可以加 QQ：

```text
3378516198
```
