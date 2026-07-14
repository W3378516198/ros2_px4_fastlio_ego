import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():


    tf_node1 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mid360_static_tf',
            arguments=[
                '0', '0', '0.17', 
                '0', '0', '0', 
                'x500_0/base_link',         
                'mid360_link'  
            ],
            output='screen'
    )

    px4_odom_tf_node = Node(
        package='offboard_control',
        executable='odom_world',
        name='px4_odom_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )    



    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/mid360s/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # '/mid360s/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',            
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/mid360s/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
            # '/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )


    cloud_converter_node = Node(
        package='offboard_control',
        executable='cloud_converter_node',
        name='cloud_converter_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'input_topic': '/mid360s/points/points'},
            {'output_topic': '/ego_points'},
            {'target_frame': 'odom_world'},
            {'source_frame': 'mid360_link'},
            {'use_msg_frame': False},
            {'min_range': 0.2},
            {'max_range': 0.0},
            {'intensity': 1.0},
        ]
    )

    fastlio_cloud_converter_node = Node(
        package='offboard_control',
        executable='fastlio_cloud_converter',
        name='fastlio_cloud_converter',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'input_topic': '/mid360s/points/points'},
            {'output_topic': '/fastlio_points'},
            {'frame_id': 'mid360_link'},
            {'scan_period': 0.0666667},
            {'default_ring': 0},
        ]
    )
    fast_lio_dir = get_package_share_directory('fast_lio')
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[os.path.join(fast_lio_dir, 'config', 'sim.yaml'),{'use_sim_time': True}] # 确保路径和你的 yaml 名字对得上
        
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.1,
            'frame_id': 'camera_init',
            'sensor_model.max_range': 5.0,
            'use_sim_time': True
        }],
        # remappings=[('cloud_in', '/mid360s/points/points')]
        remappings=[('cloud_in', '/cloud_registered')]
    )


    
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner',
        output='screen',
        parameters=[
            {'use_sim_time': True},

            # =========================
            # GridMap 地图参数：必须补全
            # =========================
            {'grid_map/frame_id': "odom_world"},
            {'grid_map/pose_type': 1},
            {'grid_map/resolution': 0.1},

            {'grid_map/map_size_x': 20.0},
            {'grid_map/map_size_y': 20.0},
            {'grid_map/map_size_z': 5.0},

            {'grid_map/local_update_range_x': 4.0},
            {'grid_map/local_update_range_y': 4.0},
            {'grid_map/local_update_range_z': 2.5},

            {'grid_map/min_ray_length': 0.2},
            {'grid_map/max_ray_length': 8.0},

            {'grid_map/obstacles_inflation': 0.05},
            {'grid_map/local_map_margin': 10},

            {'grid_map/virtual_ceil_height': 4.0},
            {'grid_map/ground_height': -1.0},

            {'grid_map/visualization_truncate_height': 10.0},  # 可视化时将高度截断在这个值以下，避免显示过高的点云
            # =========================
            # FSM 参数：保留
            # =========================
            {'fsm/flight_type': 1},
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': 5.0},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/waypoint_num': 0},

            # =========================
            # 轨迹优化 manager 参数：保留
            # =========================
            {'manager/max_vel': 1.0},
            {'manager/max_acc': 1.0},
            {'manager/max_jerk': 1.0},
            {'manager/control_points_distance': 0.5},
            {'manager/planning_horizon': 5.0},
            {'manager/num_control_points': 10},
            {'manager/drone_id': 0},
            # =========================
            # B-spline 优化器参数：必须补全
            # =========================
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},

            {'optimization/dist0': 0.5},
            {'optimization/swarm_clearance': 0.5},

            {'optimization/max_vel': 1.0},
            {'optimization/max_acc': 1.0},

            {'optimization/order': 3},
        ],
        remappings=[
            ('/odom_world', '/odom_world'),
            ('/odom', '/odom_world'),
            ('/grid_map/odom', '/odom_world'),
            ('/pointcloud', '/ego_points'),
            ('/grid_map/cloud', '/ego_points'),
            ('/move_base_simple/goal', '/goal_pose')
        ]
    )

    bspline_px4_bridge_node = Node(
        package='offboard_control',
        executable='bspline_px4_bridge',
        name='bspline_px4_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': False},

            # 起飞/悬停点，单位是 odom_world / ENU，z 正向上
            {'takeoff_x': 0.0},
            {'takeoff_y': 0.0},
            {'takeoff_z': 2.0},

            # 到达一个 bspline 控制点的距离阈值
            {'reach_thresh': 0.35},

            # 如果超过这个时间还没到达当前点，就切到下一个点
            {'waypoint_timeout': 5.0},

            # 是否自动切 OFFBOARD + ARM
            {'auto_arm': True},

            # 如果你的 odom_world 和 PX4 x/y 存在交换，就改成 True
            {'swap_xy': True},
        ]
    )


    return LaunchDescription([
        tf_node1,
        bridge_node,
        px4_odom_tf_node,
        # fastlio_cloud_converter_node,
        # fast_lio_node,
        # octomap_node,
        cloud_converter_node,
        ego_planner_node,
        bspline_px4_bridge_node,
    ])