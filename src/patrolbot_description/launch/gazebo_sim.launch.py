import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_name_in_model = 'patrolbot'

    pkg_share = get_package_share_directory('patrolbot_description')
    default_model_path = os.path.join(
        pkg_share, 'urdf', 'patrolbot.urdf.xacro')
    default_world_path = os.path.join(
        pkg_share, 'world', 'city_road.world.sdf')
    gps_fusion_config = os.path.join(
        pkg_share, 'config', 'gps_fusion.yaml')

    # ---------------------------------------------------------------- args
    action_declare_arg_model = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='机器人 URDF/Xacro 的绝对路径'
    )
    action_declare_arg_world = launch.actions.DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Gazebo 世界 SDF 的绝对路径'
    )

    # xacro 生成 URDF 字符串，robot_state_publisher 同时把它发布到
    # /robot_description；ros_gz_sim create 从该话题读取模型。
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            # 仿真里必须和 Gazebo 用同一个 /clock，否则 TF 时间戳会混用
            # 系统时间和仿真时间，RViz/Nav2 查询 TF 时容易报错。
            'use_sim_time': True,
        }]
    )

    # -------------------------------------------------------------- Gazebo
    # ros_gz_sim 自带 gz_sim.launch.py；-r 立即运行物理。
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('gz_args', [
                launch.substitutions.LaunchConfiguration('world'),
                ' -r',
            ])
        ]
    )

    # 等 Gazebo 世界和 /robot_description 就绪后再生成机器人。
    # 出生点就是 city_road 应急车道起点：world (0, 0)，朝向 +X。
    spawn_entity_node = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name_in_model,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02',
        ],
        output='screen'
    )
    delayed_spawn = launch.actions.TimerAction(
        period=5.0,
        actions=[spawn_entity_node]
    )

    # --------------------------------------------------------- controllers
    spawner_joint_state = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '60.0',
        ],
        output='screen'
    )

    spawner_diff_drive = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager-timeout', '60.0',
        ],
        output='screen'
    )

    # --------------------------------------------------------------- bridge
    gz_bridge_node = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 仿真时钟：use_sim_time 的节点都需要它，否则时钟恒为 0
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Gazebo IMU -> /imu/data（与 sensors.xacro 的话题一致）
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Gazebo navsat -> sensor_msgs/NavSatFix，供 navsat_transform
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # 多线激光雷达点云：/scan 是 LaserScan，/scan/points 是
            # PointCloud2。Nav2 用 pointcloud_to_laserscan 从点云生成
            # 水平 /scan，见 doc/1.imp/0.urdf.generate.md 第 8 节。
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # 全景相机
            '/camera/panorama/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/panorama/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # 云台相机
            '/camera/gimbal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/gimbal/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # 底盘速度由 diff_drive_controller 订阅
            # /diff_drive_controller/cmd_vel 后写入 ros2_control，
            # 不再直接桥接 Gazebo 的 /cmd_vel。
            #
            # 不要桥接 /joint_states：控制器已经在发
            # joint_state_broadcaster 的结果，两份 joint_states 会让 TF 抖动。
        ],
        output='screen'
    )

    # ------------------------------------------------------ GPS / localization
    # 数据流：
    #   Gazebo navsat -> /gps/fix -> navsat_transform -> /odometry/gps
    #     -> global EKF -> map -> odom -> base_footprint
    # local EKF 只融合轮速 + IMU，负责 odom -> base_footprint；
    # global EKF 融合轮速 + IMU + GPS，负责 map -> odom。
    ekf_local = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        parameters=[gps_fusion_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')],
        output='screen'
    )

    ekf_global = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        parameters=[gps_fusion_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')],
        output='screen'
    )

    navsat_transform_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[gps_fusion_config, {'use_sim_time': True}],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered'),
        ],
        output='screen'
    )

    ld = launch.LaunchDescription([
        action_declare_arg_model,
        action_declare_arg_world,
        robot_state_publisher_node,
        launch_gazebo,
        delayed_spawn,
        gz_bridge_node,
        # 事件链：spawn 成功 -> joint_state_broadcaster -> diff_drive_controller
        # -> GPS 融合节点。local EKF 启动后才会建立 odom -> base_footprint；
        # navsat_transform 的 delay: 3.0 就是留给它建立 TF 再做天线杆臂补偿。
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[spawner_joint_state],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawner_joint_state,
                on_exit=[spawner_diff_drive],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawner_diff_drive,
                on_exit=[
                    ekf_local,
                    # 先让 local EKF 建立 odom -> base_footprint，再启动
                    # global EKF（它要靠这条 TF 发 map -> odom 并做 GPS
                    # 杆臂补偿）；最后启动 navsat_transform（它本身还有
                    # delay: 3.0 s）。
                    launch.actions.TimerAction(
                        period=3.0, actions=[ekf_global]),
                    launch.actions.TimerAction(
                        period=6.0, actions=[navsat_transform_node]),
                ],
            )
        ),
    ])
    return ld
