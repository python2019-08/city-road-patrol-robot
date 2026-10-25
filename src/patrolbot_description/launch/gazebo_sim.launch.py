import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取默认路径
    robot_name_in_model = "patrolbot"
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_path + '/urdf/fishbot/fishbot.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/custom_room.world.sdf'
    default_rviz_config_path = urdf_tutorial_path + '/config/rviz/display_model.rviz'

    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF 的绝对路径'
    )

    # 获取文件内容生成新的参数
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
            # 系统时间和仿真时间，RViz/slam_toolbox 查询 TF 时容易报错。
            'use_sim_time': True,
        }]
    )

    # ========== JAZZY 改动：替换 gazebo_ros 为 ros_gz_sim ==========
    # 启动Gazebo Harmonic，ros_gz_sim自带gz_sim.launch.py
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('gz_args', [default_world_path, ' -r'])
        ]
    )

    # # controller_manager，放到事件里，spawn成功之后再启动
    # controller_manager_node = launch_ros.actions.Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         PathJoinSubstitution([
    #             FindPackageShare('fishbot_description'),
    #             'config',
    #             'fishbot_ros2_controller.yaml'
    #         ])
    #     ],
    #     remappings=[
    #         ('fishbot_diff_drive_controller/odom', 'odom')
    #     ],
    #     output='screen'
    # )

    # spawn 机器人：ros_gz_sim create 替代原来 gazebo_ros spawn_entity.py
    spawn_entity_node = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name_in_model,
        ],
        output='screen'
    )

    # # 2. spawn diff_drive控制器！remap写在这里！
    # spawn_diff_drive = launch_ros.actions.Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "fishbot_diff_drive_controller",
    #         "--controller-manager", "/controller_manager",
    #         "--controller-ros-args",
    #         "--ros-args "
    #         "--remap /fishbot_diff_drive_controller/cmd_vel:=/cmd_vel_01by_ros2control",
    #         "--remap /fishbot_diff_drive_controller/odom:=/odom_01by_ros2control",
    #     ],
    #     output="screen",
    # )


    # 加载控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_diff_drive_controller'],
        output='screen'
    )

    gz_bridge_node = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 仿真时钟：use_sim_time 的节点需要它，否则时钟恒为 0
            # （表现：odom 因 publish_rate 节流永不发布、cmd_vel_timeout 永不过期、TF 时间戳为 0）
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # 多线激光雷达点云（32 线默认 / 16 线可选）。不直接桥接 /scan：
            # 多线时它只含中间一条线束。Nav2 请用 pointcloud_to_laserscan
            # 从点云生成水平 /scan，见 urdf/README.md 第 8 节。
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # 注意：**不要同时开两份 joint_states**，容易造成 TF 抖动！
            # 使用ros2_control joint_state_broadcaster时，请注释掉下面这一行
            # '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_sensor/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # 开启深度相机点云才启用下面这行
            '/camera_sensor/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # 
            '/gz/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 支持 gz.msgs.Twist ↔ geometry_msgs/msg/TwistStamped 转换
            '/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist',
        ],
        output='screen'
    )

 
    ld = launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        # spawn_diff_drive,
        gz_bridge_node,
        # 事件链：spawn成功 →  加载joint_state → 加载diff drive
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_fishbot_diff_drive_controller],
            )
        ), 
    ])
    return ld

