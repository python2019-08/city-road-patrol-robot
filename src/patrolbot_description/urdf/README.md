# patrolbot_description / urdf

智能巡逻告警机器人（smart patrol &amp; alarm robot）的 URDF/Xacro 模型，
面向 **ROS 2 Jazzy + Gazebo Harmonic**。

- 外形：交通锥筒车身，橙色 + 三道白色反光带
- 底盘：0.42 × 0.32 × 0.08 m，离地 0.04 m，四轮两电机差速驱动
- 传感器：100 m 激光雷达、IMU、RTK/GNSS、1080p 全景相机、云台长焦相机
  （mmWave 与超声波目前只有坐标帧，见第 10 节）
- 执行器：三节电动升降杆（最大抬升 2.5 m）+ 三轴增稳云台
- 整机质量约 29.4 kg（要求 ≤ 35 kg）

---

## 1. 文件说明

| 文件 | 内容 |
| --- | --- |
| `patrolbot.urdf.xacro` | 入口文件：声明参数、包含子文件、调用各宏 |
| `common.xacro` | 尺寸/质量参数、材质、惯量宏 |
| `chassis.xacro` | 底盘盒体、四个车轮及接触参数 |
| `body.xacro` | 锥筒车身、LED 显示屏、红蓝爆闪灯、定向喇叭 |
| `mast_gimbal.xacro` | 三节电动升降杆与三轴云台 |
| `sensors.xacro` | 雷达、IMU、GNSS、两台相机、mmWave/超声波坐标帧 |
| `ros2_control.xacro` | `gz_ros2_control` 硬件接口与控制器插件 |
| `../config/ros2_control.yaml` | 控制器配置（`diff_drive_controller` 等） |
| `../meshes/*.stl` | 锥筒、反光带、LED 屏；由 `meshes/generate_meshes.py` 生成 |

```bash
# 只做语法/结构检查，不需要编译
xacro patrolbot.urdf.xacro > /tmp/patrolbot.urdf
check_urdf /tmp/patrolbot.urdf
gz sdf -p /tmp/patrolbot.urdf > /tmp/patrolbot.sdf   # 验证 Gazebo 能解析
```

## 2. 坐标系与 TF 树

`base_footprint` 是地面投影根节点，供 Nav2 / robot_localization 使用；
`base_link` 位于轮轴高度（离地 0.080 m）。

```
base_footprint
└── base_link                          z = +0.080
    ├── body_link                      z = +0.040（底盘顶面）
    │   ├── led_bezel_link → led_screen_link
    │   ├── strobe_red_link / strobe_blue_link
    │   ├── speaker_link
    │   ├── lidar_link
    │   ├── panorama_camera_link → panorama_camera_optical_frame
    │   └── mast_base_link             x = -0.055
    │       └── mast_stage1_link
    │           └── mast_stage2_link   prismatic，0 … 0.525 m
    │               └── mast_stage3_link
    │                   └── gimbal_yaw_link    revolute，±π
    │                       └── gimbal_pitch_link  revolute，±1.4
    │                           └── gimbal_roll_link   revolute，±0.6
    │                               └── gimbal_camera_link → gimbal_camera_optical_frame
    ├── front_left_wheel_link / front_right_wheel_link
    ├── rear_left_wheel_link  / rear_right_wheel_link
    ├── imu_link
    ├── gnss_link
    ├── mmwave_link
    └── {front_left,front_right,rear_left,rear_right,left,right}_ultrasonic_link
```

固定关节在 SDF 中会被 sdformat 合并到父连杆（子连杆消失、视觉改名
`*_fixed_joint_lump__*`），但 `robot_state_publisher` 仍然发布全部 TF 帧，
上层导航/感知代码不受影响。

## 3. 主要尺寸与质量

| 项目 | 数值 |
| --- | --- |
| 底盘盒体 | 0.420 × 0.320 × 0.080 m，离地间隙 0.040 m |
| 车轮 | 半径 0.080 m，宽 0.040 m；轮距 0.360 m；轴距 0.280 m |
| 锥筒车身 | 高 0.740 m，底半径 0.165 m，顶半径 0.100 m；底面 0.120 m，顶面 0.860 m |
| 反光带中心（相对 `body_link`） | 0.090 / 0.610 / 0.700 m |
| LED 屏中心（相对 `body_link`） | 0.320 m |
| 激光雷达扫描面 | 0.882 m（雷达顶 0.917 m） |
| 云台相机光心（收杆） | 0.9425 m；整机最高点约 0.955 m |
| 云台相机光心（完全升起） | **2.5175 m**（3 × 0.525 m 行程，满足 ≥ 2.5 m） |
| 总质量 | 约 29.4 kg（要求 ≤ 35 kg） |

说明：需求中的“整机高度约 90 cm”对应锥筒车身（顶面 0.860 m）；收杆时
云台相机比锥顶高约 8 cm，这是为了让长焦相机越过雷达向前看所必需的。

## 4. 传感器与话题

| 传感器 | Gazebo 类型 | 话题 | 帧 | 频率 | 主要参数 |
| --- | --- | --- | --- | --- | --- |
| 100 m 激光雷达 | `gpu_lidar` | `/scan` | `lidar_link` | 10 Hz | 720 线，−π…π，0.20–100 m，σ = 0.01 m |
| IMU | `imu` | `/imu/data` | `imu_link` | 100 Hz | 角速度 σ 0.0015 rad/s，加速度 σ 0.02 m/s² |
| RTK/GNSS | `navsat` | `/gps/fix` | `gnss_link` | 10 Hz | 水平 σ 0.02 m，垂直 σ 0.04 m |
| 全景相机 | `camera` | `/camera/panorama/image_raw` | `panorama_camera_optical_frame` | 25 Hz | 1920×1080，HFOV 2.4 rad（≈137°），0.05–100 m |
| 云台相机 | `camera` | `/camera/gimbal/image_raw` | `gimbal_camera_optical_frame` | 25 Hz | 1920×1080，HFOV 0.6 rad（≈34°），0.10–250 m |
| mmWave 雷达 | — | — | `mmwave_link` | — | 车前 0.222 m，仅坐标帧 |
| 超声波 ×6 | — | — | `*_ultrasonic_link` | — | 四角 + 两侧，仅坐标帧 |

`<topic>` 是 **Gazebo** 话题名，经 `ros_gz_bridge` 后沿用同名 ROS 话题。
`navsat` 需要在 world 中提供 `<spherical_coordinates>` 才会有有意义的经纬度。
Gazebo Harmonic 的相机还会在 `<camera>/camera_info` 上发布内参
（`gz.msgs.CameraInfo`），桥接到 `sensor_msgs/msg/CameraInfo` 后可直接给
image_proc / 标定流程使用（见第 8 节）。

## 5. 驱动与控制器

- 四个轮关节通过 `gz_ros2_control` 暴露为 **速度命令 + 位置/速度状态**；
  同侧前/后轮在 `diff_drive_controller` 中作为“一个虚拟轮”处理，
  实现“四轮两电机”的驱动方案。
- 控制器由 `config/ros2_control.yaml` 配置：

| 接口 | 话题 | 类型 |
| --- | --- | --- |
| 速度输入 | `/diff_drive_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| 里程计输出 | `/diff_drive_controller/odom` | `nav_msgs/msg/Odometry` |
| 关节状态 | `/joint_states` | `sensor_msgs/msg/JointState` |
| TF | `odom → base_footprint` | 由控制器发布 |

注意 Jazzy 的 `diff_drive_controller` 只接受 `TwistStamped`
（`teleop_twist_keyboard` 发布 `Twist`，需要 `twist_stamper` 转换）。

升降杆与云台**没有**接入 ros2_control，保持被动（阻尼 + 摩擦定位），
避免仿真启动时被控制器抢走。若需要模拟举升/增稳，可把
`mast_lift_{1,2,3}_joint` 改为 `position` 命令接口并新增控制器。

## 6. Xacro 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `controllers_file` | `$(find patrolbot_description)/config/ros2_control.yaml` | 控制器配置 |
| `lidar_samples` | `720` | 每圈点数 |
| `lidar_max_range` | `100.0` | 最大量程（m） |
| `lidar_update_rate` | `10.0` | Hz |
| `imu_update_rate` | `100.0` | Hz |
| `gnss_update_rate` | `10.0` | Hz |
| `panorama_width` / `panorama_height` | `1920` / `1080` | 全景相机 |
| `panorama_rate` / `panorama_hfov` | `25.0` / `2.4` | 频率 / 水平视场角（rad） |
| `gimbal_camera_width` / `..._height` | `1920` / `1080` | 云台相机 |
| `gimbal_camera_rate` / `..._hfov` | `25.0` / `0.6` | 频率 / 水平视场角（rad） |

## 7. Gazebo world 需要的系统插件

模型本身只带 `gz_ros2_control` 插件；传感器与物理需要在 world 中启用：

```xml
<plugin filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system"
        name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster"/>
<!-- gpu_lidar 与 camera 都依赖渲染系统，必须使用 ogre2 -->
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-navsat-system"
        name="gz::sim::systems::NavSat"/>

<!-- GNSS 需要球面坐标原点 -->
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>31.2304</latitude_deg>
  <longitude_deg>121.4737</longitude_deg>
  <elevation>0</elevation>
</spherical_coordinates>
```

## 8. ros_gz_bridge 映射

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat \
  /camera/panorama/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera/gimbal/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera/panorama/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /camera/gimbal/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

`/diff_drive_controller/*` 和 `/joint_states` 由 ROS 侧控制器产生，不需要桥接。
使用仿真时间时记得设置 `use_sim_time:=true`。

## 9. 构建与运行

```bash
# 1) 编译
colcon build --packages-select patrolbot_description
source install/setup.bash

# 2) 生成 URDF
xacro install/patrolbot_description/share/patrolbot_description/urdf/patrolbot.urdf.xacro \
  > /tmp/patrolbot.urdf

# 3) 启动 Gazebo（world 需包含第 7 节的插件；去掉 -s 打开 GUI）
gz sim -r -s your_world.sdf

# 4) 发布 robot_description 并生成模型
ros2 run robot_state_publisher robot_state_publisher /tmp/patrolbot.urdf
ros2 run ros_gz_sim create -topic robot_description -name patrolbot -z 0.05

# 5) 加载控制器
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner diff_drive_controller

# 6) 启动桥接（见第 8 节），然后下发速度
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.5}}}"
```

## 10. 设计说明与已知限制

- **雷达布局**：雷达装在锥顶前方，扫描面 0.882 m。前向与两侧的所有几何
  （锥顶 0.860 m、爆闪灯顶 0.875 m、升降杆法兰顶 0.870 m、全景相机顶
  0.793 m）都低于扫描面，因此前向/侧向无自遮挡。唯一穿过扫描面的是
  后方的云台头：收杆时遮挡约 **±21°**（正后方）；升降杆升起后云台让开，
  只有第一节套管穿过扫描面，遮挡收窄到约 **±18°**。这是有意取舍——
  把升降杆放在后方，把雷达的前向视野做到最大。
- **雷达最小量程 0.20 m**：扫描面附近的车体、地面回波和近距离杂点在此
  范围内被丢弃，0.2 m 以内的障碍由车身周围的超声波负责。
- **视觉 vs 碰撞**：车身、底盘使用简化碰撞体（锥筒用三段圆柱近似）；反光
  带、LED、云台等只有视觉，不参与碰撞，避免自碰撞和仿真抖动。
- **材质**：LED 屏、爆闪灯只是纯色材质，不发光。需要“点亮”效果时，可在
  world 中给对应视觉加 emissive 材质或加 `<light>`。
- **升降杆/云台是被动关节**（见第 5 节），喇叭/灯光/声音也仅是模型，
  没有对应 Gazebo 系统。
- **mmWave 与超声波**只有坐标帧和外观，接入插件后即可输出数据。
- **`package://` 解析**：正常编译安装后 Gazebo 通过 ament 索引找到网格。
  若在未安装的工作区里直接运行而报找不到网格，可设置
  `export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix patrolbot_description)/share`。
- **车轮接触参数**：`mu1 = mu2 = 1.0`、`kp = 1e6`、`kd = 100`，配合
  `diff_drive_controller` 的加速度限制，可减少低速打滑和抖动。
- **仿真中的雷达“近距裁剪”**：Ogre2 后端的 GPU 雷达不渲染距离小于
  `range.min` 的几何，射线会直接穿过去。因此本模型中 `range.min = 0.20 m`
  时，收杆状态的云台头（离扫描面约 0.09 m）不会在 `/scan` 上形成上面
  说的 ±21° 后向盲区，仿真里 360° 都有数据。真机上该盲区仍然存在。
  若要在仿真里复现自遮挡，需要把 `range.min` 降到 0.08 m 以下（修改
  `sensors.xacro`），代价是 0.2 m 内的地面和车体杂点会进入扫描。
- **四轮滑移转向的里程计偏差**：同侧前后轮由同一电机驱动、间距 0.28 m，
  原地转向时四个轮子都要横向擦地（skid steer）。实测仿真中指令
  0.6 rad/s 的实际偏航约 0.43 rad/s，而按轮速积分的 `/odom` 仍按
  0.6 rad/s 计算，因此 `/odom` 的偏航和位移在纯转向时会明显超前
  （一轮约 3 s 的原地转向后偏差可达 30%–45%）。直线行驶时两者一致。
  真机同样存在这种现象，建议用 IMU + RTK 融合定位；如果必须用轮式
  里程计，按实测把 `wheel_separation_multiplier` 调到约 1.4 作为
  经验校准值。
- **必须桥接 `/clock`**：控制器的 `use_sim_time` 由 `gz_ros2_control`
  自动置为 `true`。若只桥接传感器话题而漏掉 `/clock`，控制器能正常
  加载和运动，但 `/joint_states` 时间戳恒为 0，`/diff_drive_controller/odom`
  也不会发布。

## 11.总结
### 关键参数

┌──────────┬────────────────────────────────────────────────────────────┐
│   项目   │                             值                             │
├──────────┼────────────────────────────────────────────────────────────┤
│ 底盘     │ 0.42×0.32×0.08 m，离地 0.04 m，轮距 0.36 m，轴距 0.28 m    │
├──────────┼────────────────────────────────────────────────────────────┤
│ 锥筒车身 │ 顶面 0.860 m（约 90 cm），三道反光带                       │
├──────────┼────────────────────────────────────────────────────────────┤
│ 雷达     │ 扫描面 0.882 m，720 线，±π，0.2–100 m，10 Hz               │
├──────────┼────────────────────────────────────────────────────────────┤
│ 升降杆   │ 3 × 0.525 m 行程，云台相机最高 2.5175 m                    │
├──────────┼────────────────────────────────────────────────────────────┤
│ 云台     │ yaw ±π、pitch ±1.4、roll ±0.6                              │
├──────────┼────────────────────────────────────────────────────────────┤
│ 质量     │ 约 29.4 kg（要求 ≤35 kg）                                  │
├──────────┼────────────────────────────────────────────────────────────┤
│ 驱动     │ 四轮两电机，同侧前后轮作为虚拟轮交给 diff_drive_controller │
└──────────┴────────────────────────────────────────────────────────────┘

### 验证结果

- xacro → check_urdf → gz sdf -p 全部通过（11 links / 10 joints）；colcon build 通过，安装后 package:// 网格解析为 model://，无报错。
- Gazebo Harmonic 无界面启动、spawn 成功；/scan、/imu/data、/gps/fix、两台相机的 image_raw 与 camera_info 均正常发布。
- 控制器：joint_state_broadcaster 与 diff_drive_controller 加载并激活；下发 0.5 m/s 后 odom 与实际模型位姿一致（2.20835 vs 2.20841 m），转向指令也正常。
- 期间修复了一个配置错误：Jazzy 的 diff_drive_controller 要求 max_deceleration ≤ 0、max_acceleration_reverse ≤ 0，已改为负值并在 YAML 中注明符号约定，这是之前加载失败的根因。

### 已知限制（README 第 10 节有详细说明）

- 雷达被后方云台头遮挡 ±21°（收杆）/ ±18°（升起），这是真机属性；仿真中 GPU 雷达的近距裁剪会让 0.2 m 内的遮挡物直接穿透，需把 range.min 降到 <0.08 m 才能复现。
- 四轮滑移转向在纯原地转向时有横向擦地，轮速里程计的偏航会比真实运动超前约 30–45%，建议真机用 IMU+RTK 融合或校准 wheel_separation_multiplier。
- 升降杆/云台为被动关节；mmWave 和超声波目前只有坐标帧。
- 本机 colcon build 会优先调用 conda 的 python 而缺少 catkin_pkg，可用 -DPython3_EXECUTABLE=/usr/bin/python3 规避。