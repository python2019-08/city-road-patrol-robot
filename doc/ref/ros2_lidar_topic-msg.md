# `/scan` 与点云 `/points`（PointCloud2）
<!-- doubaoAI : doc/ref/ros2_lidar_topic-msg.md  -->
多线（3D）激光雷达的原生输出通常是 **PointCloud2 点云**（话题名随驱动而定，常见 `/points`、`/lidar/points`，本仓库 Gazebo 仿真中为 `/scan/points`）；
`/scan` 是 **sensor_msgs/LaserScan**，是**单平面极坐标测距**：多线雷达一般需要在后期**按高度切片，把选中的点投影到水平面**后才生成 `/scan`，不是原生数据。

## 1. 两个消息对比
| 话题 | 消息类型 | 数据内容 | 适用场景 | 多线雷达说明 |
|---|---|---|---|---|
| `/points` | `PointCloud2` | 每个点：x,y,z + intensity/ring/timestamp等，三维笛卡尔坐标 | 3D SLAM（LIO-SAM、FAST-LIO）、3D检测、高程、台阶、上下障碍物 | ✅ 原生输出，保留全部三维信息；数据量大、带宽高 |
| `/scan` | `LaserScan` | 单平面、角度数组 + range距离数组（极坐标） | 2D SLAM（slam_toolbox、gmapping）、AMCL、2D costmap导航 | ⚠️ 多数多线雷达驱动不直接输出 scan，要用 `pointcloud_to_laserscan` 取某一层/按高度切片投影，**丢掉 z 轴高度信息**，高处、低处障碍物可能丢失 |

> 典型驱动话题名：Velodyne → `/velodyne_points`；RoboSense → `/rslidar_points`；镭神 → `/lslidar_point_cloud`；禾赛 → `/pandar_points`（驱动版本不同名称可能不同）；Livox → `/livox/lidar`（**不是** `livox_points`）。具体以所用驱动的 launch 为准；它们发布的都是 PointCloud2，一般不会原生发 `/scan`

## 2. PointCloud2 内部字段（重点）
```cpp
Header header
uint32 height, width
PointField[] fields    # 常用字段：x,y,z,intensity,ring,time
                       # 名称随驱动而异：time/timestamp/offset_time、ring/line；Livox 无 ring
bool is_bigendian
uint32 point_step     # 单个点字节数
uint32 row_step
uint8[] data
bool is_dense
```
- 无序点云：`height=1, width=总点数`（如 Livox，或驱动配置为无序输出时）
- 有序点云：`height=线束数, width=每线点数`（如 Velodyne、Ouster 等驱动的 organized 输出）
- `ring` 字段：标记属于第几线，非常重要，滤波、去畸变、分层都依赖它

## 3. 工程里怎么互相转（以 ROS2 命令为例）
1. **PointCloud2 → LaserScan（你需要/scan给2D导航）**
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/scan/points -r scan:=/scan \
  -p target_frame:=base_link \
  -p min_height:=-0.10 -p max_height:=0.30
```
参数：`min_height / max_height`（相对 `target_frame`），只有落在该 z 区间内的点会投影成平面 scan，区间外的点（多数地面点、高处障碍物）被丢弃
> 坑：机器人上坡、地面不平的时候，投影出来的scan会失真、噪点爆炸

2. **LaserScan → PointCloud2**：ROS1 用 `laser_geometry` 包；ROS2 也可直接用 `pointcloud_to_laserscan` 包自带的 `laserscan_to_pointcloud_node`。一般单线雷达用，多线很少反向转

## 4. 选型/算法怎么选输入
- FAST-LIO、LIO-SAM（原版为 ROS1，ROS2 需用移植分支）、LOAM、RTAB-Map 3D模式、目标检测、地面分割 → **订阅点云（PointCloud2）**
- slam_toolbox、nav2_costmap_2d、AMCL（纯2D导航） → 订阅 `/scan`，但代价是丢失高度信息，容易踩台阶、撞悬空障碍物
- 很多机器人方案：**同时开两路**：原始点云做3D避障/建图；投影scan给传统2D导航栈

## 5. 常见踩坑
1. 直接拿多线雷达的scan跑2D SLAM，地面起伏大时定位飘，因为不是真正平面激光；
2. PointCloud2 带宽巨大：16 线约 3 万点/帧（≈30 万点/秒 @10 Hz），32 线约 7 万点/帧，64 线约 13~22 万点/帧，128 线可达几十万点/帧；不做降采样会 CPU/网卡扛不住；
3. 点云去畸变（deskew）应在 PointCloud2 上做（前提是点云带逐点时间字段）：LaserScan 只有 `time_increment` / `scan_time` 这种均匀扫描模型，多线点云投影后各点的原始时间戳已丢失，无法精确补偿运动畸变；
4. rviz可视化：`PointCloud2`用PointCloud2显示插件；`LaserScan`用LaserScan插件。

## 6. 常用命令调试（ROS2）
```bash
# 看话题类型
ros2 topic info /scan/points
# 查看点云字段
ros2 topic echo /scan/points --no-arr
# 可视化
rviz2
```

如果你需要，我可以给你一份 `pointcloud_to_laserscan` 完整参数yaml，或者FAST-LIO接收points的launch配置。