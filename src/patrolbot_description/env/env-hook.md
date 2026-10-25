# ch01.uri [file://cafe_table/meshes/cafe_table.dae] could not be resolved

问题如下:
```sh
(base) abner@abner-XPS-8940:~$ gz sim /home/abner/a2/zdev/nv/adas-01/ros/ros2bookcode_jazzy/chapt7_ws/src/fishbot_description/world/custom_room.world.sdf

[Err] [Server.cc:86] Error Code 14: Msg: Parser configurations requested resolved uris, but uri [model://cafe_table/meshes/cafe_table.dae] could not be resolved.
[Err] [Server.cc:86] Error Code 14: Msg: Parser configurations requested resolved uris, but uri [model://cafe_table/meshes/cafe_table.dae] could not be resolved.
[Err] [Server.cc:86] Error Code 14: Msg: Parser configurations requested resolved uris, but uri [model://cafe_table/meshes/cafe_table.dae] could not be resolved.
[Err] [Server.cc:86] Error Code 9: Msg: Failed to load a world.

------

(base) abner@abner-XPS-8940:~$ gz sim /home/abner/a2/zdev/nv/adas-01/ros/ros2bookcode_jazzy/chapt7_ws/src/fishbot_description/world/custom_room.world.sdf

[Err] [Server.cc:86] Error Code 14: Msg: Parser configurations requested resolved uris, but uri [file://cafe_table/meshes/cafe_table.dae] could not be resolved.
[Err] [Server.cc:86] Error Code 14: Msg: Parser configurations requested resolved uris, but uri [file://cafe_table/meshes/cafe_table.dae] could not be resolved.
[Err] [Server.cc:86] Error Code 14: Msg: Parser configurations requested resolved uris, but uri [file://cafe_table/meshes/cafe_table.dae] could not be resolved.
[Err] [Server.cc:86] Error Code 9: Msg: Failed to load a world.

------

(base) abner@abner-XPS-8940:~$ ls /home/abner/a2/zdev/nv/adas-01/ros/ros2bookcode_jazzy/chapt7_ws/src/fishbot_description/world/ -l
total 88
drwxrwxr-x 5 abner abner  4096 Oct 16 09:28 cafe_table
-rw-rw-r-- 1 abner abner 80257 Oct 16 09:29 custom_room.world.sdf
drwxrwxr-x 2 abner abner  4096 Sep  8 16:29 room

------

(base) abner@abner-XPS-8940:~$ ls /home/abner/a2/zdev/nv/adas-01/ros/ros2bookcode_jazzy/chapt7_ws/src/fishbot_description/world/cafe_table/ -l
total 24
drwxrwxr-x 3 abner abner 4096 Jul 22  2025 materials
drwxrwxr-x 2 abner abner 4096 Jul 22  2025 meshes
-rw-rw-r-- 1 abner abner  937 Jul 22  2025 model-1_4.sdf
-rw-rw-r-- 1 abner abner  328 Jul 22  2025 model.config
-rw-rw-r-- 1 abner abner  937 Jul 22  2025 model.sdf
drwxrwxr-x 2 abner abner 4096 Jul 22  2025 thumbnails
```

## 用 env-hook 一劳永逸（工程级）
 
下面给你一套**能直接照抄、不会踩坑的完整工程级写法**（ROS 2 Jazzy / ament_cmake）。

---

## 一、目录结构（确认对齐）

```
fishbot_description/
├── CMakeLists.txt
├── package.xml
├── env/
│   └── set_gz_paths.dsv
└── world/
    ├── custom_room.world.sdf
    ├── cafe_table/
    │   ├── model.sdf
    │   ├── model.config
    │   └── meshes/cafe_table.dae
    └── room/
```

---

## 二、`.dsv` 文件（关键细节）

> ⚠️ 这里有个**最容易写错的地方**：`.dsv` 里的路径是**install 后的相对路径**，不是源码绝对路径。

### ✅ 正确写法（推荐）

`fishbot_description/env/set_gz_paths.dsv`

```
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share/fishbot_description/world
```

含义拆解：

| 字段 | 说明 |
|---|---|
| `prepend-non-duplicate` | 追加到环境变量最前面，不重复 |
| `GZ_SIM_RESOURCE_PATH`  | Gazebo Sim 认的变量 |
| `share/fishbot_description/world` | 相对于 `${AMENT_PREFIX_PATH}` 的路径 |

install 后实际展开为：

```
/opt/ros/jazzy/share/fishbot_description/world
# 或 workspace 下：
~/ros2_ws/install/fishbot_description/share/fishbot_description/world
```

这正是 Gazebo 需要的：

```
<resource_path>/cafe_table/meshes/cafe_table.dae
```

---

## 三、`CMakeLists.txt` 必须加的两行

```cmake
# 安装 env hook
install(DIRECTORY env
  DESTINATION share/${PROJECT_NAME}
)

# 注册 ament environment hook
ament_environment_hooks("env/set_gz_paths.dsv")
```

完整最小可用片段（只保留相关部分）：

```cmake
cmake_minimum_required(VERSION 3.8)
project(fishbot_description)

find_package(ament_cmake REQUIRED)

# 安装 URDF / meshes / worlds
install(DIRECTORY urdf meshes world
  DESTINATION share/${PROJECT_NAME}
)

# 安装 env hook
install(DIRECTORY env
  DESTINATION share/${PROJECT_NAME}
)

# 注册 hook（这一行不能少）
ament_environment_hooks("env/set_gz_paths.dsv")

ament_package()
```

> ✅ `ament_environment_hooks()` **不需要** `find_package(ament_environment_hooks)`，
> 它是 `ament_cmake` 内置宏。

---

## 四、`package.xml` 无需额外依赖

不需要加 `<depend>ament_environment_hooks</depend>`（不是独立包）。

保持正常即可：

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<exec_depend>ros_gz_sim</exec_depend>
```

---

## 五、验证是否真的生效（很重要）

重新编译并 source：

```bash
colcon build --packages-select fishbot_description
source install/setup.bash
```

检查 hook 是否被解析：

```bash
echo $GZ_SIM_RESOURCE_PATH
```

你应该能看到类似：

```
/home/abner/ros2_ws/install/fishbot_description/share/fishbot_description/world:...
```

再确认路径下真的有 `cafe_table`：

```bash
ls $GZ_SIM_RESOURCE_PATH/cafe_table/meshes/
# 应该能看到 cafe_table.dae
```

然后直接跑，不用再 export：

```bash
gz sim $(ros2 pkg prefix fishbot_description)/share/fishbot_description/world/custom_room.world.sdf
```

或者走 launch：

```bash
ros2 launch fishbot_description start_custom_room.launch.py
```

---

## 六、什么时候 launch 里还需要 SetEnvironmentVariable？

| 场景      | 是否需要 launch 里再设 |
|--------------------------------------------|----------|
| 已 source install/setup.bash，env-hook 生效 | ❌ 不需要 |
| CI / 容器里没 source，直接 `ros2 launch`     | ✅ 兜底加一个 `AppendEnvironmentVariable` |
| 临时调试、快速验证                            | ✅ 终端手动 export |

工程最佳实践：**env-hook 做默认保障，launch 里只兜底，不重复覆盖**。

---

## 七、一个常见翻车点（提前帮你避坑）

如果 install 后你发现：

```bash
ls install/fishbot_description/share/fishbot_description/environment/
# 只有 set_gz_paths.dsv
# 但 world/ 目录没被装进去
```

说明你忘了 `install(DIRECTORY world ...)`。

没有 `world/` 被安装 → hook 指向的路径存在但里面是空的 → Gazebo 依然报 URI 解析失败。

快速自检：

```bash
ls install/fishbot_description/share/fishbot_description/world/cafe_table/meshes/
```

能看到 `.dae` 就稳了。

---

## 一句话总结

> **env-hook 不是只放个 `.dsv` 就完事**：  
> `CMakeLists.txt` 里 `install(DIRECTORY environment ...)` + `ament_environment_hooks(...)` 两步缺一不可，install 后 source 一次，所有终端 / launch 自动继承 `GZ_SIM_RESOURCE_PATH`，彻底告别手动 export。

如果编译完 `echo $GZ_SIM_RESOURCE_PATH` 还是空的，把你的 `CMakeLists.txt` 完整贴出来，我帮你逐行对。