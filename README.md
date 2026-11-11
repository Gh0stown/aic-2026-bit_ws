# aic-2026-bit_ws

2026 智能车 / 巡检机器人工作区（ROS1 Noetic + catkin）。

两轮差速底盘（带麦轮，可横移），激光雷达 + IMU + 轮式里程计经 EKF 融合做里程计，
支持 Karto / Gmapping 建图、AMCL + move_base 导航、20 点位巡检巡航（图形化），
以及摄像头拍照 / YOLO 检测 / 区域扫描拼图。

## 硬件与场地

| 项目 | 说明 |
| --- | --- |
| 底盘 | 两轮差速 + 麦轮平移，车宽约 30cm；串口 `/dev/carserial`，115200 |
| 雷达 | 镭神 lslidar，发布 `/scan`，frame `laser_link` |
| IMU | WIT，发布 `/wit/imu` |
| 相机 | UVC 摄像头 `/dev/video0`（拍照用）；Orbbec Dabai DCW2（深度，可选） |
| 场地 | 4.4m × 4.4m 方形，道路宽约 60cm；地图上只有四周墙体 |

## 环境与编译

Ubuntu 20.04 + ROS Noetic。

```bash
sudo apt install ros-noetic-desktop-full ros-noetic-robot-localization \
     ros-noetic-gmapping ros-noetic-map-server ros-noetic-amcl \
     python3-opencv python3-tk python3-pil.imagetk

cd ~/bit_ws
catkin_make
source devel/setup.bash
```

- 本机 `~/.bashrc` 已 source `/opt/ros/noetic/setup.bash` 和 `devel/setup.bash`，新终端一般不用手动 source。
- YOLO（ultralytics）和模型文件（`~/AI视觉/*.pt`）不在仓库内，需要单独安装 / 拷贝。
- `build/`、`devel/`、`logs/` 和巡航调试日志已在 `.gitignore` 中，不要提交。

## 快速开始

| 场景 | 命令 |
| --- | --- |
| Karto 建图（主力） | `roslaunch ~/bit_ws/src/karto.launch` |
| Gmapping 建图 | `roslaunch ~/bit_ws/src/gmapping.launch` |
| 保存地图 | `~/bit_ws/src/nav_demo/scripts/save_map.sh` |
| 底盘速度下发（要动车必开） | `rosrun car_bringup newt.py` |
| 键盘遥控 | `roslaunch mbot_teleop mbot_teleop.launch` |
| 导航（雷达/定位/规划/rviz） | `roslaunch ~/bit_ws/src/nav777.launch` |
| 图形化巡航（当前主用） | `rosrun nav_demo cruise_debug.py --route left` |
| 记录巡检点位 | `rosrun nav_demo waypoint_gui.py` |
| 拍照 / 模型测试 | `rosrun photo_capture capture_photos.py` |
| 区域扫描拼图 | `rosrun photo_capture capture_photos.py --mode scan` |
| 3D 相机 | `roslaunch orbbec_camera dabai_dcw2.launch` |

完整流程：建图 → 定点位 → 巡航。

```bash
# 1) 建图：跑完先保存，地图在节点内存里
roslaunch ~/bit_ws/src/karto.launch
rosrun car_bringup newt.py
roslaunch mbot_teleop mbot_teleop.launch
~/bit_ws/src/nav_demo/scripts/save_map.sh

# 2) 定点位：起导航（用新图就加 map:=新图.yaml），遥控到位后在图里记录
roslaunch ~/bit_ws/src/nav777.launch
rosrun car_bringup newt.py
rosrun nav_demo waypoint_gui.py

# 3) 巡航（沿用上一步的导航，不用重开）
rosrun nav_demo cruise_debug.py --route left
```

> `newt.py` 负责把 `/cmd_vel` 写给底盘，建图和导航的 launch 里只有读串口的
> `pubv.py` 和算里程计的 `base_node`，**不能**代替它。

## 系统结构

```
map ──(karto/gmapping 或 AMCL)──> odom ──(EKF)──> base_footprint ──(URDF)──> base_link ──> laser_link
```

- 底盘反馈：`/dev/carserial` → `pubv.py` → `/vel_raw` → `base_node` → `/odom_raw`
- 融合：`/odom_raw` + `/wit/imu` → `robot_localization` EKF → `/odom`，发布 `odom -> base_footprint`
- 控制：`/cmd_vel` → `newt.py` → `/dev/carserial`
- 建图时由 karto / gmapping 发布 `map -> odom`；导航时由 AMCL 发布

## 目录结构

```
src/
├── car_bringup/                     底盘驱动（newt.py / pubv.py / base_node）、EKF、gmapping/karto launch
├── nav_demo/                        地图、AMCL/move_base launch、代价地图与 DWA 参数、巡航/记点脚本
├── photo_capture/                   拍照 / YOLO 检测 / 区域扫描 / 仪表识别
├── mbot_teleop/                     键盘遥控（发布 /cmd_vel）
├── mbot_bringup/                    底盘原始驱动、PS2 与串口模式切换脚本
├── leishen/                         lslidar 雷达驱动
├── wit/                             WIT IMU 驱动
├── OrbbecSDK_ROS/                   Orbbec 深度相机
├── robot_description/               URDF（mowen 包）
├── slam_karto/ open_karto/          Karto 建图
├── imu_tools/                       IMU 滤波插件
├── rf2o_laser_odometry/ robot_pose_ekf/   备用定位方案
├── lidar_undistortion_2d-master/    雷达去畸变（未启用）
docs/                                设计记录
```

## 地图与航点

| 内容 | 位置 |
| --- | --- |
| 地图 | `src/nav_demo/map/*.pgm` / `*.yaml`，默认 `mymap_20261106_1916_clean.yaml`（0.025m，已清理墙外杂点） |
| 保存地图 | `src/nav_demo/scripts/save_map.sh`，按时间戳命名，不覆盖旧图 |
| 巡检点位 | `src/nav_demo/waypoints/inspection_waypoints.yaml`（`waypoint_gui.py` 生成，含识别朝向 tasks） |
| 预设点位 | `src/nav_demo/config/inspection_points.yaml` |
| 路线顺序 | `src/nav_demo/config/inspection_routes.yaml` |

现成路线：`left`（左转，原 fast）、`straight`（直行）、`left_noopt` / `straight_noopt`（不平移优化对照），以及调试用的 `test_789`、`test_strafe`、`test_15`。

**换地图后点位要重记**，旧坐标对新图不成立。

## 巡航

- **图形化（当前主用）**：`rosrun nav_demo cruise_debug.py --route left`
  顶部选路线 / 填点位顺序 / 停留时间 / 平移速度 / 跳过 0 号点，底部 开始 / 暂停 / 继续 / 终止 / 记录现场 / 扫描校准，
  计划表会标出每点是 `move_base`、平移还是倒车入库，以及识别朝向。
- **命令行（早期版本）**：`rosrun nav_demo cruise_waypoints.py --route left --dry-run`，
  没有倒车入库、扫描校准、跳过 0 号点，主要用于快速看计划。
- 14→18 段用麦轮平移（只发 `linear.y`），15、17 会停车转向做识别；
- 19 号是终点，最后一段是倒车入库：先对齐 18，转到 0°（朝红绿灯），再倒进 19，对准到 2cm。

## 视觉

`photo_capture` 有四个模式：拍照、模型测试、区域扫描（横移拼图）、仪表识别。
输出统一在 `~/Pictures/photo_capture/`，模型在 `~/AI视觉/`。详见 `src/photo_capture/README.md`。

> 仪表识别**未完成**，读数还在调，暂不参与比赛流程。

## ESP32 红外测温（MLX90640）

独立测温包，不自动加入导航、不占用底盘串口；仅接入 ESP32-S3，不包含其他 MCU 工程。

```bash
catkin_make --pkg mlx90640_ros -j2
source devel/setup.bash
# 无硬件回放：需先启动 roscore
roslaunch mlx90640_ros replay.launch loop:=true
# 实机：替换为确认属于 ESP32 UART USB-C 的设备，不能照搬其他传感器端口
roslaunch mlx90640_ros mlx90640.launch port:=/dev/serial/by-id/你的ESP32设备
```

发布 32×24 摄氏度矩阵、伪彩热图和环境温度；约 2Hz 子页合成为约 1Hz 完整图。
[模块说明](<src/mlx90640_ros/README.md>) · [ESP32 接线与烧录](<firmware/mlx90640/MLX90640_ESP32S3/README.md>)。
楼层识别、巡航触发与裁判上传尚未接入。

## 已知问题 / 注意

- 手柄是底盘自带的 PS2 接收器，不经过 ROS。用过手柄后要执行
  `python3 src/mbot_bringup/scripts_can_york/ps2_serial.py` 切回串口模式，否则车会无视 ROS 指令或一卡一卡。
- 跑过 Orbbec 相机节点后 `/dev/video0` 可能消失（SDK detach 了 uvcvideo），拔插 USB 恢复；拍照脚本依赖它。
- 检测服务 `/detect_no_entry` 需要先启动对应的检测节点，并在 `conda activate torch_deploy` 环境里调用。
- 地图是 0.025m 的 karto 图，建图时别开太快，急转弯会加重影。

## 文档

- `docs/waypoint_route_plan.md`：航点 / 路线方案与暂缓原因
- `docs/waypoint_headings_review.md`：识别朝向核对
- `src/photo_capture/README.md`：拍照与检测模块说明
- 开发机上另有一份逐条命令的中文操作手册（`~/桌面/readme.md` / `readme.html`），不在仓库内
