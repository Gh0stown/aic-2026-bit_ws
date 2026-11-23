# 电动车连续视频识别 README

本文记录“区域扫描拍连续视频 → YOLO 检测 → ByteTrack 跟踪 → 唯一 ID 计数”的完整技术栈和操作流程。

> 结论：这套方案用于**清点区域内的电动车数量**，不是拼接全景。对 3D 场景来说，逐帧检测后相加一定会重复，必须用视频跟踪后的唯一 track ID 计数。

## 1. 目标

- 控制麦轮底盘横向扫过目标区域；
- 连续录制一段视频，覆盖区域内所有电动车；
- 用 YOLO 检测电动车（立着/倒地）；
- 用多目标跟踪把同一辆车在多帧中的检测合并成一个 ID；
- 输出唯一数量、分类数量、标注视频和每辆车的裁剪图。

## 2. 技术栈

| 层 | 技术 | 说明 |
|---|---|---|
| 底盘 | ROS1 Noetic + 麦轮 | `/cmd_vel.linear.y` 横移，巡航速度 0.15 m/s |
| 定位/反馈 | `/odom`（EKF）、`/scan` | 平移距离闭环和安全净空检查 |
| 采集 | Python + OpenCV UVC | 1280×720 MJPG，默认 15fps |
| 拍摄 GUI | Tkinter + Pillow | 区域扫描设置、预览、停止按钮 |
| 视频 | MJPG AVI | `sweep.avi`，连续录制，不是幻灯片 |
| 检测 | Ultralytics YOLO | `~/AI视觉/Cycle.pt`，类别 `UP` / `DOWN` |
| 跟踪 | ByteTrack | 依赖 `lap`，逐帧 `persist=True` |
| 计数 | 唯一 track ID | 过滤短轨迹 + 类别多数投票 |
| 图像处理 | OpenCV / numpy | 裁剪、标注、视频写入 |
| 结果接口 | Python + JSON | `count_video()` / `count_video.py --json` |

## 3. 数据流

```text
区域扫描设置
  → 麦轮按 0.15 m/s 横移
  → 每步停稳拍关键帧（frames/）
  → 同时连续写 sweep.avi（15fps）
  → YOLO + ByteTrack 逐帧跟踪
  → 过滤 < min_frames 的短轨迹
  → 每条轨迹按类别多数投票
  → 唯一 ID 计数
  → count.txt / track_summary.json / sweep_tracked.avi / 裁剪图
```

## 4. 采集

### 4.1 启动底盘

```bash
rosrun car_bringup newt.py
roslaunch /home/mowen/bit_ws/src/car_bringup/launch/bringup.launch
```

`newt.py` 提供 `/cmd_vel` 订阅；`bringup.launch` 提供 `/scan`、`/odom`。如果已经启动导航/建图，不要重复启动。

### 4.2 扫掠拍摄

```bash
rosrun photo_capture capture_photos.py --mode scan \
  --scan-frames 8 --scan-distance 2.0 \
  --scan-speed 0.15 --scan-feedback odom \
  --scan-record-video --scan-video-fps 15 \
  --scan-label 电动车采集_01
```

窗口里确认：

- 自动平移；
- 8 帧、2.0m、0.15m/s；
- 勾选「保存扫掠视频 sweep.avi」。

输出：

```text
~/Pictures/photo_capture/区域扫描/电动车采集_01_<时间戳>/
├── frames/          # 每步关键帧
├── sweep.avi        # 连续视频，识别的真正输入
├── panorama.jpg     # 拼接预览，3D 场景可能有重影
└── meta.json        # 参数、帧、位姿、拼接结果
```

没有图形界面时加 `--no-gui --yes`，但会立即开始底盘运动，必须先确认场地清空。

## 5. 识别与计数

### 5.1 命令行

```bash
python3 /home/mowen/bit_ws/src/photo_capture/scripts/count_video.py \
  --video ~/Pictures/photo_capture/区域扫描/电动车采集_01_<时间戳>/sweep.avi \
  --model /home/mowen/AI视觉/Cycle.pt \
  --min-frames 20
```

### 5.2 Python 调用

```python
import sys
sys.path.insert(0, "/home/mowen/bit_ws/src/photo_capture/scripts")

from count_video import count_video

result = count_video(
    video=".../sweep.avi",
    model="/home/mowen/AI视觉/Cycle.pt",
    min_frames=20,
)
print(result.count)
print(result.by_class)   # {"UP": 3, "DOWN": 2}
```

### 5.3 JSON 接口

```bash
python3 /home/mowen/bit_ws/src/photo_capture/scripts/count_video.py \
  --video .../sweep.avi --json
```

stdout 只有一行 JSON，适合导航脚本或子进程解析。完整字段见
[INTEGRATION.md](../src/photo_capture/INTEGRATION.md)。

## 6. 输出

默认在视频同目录下生成：

```text
识别_<时间戳>_sweep/
├── count.txt              # 总数 + 分类数
├── track_summary.json     # 每条唯一轨迹的类别/帧数/首末帧/置信度
├── sweep_tracked.avi      # 只画最终唯一轨迹
├── <类别>_<ID>.jpg        # 每辆车的裁剪图
└── tracks_contact.jpg     # 所有唯一目标的缩略图总览
```

## 7. 为什么必须跟踪

一辆电动车会出现在多帧里。逐帧检测后求和，同一辆车会被算多次。

当前做法：

1. ByteTrack 给每个目标分配 track ID；
2. 少于 `--min-frames` 帧的轨迹视为重复框/误检/碎片，丢弃；
3. 每条轨迹按类别投票，避免单帧 `UP/DOWN` 抖动；
4. 统计剩余唯一 track ID 数量。

默认 `--min-frames 20`，15fps 下约 1.3 秒。

## 8. 已测结果

01~04 四次采集，总数均为 **5**：

| 测试 | 视频帧数 | 总数 | 分类 |
|---|---:|---:|---|
| 采集_01 | 198 | 5 | UP 4 / DOWN 1 |
| 采集_02 | 199 | 5 | UP 3 / DOWN 2 |
| 采集_03 | 139 | 5 | DOWN 5 |
| 采集_04 | 144 | 5 | UP 5 |

其中某次“最佳单帧”只看到 4 辆车，但整段视频跟踪补出了第 5 辆，说明**单帧计数不可靠**。

## 9. 为什么放弃“拼接全景后检测”

最初方案是让车横移、每步停稳拍 8 帧，先拼成一张横向全景，再在全景上跑 YOLO 数框。
这个方案能生成一张好看的预览图，但不能稳定得到唯一数量，主要坑如下。

### 9.1 3D 视差：全景不是“俯视地图”

- 场地里的电动车、墙、桌面不在同一深度；相机横移时近处物体和远处背景的位移不同；
- 单应矩阵只能对齐一个平面。拼出来的全景在接缝处会出现“同一个车头/车尾错位”，
  或者同一辆车被拉成两部分；
- 全景看起来完整，不代表几何一致；用它做计数相当于在一个有重影的 2D 图上数 3D 物体。

### 9.2 同一辆车出现在接缝两侧

- 每帧重叠区域里的车会被拼进两次；检测器在左边和右边各框一次；
- 没有 track ID、没有时间信息，无法判断“这两个框是不是同一辆车”；
- 直接对全景检测框求和，会系统性多算；靠 NMS/位置聚类也只能缓解，不能从根本上区分。

### 9.3 低纹理和运动导致拼接失败

- 黑色背景墙、灰黑地胶纹理弱，ORB/特征匹配容易失败；实测关键帧 07/08 基本只剩地面，
  全景会出现黑边、错位、局部拉伸；
- 扫描过程中曝光变化、车辆/人移动，接缝处有鬼影；
- 拼接失败时程序不一定报错，只是生成一张“看起来能看”的图，后续检测结果更不可信。

### 9.4 全景缩放后小目标变糊

- 全景宽度约 1489px 时，单辆电动车只占几十像素；YOLO 再缩到 640/1600，
  小目标、倒地车、遮挡车容易漏；
- 分块检测又会在 tile 边界重复框选，需要额外去重，仍然没有时间身份。

### 9.5 单帧/单张全景的漏检无法补救

- 某次“最佳单帧”只看到 4 辆，人工真值是 5 辆；全景是静态图，缺的那辆在别的时刻/角度，
  拼接后也不一定出现；
- 连续视频可以利用前后帧补出被遮挡/只在一小段出现的车；静态全景没有这个信息。

### 9.6 结论

拼接全景适合：

- 现场给裁判/队友看“大概长什么样”；
- 调试相机、检查覆盖范围；
- 作为采集质量的人工参考。

不适合：

- 作为电动车**唯一数量**的输入；
- 用“全景检测框数量”直接上报。

最终改为：连续录 `sweep.avi` → YOLO 逐帧检测 → ByteTrack 唯一 ID → 过滤短轨迹 →
类别投票 → 唯一数量。01~04 四段视频均为 5 辆；其中单帧最佳只有 4 辆，视频跟踪补出第 5 辆。

## 10. 调参

| 参数 | 默认 | 作用 |
|---|---:|---|
| `--conf` | 0.25 | YOLO 置信度；漏检多可降到 0.20 |
| `--imgsz` | 640 | 推理尺寸；CPU 慢可降 416 |
| `--min-frames` | 20 | 轨迹最少帧数；数量偏多调大，偏少调小 |
| `--classes` | 全部 | 只统计 `UP,DOWN` 等指定类别 |
| `--no-video` | 关 | 不写标注视频，只出数量/JSON/裁剪 |

常用排查：

- 数量偏多：提高 `--min-frames` 到 30/40，检查重复框；
- 数量偏少：降低 `--min-frames` 到 10/15，或降低 `--conf`；
- 同一辆车被拆成两个 ID：相似黄色电动车容易 ID 跳变，需要提高跟踪稳定性或做位置聚类；
- 不要用拼接全景来计数：3D 场景前后景视差会导致重影/漏车。

## 11. 依赖

```bash
# Python 包
ultralytics
lap          # ByteTrack 必需
opencv-python
numpy
```

本机 `lap` 已安装；换机器如果报 `No module named 'lap'`，需要安装对应 Python 3.8 的 `lap`。

采集侧还需要 ROS1 Noetic、`rospy`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`。

## 12. 文件位置

| 文件 | 作用 |
|---|---|
| `src/photo_capture/scripts/capture_photos.py` | 拍照/扫描 GUI，录制 `sweep.avi` |
| `src/photo_capture/scripts/count_video.py` | YOLO + ByteTrack 唯一计数 |
| `src/photo_capture/INTEGRATION.md` | 导航脚本接口契约 |
| `src/car_bringup/launch/bringup.launch` | `/scan` + `/odom` 最小启动 |

## 13. 导航接入流程

```text
导航到目标点 → 停车/暂停导航
  → 启动 newt + bringup
  → 区域扫描录制 sweep.avi
  → count_video 取唯一数量
  → 恢复导航
```

注意：区域扫描控制 `/cmd_vel`，不能和导航同时抢底盘。详细接口见
[INTEGRATION.md](../src/photo_capture/INTEGRATION.md)。
