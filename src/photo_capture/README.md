# photo_capture —— 拍照 / YOLO 检测 / 区域扫描 / 仪表识别 / 红外热像

调用本机摄像头的五个模式：

- **拍照模式**：按类型采集照片（默认 1280×720），照片存到 `输出目录/类型/`；
  可设连拍张数，运行中换类型。
- **模型测试模式**：从 `~/AI视觉` 选 YOLO `.pt`，摄像头实时画框，显示类别、置信度、推理耗时；
  可换模型、调置信度、保存带框截图。
- **区域扫描模式**：控制麦轮底盘横向平移，停一下拍一张，最后拼成横向大图；
  也可不用底盘、手动遥控平移，脚本只负责抽帧和拼接。拼接后可用 YOLO 分块检测全景。
- **仪表识别模式**：加载 Pose.pt，识别温度/湿度表的旋转轴、指针尖端、最小/最大刻度，
  按量程换算读数，保存截图和 JSONL 记录。
- **红外热像模式**：通过 `mlx90640_ros.thermal_session` 显示 MLX90640 热图，
  支持 `capture.bin` 回放、串口实机、楼层标定和 PNG+JSON 快照。

> 区域扫描目前只做**平移**场景，不做旋转全景。

## 依赖

```bash
sudo apt install python3-opencv python3-tk python3-pil.imagetk
```

检查：

```bash
python3 -c "import cv2, tkinter, PIL; print('deps ok')"
python3 -c "import ultralytics, torch; print(ultralytics.__version__, torch.__version__)"
```

模型测试 / 全景检测还需要 ultralytics；本机系统 Python 已装好。没有 GPU 时推理走 CPU，
640 尺寸大约 2–6 FPS，`--imgsz 416` 会快一些。

区域扫描的自动平移需要 ROS（`rospy`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`）和底盘驱动。
红外热像需要先 `source devel/setup.bash` 并确认 `mlx90640_ros` 已 `catkin_make`；
它走串口，不占用白光摄像头。

> OpenCV 编译时没有 GUI 后端（`GUI: NONE`），预览用 tkinter，不用 `cv2.imshow`。

## 运行

```bash
source devel/setup.bash
rosrun photo_capture capture_photos.py

# 或者直接跑脚本
python3 src/photo_capture/scripts/capture_photos.py
```

启动后先弹「选择启动模式」：点按钮或按 `1`（拍照）/ `2`（模型测试）/ `3`（区域扫描）/
`4`（仪表识别）/ `5`（红外热像）。想跳过询问可以用 `--mode photo` / `--mode detect` /
`--mode scan` / `--mode gauge` / `--mode thermal`。

## 拍照模式

选完模式后出现「拍摄设置」：填照片类型（可直接输入新类型，也可双击已有类型）、
选一次连拍张数，点「开始拍照」进入预览。

| 操作 | 说明 |
| --- | --- |
| 点「拍照」 / 空格 / 回车 | 按当前张数连拍，画面左上角会闪一下绿框 |
| 点「更换类型」 / `T` | 换照片类型，自动切到对应文件夹 |
| 改「一次连拍」数字 / `N` | 改一次拍几张（1-99） |
| 点「打开保存目录」 | 用文件管理器打开当前类型的文件夹 |
| `Esc` / 关窗口 | 退出 |

### 拍照模式命令行

没有图形界面（或 `--no-gui`）时，先选模式（`--mode photo` 可跳过）：

```
1 / 2 / 3 / 4    选择拍照 / 模型测试 / 区域扫描 / 仪表识别
直接回车 / p     拍一次（按当前张数）
t [类型]         换类型，例如：t 侧面
n [张数]         改连拍张数，例如：n 3
q                退出
```

批量采集：

```bash
printf '正面\n\n\nq\n' | python3 src/photo_capture/scripts/capture_photos.py \
  --mode photo --no-gui --count 3 --output-dir ~/Pictures/photo_capture
```

## 模型测试模式

### 启动流程

1. 启动模式选「模型测试」；
2. 在模型列表里选一个 `.pt`（默认扫描 `~/AI视觉`，也可点「浏览…」）；
3. 加载模型后进入实时检测窗口。

模型列表里有哪些文件就显示哪些；权重损坏（本机 `yolov8n.pt` 只有 0.7MB，没下载完整）
时加载会弹窗报错，换一个即可。

### 检测窗口操作

| 操作 | 说明 |
| --- | --- |
| 点「更换模型」 / `M` | 重新选模型，加载成功后立即切换 |
| 改「置信度」数字 | 0.05–0.95，改完生效 |
| 点「保存当前帧」 / `S` | 保存一张带检测框的截图 |
| 点「保存裁剪图」 / `C` | 把当前每个检测框裁成小图，按模型和类别分文件夹保存 |
| `Esc` / 关窗口 | 退出 |

截图保存到 `~/Pictures/photo_capture/模型测试/<模型名>/`；
裁剪图保存到 `~/Pictures/photo_capture/模型测试/<模型名>/裁剪/<类别>/`。

### 模型测试命令行

```bash
rosrun photo_capture capture_photos.py --mode detect --no-gui --model fire.pt
```

没有图形界面时每秒打印一行检测结果，`Ctrl+C` 退出。

## 区域扫描模式（平移拼图）

### 前提与安全

- **自动平移**需要：
  - 底盘驱动：`rosrun car_bringup newt.py`；
  - 建议同时启动 bringup，提供 `/odom` 和 `/scan`（没有 `/odom` 会退化为计时运动）；
  - 先停止遥控和导航，避免多个节点同时发布 `/cmd_vel`；
  - 场地足够宽。车宽约 30cm、道路约 60cm，**在窄走廊里不要自动平移**；
    把车放到开阔区域，或只用普通拍照模式。
- 开始前会检查 `/cmd_vel` 订阅者，检测到遥控/导航 publisher 会阻止启动（`--force` 可强制）；
- 运动前 3 秒倒计时；窗口上有「停止」按钮，`Esc`、`Ctrl+C` 也会先停车；
- 速度与巡航脚本一致（默认 0.15 m/s，上限 0.15）、短步长、每步归零，退出/异常都会发零速度；
- 有 `/scan` 时，平移前检查运动侧最近障碍，小于 `--scan-clearance` 会拒绝平移。
- **手动录制**不控制底盘，只按帧率录画面；用你自己的遥控/节点移动。
  注意：现有 `mbot_teleop` 键盘遥控的 `linear.y` 固定为 0，**不支持横移**；
  要横向采集请用自动平移模式，或自己发 `/cmd_vel.linear.y`。

### 自动平移

1. 启动模式选「区域扫描」；
2. 区域名称（可空，默认时间戳）、扫描方式选「自动平移扫描」；
3. 设总距离、张数、方向、速度、停稳时间；
4. 点「开始扫描」，等倒计时结束后车会横移、停稳、拍照，重复到设定帧数；
5. 自动拼接，显示全景缩略图。

默认参数：向左 1.0m、5 帧、每步 0.25m、速度 0.15 m/s。界面会显示实际步长。
相机默认按 1280×720 采集；如果要最大细节可加
`--width 1920 --height 1080`（1080 在全屏放大时会更柔）。

### 手动录制

1. 扫描方式选「手动录制（自己移动）」，设抽帧帧率；
2. 点「开始录制」，用你自己的遥控/节点移动（现有键盘遥控不能横移）；
3. 点「停止」（或回车），脚本按已录帧拼接。

### 拼接与结果

- 优先用 `cv2.Stitcher_SCANS`（平移扫描）；失败自动换 `PANORAMA`，再失败走
  相邻帧 ORB + RANSAC 单应兜底；
- 拼接失败不会丢帧，原始帧都在 `frames/`，可以调参数重拼；
- 勾选“保存扫掠视频”后会连续写 `sweep.avi`（默认 15fps），不是每步一张的幻灯片；
  适合后续用 YOLO 跟踪/计数，也可以 `ffmpeg -i sweep.avi -vf fps=2 frames/%04d.jpg` 抽帧；
- 拼接时每帧会缩到 `--stitch-max-width`（默认 1600）以内，控制内存。

保存结构：

```
~/Pictures/photo_capture/区域扫描/<区域名>_<时间戳>/
├── frames/
│   ├── 0001.jpg
│   └── ...
├── sweep.avi                 # 勾选“保存扫掠视频”时，连续录制（默认 15fps）
├── panorama.jpg              # 拼接成功时
├── panorama_detected.jpg     # 全景检测后
└── meta.json                 # 参数、帧、位姿、拼接/检测结果
```

### 视频识别与唯一数量（推荐用于清点电动车）

扫描时勾选「保存扫掠视频」，或命令行加 `--scan-record-video`；扫描完成后对
`sweep.avi` 做 YOLO + ByteTrack，统计**唯一 track ID**，不是把每帧检测数相加。

```bash
# 先采集（会打开扫描窗口，确认后开始）
rosrun photo_capture capture_photos.py --mode scan \
  --scan-frames 8 --scan-distance 2.0 \
  --scan-speed 0.15 --scan-feedback odom \
  --scan-record-video --scan-video-fps 15 \
  --scan-label 电动车采集_01
```

采集完成后：

```bash
python3 /home/mowen/bit_ws/src/photo_capture/scripts/count_video.py \
  --video ~/Pictures/photo_capture/区域扫描/电动车采集_01_<时间戳>/sweep.avi \
  --model /home/mowen/AI视觉/Cycle.pt \
  --min-frames 20
```

输出目录默认在视频同目录下 `识别_<时间戳>_<视频名>/`，包含：

- `count.txt`：总数和分类数量；
- `track_summary.json`：每条唯一轨迹的类别、帧数、首末帧、置信度；
- `sweep_tracked.avi`：只画最终唯一轨迹的标注视频；
- `<类别>_<ID>.jpg`：每个唯一目标的裁剪图；
- `tracks_contact.jpg`：所有唯一目标的缩略图总览。

计数调参：

- 数量偏多：提高 `--min-frames`（例如 30/40），过滤重复框和短碎片；
- 数量偏少：降低 `--min-frames` 或 `--conf`，或换更好的模型；
- `UP` / `DOWN` 按每条轨迹的类别投票，不按单帧；
- ByteTrack 需要 `lap`；本机已装，其他机器可 `pip3 install lap`（或手动装对应 Python 版本的 wheel）。

注意：**逐帧检测后相加一定会重复**；只有跟踪得到的唯一 ID 才能当数量。

### 给导航/其他脚本的接口

`count_video.py` 提供稳定的 Python 函数和 `--json` 子进程接口，导航脚本不要解析
人类可读 stdout。完整契约、字段和调用示例见
[INTEGRATION.md](INTEGRATION.md)。

```python
from count_video import count_video
result = count_video(video=".../sweep.avi", min_frames=20)
print(result.count, result.by_class)
```

```bash
python3 count_video.py --video ... --json   # stdout 只有 JSON
```

### 全景检测

拼接成功后点「检测全景」（或 CLI 加 `--detect-panorama`）：

- 用「模型测试」同一套模型列表选权重；
- 把全景切成 640 像素、20% 重叠的块，逐块 YOLO，框映射回全景坐标后按类别 NMS；
- 保存 `panorama_detected.jpg`，检测结果写进 `meta.json`；
- 检测完成后点「保存裁剪 (C)」，把每个检测框裁成小图，按
  `区域扫描/<区域>/裁剪/<模型名>/<类别>/` 分类保存；
- 没有 ultralytics 或模型加载失败时只影响检测，不影响采集和拼接。

### 区域扫描命令行

加 `--no-gui` 才走命令行；不加会打开图形界面。

```bash
# 自动平移：需要 newt.py；--yes 跳过确认
rosrun photo_capture capture_photos.py --no-gui --mode scan --scan-mode strafe \
  --scan-distance 1.0 --scan-frames 5 --scan-direction left \
  --scan-speed 0.15 --scan-label 大厅 --yes

# 拼接后顺便用 YOLO 检测全景
rosrun photo_capture capture_photos.py --no-gui --mode scan --scan-mode strafe \
  --scan-distance 1.0 --scan-frames 5 --yes \
  --detect-panorama --model fire.pt --imgsz 416

# 手动录制：不控制底盘，自己移动，按回车停止
rosrun photo_capture capture_photos.py --no-gui --mode scan --scan-mode manual

# 干跑测试（不连接 ROS、不产生真实运动）
rosrun photo_capture capture_photos.py --no-gui --mode scan --scan-mode strafe \
  --scan-dry-run --yes --scan-distance 0.1 --scan-frames 3
```

CLI 自动扫描默认会先打印摘要并等回车确认；`--yes` 跳过。

## 仪表识别模式（Pose）

针对圆形温度/湿度表：模型 `~/AI视觉/Pose.pt` 输出 4 个关键点：

- 0 = 指针旋转轴（静态）
- 1 = 指针尖端（动态）
- 2 = 量程最小值刻度（静态）
- 3 = 量程最大值刻度（静态）

类别：`temperature` / `humidity`。

### 正视化与读数

当前 Pose.pt 只有 **3 个静态点**（轴、最小、最大），所以配准只用这 3 个点做
**仿射 warp**，`tip` 绝对不参与配准：

1. 第一次识别到某类表时自动标定模板：轴映射到 512×512 中心，最小/最大刻度按
   实际角度映射到半径 190 的圆上，模板存到
   `~/Pictures/photo_capture/仪表识别/<模型名>/templates/<类别>.json`；
2. 每帧只把当前 轴/最小/最大 映射到模板三点，`cv2.warpAffine` 得到 512×512 正视图；
3. 在正视空间里用 Radon（沿射线找最暗的指针线）算指针角度，`tip` 只作为搜索先验，
   Radon 不明显更优时回退到 `tip`；
4. 按模板里的最小/最大刻度弧算比例，再按量程换算读数。

因为 `tip` 不参与 warp，指针转动不会改变正视化图像，也就不会出现“每帧模板不同导致读数飘”。

> 3 点只能做仿射，强斜视时仍有透视误差；要做到真正的 4 点单应，需要给 Pose 模型
> 再加一个**静态**关键点（logo、螺丝、固定刻字、底边中点等）。当前版本不把 tip 当第四点。

默认量程是 `0–100`（温度 °C、湿度 %RH），**请按实际表盘改**：

```bash
rosrun photo_capture capture_photos.py --mode gauge \
  --gauge-temp-min -20 --gauge-temp-max 60 \
  --gauge-humidity-min 0 --gauge-humidity-max 100
```

常见表计的刻度从最小值经顶部绕到最大值，默认用**长弧** `--gauge-arc long`；
如果表盘是短弧，换 `short`，拿不准用 `auto`。指针角度默认 `--gauge-pointer radon`，
也可以切 `tip` 直接使用模型尖端。Radon 假设指针比表盘暗；如果指针是亮色/强反光，
优先用 `tip` 或调整光照。

### 界面操作

| 操作 | 说明 |
| --- | --- |
| 点「更换模型」 / `M` | 换一个 `.pt`（必须是 pose 任务） |
| 点「重新标定」 / `T` | 用当前检测覆盖该类别的正视化模板 |
| 点「正视化」 / `V` | 查看 512×512 正视化图像 |
| 点「保存当前帧」 / `S` | 保存带关键点和读数的截图 |
| 点「保存读数」 / `R` | 把当前读数追加到 `readings.jsonl` |
| 点「打开记录目录」 | 打开当前模型的记录目录 |
| `Esc` / 关窗口 | 退出 |

第一次使用建议让表盘正对相机，等识别稳定后按 `T` 标定一次；模板不对时也按 `T` 覆盖。
截图和读数保存到：

```
~/Pictures/photo_capture/仪表识别/<模型名>/
├── snapshots/<模型名>_YYYYmmdd_HHMMSS_mmm.jpg
├── templates/<类别>.json
└── readings.jsonl
```

`readings.jsonl` 每行是一条记录，包含时间、模型、类别、读数、量程比例、指针角度、
目标置信度、关键点坐标、关键点置信度和模板名。

### 命令行

```bash
rosrun photo_capture capture_photos.py --no-gui --mode gauge \
  --pose-model ~/AI视觉/Pose.pt --gauge-conf 0.5 --gauge-kpt-conf 0.3

# 指定量程、弧和指针算法
rosrun photo_capture capture_photos.py --no-gui --mode gauge \
  --gauge-temp-min 0 --gauge-temp-max 50 --gauge-humidity-min 20 --gauge-humidity-max 90 \
  --gauge-arc long --gauge-pointer radon
```

终端每秒打印一次，`Ctrl+C` 退出。没有图形界面时只打印，不保存截图/JSONL；模板仍会
自动标定到输出目录下。

> 模型必须是 pose 任务；选错普通检测模型会提示“所选模型不是 pose 模型”。
> 关键点置信度低于 `--gauge-kpt-conf`、或关键点挤成一团时不会算读数，界面/终端会显示原因。

## 红外热像模式（MLX90640）

热像只回答“画面里哪个像素最热、对应哪一层”，不提供世界坐标；它走 ESP32-S3 串口，
和白光摄像头互不影响。**一个进程只能有一个 `ThermalSession`**，也不要同时开
`roslaunch mlx90640_ros` 抢同一个串口。

### 无硬件回放

```bash
rosrun photo_capture capture_photos.py --mode thermal \
  --capture src/mlx90640_ros/test/data/capture.bin
```

`--capture` 是回放抓包，`loop=True` 会循环；适合先调界面和验证出图，不需要接热像硬件。

### 实机串口

先确认设备身份：

```bash
ls -l /dev/serial/by-id/
rosrun photo_capture capture_photos.py --mode thermal \
  --thermal-port /dev/thermal
```

也可以用界面里的「回放抓包…」和「连接串口」按钮切换数据源。

### 楼层标定：每层记录 2~3 次取中位数

热像没有世界坐标，只有 24×32 温度矩阵的**图像行号**（row 0 是图像顶部）。
标定流程：

1. 车停 12 号点，暖宝宝一次只贴一层；
2. 等 1~2 秒，看着热像峰值落在目标层；
3. 点「记录 1F / 2F / 3F」；同一层可以换几个停车位置**重复记录 2~3 次**；
4. 界面会对同一层的记录取 **row 中位数**，再自动按相邻楼层中心的中点划分区间；
5. 最后看楼层表，例如四层 `4F:0-5,3F:6-11,2F:12-17,1F:18-23`，点「应用楼层表」也可以手动改。
   默认表只是四层等分占位，**未到 12 号点标定前不要把它当准确楼层**。

运行时的判层用**峰值行**（`frame.hotspot.row`），不是热点框中心，也不是世界坐标。
所以“多拍几张”是为了对同一层多取几个停车样本，不是把多张图相加。

默认预览是**双画面**：左边 MLX90640 热像，右边白光 RGB；也可以在侧栏切「热像」或「白光」。
RGB 用普通摄像头参数（`--device`，默认 1280×720）；白光打不开时热像仍可继续用。

### 界面操作

| 操作 | 说明 |
| --- | --- |
| 「双画面 / 热像 / 白光」 | 切换预览；默认双画面 |
| 点「保存快照」 / `S` | 保存热像 PNG + 同名 JSON |
| 「连接串口」 | 用输入框里的 `/dev/serial/by-id/...` 重建 session |
| 「回放抓包…」 | 选择 `capture.bin` 做无硬件回放 |
| 「应用楼层表」 | 写入 `4F:0-5,3F:6-11,2F:12-17,1F:18-23` 这类映射 |
| 「记录 1F/2F/3F/4F」 | 用当前帧峰值行记录楼层中心；同一层可多次记录取中位数 |
| 热点框 Δ℃ + 应用 | 调整热点区域阈值，默认 2.0℃ |
| `Esc` / 关窗口 | `session.stop()` 并退出 |

当前读数会显示峰值温度、峰值行列、楼层和环境温度。保存位置：

```
~/Pictures/photo_capture/红外热像/thermal_YYYYmmdd_HHMMSS_mmm.png
~/Pictures/photo_capture/红外热像/thermal_YYYYmmdd_HHMMSS_mmm.json
```

热像装反时楼层表要整体倒过来：`row=0` 是图像顶部。判层用峰值行，不要用热点框中心。
`stop()` 后不要复用同一个 `ThermalSession` 对象；界面切换数据源时会先 stop 再新建。

### 命令行回放

```bash
rosrun photo_capture capture_photos.py --no-gui --mode thermal \
  --capture src/mlx90640_ros/test/data/capture.bin
```

终端按 1 Hz 左右打印峰值、行列、楼层和环境温度，`Ctrl+C` 退出。

### 注意

- 不要修改 `mlx90640_ros` 内部；GUI 只调用 `ThermalSession` / `FloorTable` 公开接口；
- 热像和 `roslaunch mlx90640_ros` 二选一；
- 串口不能用 `/dev/ttyUSB0` 猜，先看 `/dev/serial/by-id/`。

## 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--mode` | `ask` | `ask` 每次询问 / `photo` / `detect` / `scan` / `gauge` / `thermal` |
| `-d, --output-dir` | `~/Pictures/photo_capture` | 照片、模型截图、区域扫描、仪表记录的根目录 |
| `-t, --type` | 无（启动时询问） | 预填照片类型 |
| `-c, --count` | `1` | 一次连拍张数（1-99） |
| `--device` | `0` | 摄像头编号 / `/dev/video0` / 视频文件（测试用） |
| `--model` | 无 | YOLO 权重路径或文件名 |
| `--model-dir` | `~/AI视觉` | 模型目录 |
| `--conf` | `0.25` | 检测置信度阈值 0.01–0.99 |
| `--thermal-capture` / `--capture` | 无 | MLX90640 `capture.bin` 回放 |
| `--thermal-port` | `/dev/thermal` | MLX90640 专属串口；默认 `/dev/thermal` |
| `--thermal-scale` | `12` | 热像伪彩放大倍数 |
| `--thermal-delta` | `2.0` | 热点框温度差 ℃ |
| `--thermal-floor-table` | `4F:0-5,3F:6-11,2F:12-17,1F:18-23` | 初始四层等分表（占位，现场必须重录） |
| `--thermal-replay-rate` | `2.0` | 回放帧率 Hz |
| `--imgsz` | `640` | 推理输入尺寸，CPU 慢可用 `416` |
| `--infer-device` | `auto` | `auto` / `cpu` / `cuda` |
| `--width / --height` | `1280 / 720` | 采集分辨率；0 表示用摄像头默认值；≥1280 时自动用 MJPG |
| `--sharpness` | `3` | 摄像头锐度 0-6；默认相机原值，6 更硬但噪点/光晕更明显 |
| `--pixel-format` | `auto` | `auto` / `mjpg` / `yuyv`；yuyv 未压缩但帧率低 |
| `--fps` | 摄像头默认 | 采集帧率 |
| `--format` | `jpg` | `jpg` / `png` |
| `--quality` | `95` | JPEG 质量 1-100 |
| `--interval` | `0.4` | 连拍间隔（秒） |
| `--mirror` | 关 | 水平镜像 |
| `--no-gui, --headless` | 关 | 不打开窗口，用命令行交互 |
| `--scan-mode` | `strafe` | `strafe` 自动平移 / `manual` 手动录制（不控制底盘） |
| `--scan-frames` | `5` | 平移扫描总帧数 |
| `--scan-distance` | `1.0` | 平移总距离（米） |
| `--scan-direction` | `left` | `left` / `right` |
| `--scan-speed` | `0.15` | 平移速度 m/s（上限 0.15，与巡航脚本一致） |
| `--scan-settle` | `0.5` | 每步停稳时间（秒） |
| `--scan-fps` | `3.0` | 手动录制抽帧帧率 |
| `--scan-label` | 时间戳 | 区域名称 |
| `--scan-feedback` | `auto` | `auto` / `odom` / `time` |
| `--scan-clearance` | `0.25` | 平移前雷达净空（米），0 关闭 |
| `--stitch-max-width` | `1600` | 参与拼接的单帧最大宽度 |
| `--scan-record-video` | 关 | 连续保存 `sweep.avi`（默认 15fps，用于跟踪/抽帧） |
| `--scan-video-fps` | `15` | `sweep.avi` 帧率（5~30） |
| `--no-stitch` | 关 | 只采集不拼接 |
| `--scan-dry-run` | 关 | 干跑，不连接 ROS、不产生真实运动 |
| `--detect-panorama` | 关 | 拼接后用 YOLO 分块检测 |
| `--yes` | 关 | CLI 自动扫描免确认 |
| `--pose-model` | `~/AI视觉/Pose.pt` | 仪表 Pose 权重 |
| `--gauge-conf` | `0.5` | 仪表目标置信度 |
| `--gauge-kpt-conf` | `0.3` | 4 个关键点的最低置信度 |
| `--gauge-arc` | `long` | `long` / `short` / `auto` 刻度弧 |
| `--gauge-pointer` | `radon` | `radon` 正视化后射线搜索 / `tip` 模型尖端 |
| `--gauge-temp-min` | `0` | 温度量程下限 |
| `--gauge-temp-max` | `100` | 温度量程上限 |
| `--gauge-humidity-min` | `0` | 湿度量程下限 |
| `--gauge-humidity-max` | `100` | 湿度量程上限 |
| `--force` | 关 | 忽略 `/cmd_vel` 上其他发布者的冲突 |

## 拍照分辨率与模糊

- 默认按 **1280×720** 打开摄像头，并强制 **MJPG**：720p 通常比 1080p
  每像素更锐，文件也更小；1080p 的 YUYV 只有约 5fps，MJPG 才有 30fps。
- 想保留更多细节/裁剪空间可以显式用 `--width 1920 --height 1080`，
  但在 100% 放大看时通常比 720p 更柔。
- 摄像头不支持 720p 时会退回实际支持的分辨率，终端摘要里的“摄像头画面”
  会打印实际尺寸；也可以用 `--width 0 --height 0` 用驱动默认值。
- 默认锐度用 3（相机原值，噪点较少）；`--sharpness 6` 边缘更硬，但会放大噪点，
  对比主机时注意两边锐度是否一致。退出时会恢复原来的锐度。
- 如果同一相机在别的电脑上更清楚，先确认那边不是默认 640×480；qv4l2 默认
  常见就是 640×480 MJPG，窗口又小，看起来会锐很多。要和主机对齐 640×480：
  `--width 640 --height 480 --pixel-format mjpg --fps 25`。
- 再对比 `--pixel-format yuyv`（720p 约 10fps、1080p 约 5fps，无 MJPEG 压缩），
  以及直连 USB 口/带供电的 USB Hub（本机相机原来挂在 Hub 上）。
- 如果 720p + 锐度 3 下仍然觉得糊，先排除对焦/光照/运动：这类小相机通常是固定对焦，
  弱光下噪点和长曝光会糊；拍照时保持静止、补光，比继续加分辨率更有效。
  可用 `--sharpness 6` 对比，或 `--sharpness -1` 完全不动摄像头设置。

## 保存规则

拍照：

```
<输出目录>/<类型>/<类型>_YYYYmmdd_HHMMSS_mmm_序号.jpg
```

模型截图：

```
<输出目录>/模型测试/<模型名>/<模型名>_YYYYmmdd_HHMMSS_mmm.jpg
```

YOLO 裁剪图（按模型 / 类别）：

```
<输出目录>/模型测试/<模型名>/裁剪/<类别>/<模型名>_YYYYmmdd_HHMMSS_mmm_序号.jpg
<输出目录>/区域扫描/<区域名>_<时间戳>/裁剪/<模型名>/<类别>/<模型名>_..._序号.jpg
```

每次保存还会在对应模型目录下写一份 `manifest_<时间戳>.json`，记录类别、置信度和框坐标。

区域扫描：

```
<输出目录>/区域扫描/<区域名>_<时间戳>/{frames,panorama.jpg,panorama_detected.jpg,meta.json}
```

仪表识别：

```
<输出目录>/仪表识别/<模型名>/snapshots/*.jpg
<输出目录>/仪表识别/<模型名>/templates/<类别>.json
<输出目录>/仪表识别/<模型名>/readings.jsonl
```

红外热像：

```
<输出目录>/红外热像/thermal_YYYYmmdd_HHMMSS_mmm.png
<输出目录>/红外热像/thermal_YYYYmmdd_HHMMSS_mmm.json
```

名字里的 `/ \ : * ? " < > |` 会换成 `_`；文件名带毫秒时间戳，不覆盖旧图。

## 常见问题

- **打不开摄像头**：确认 `ls /dev/video*` 有设备；换编号试 `--device 1`；
  确认没有别的程序占用。Orbbec 节点跑完后 `/dev/video0` 可能消失（SDK detach uvcvideo），
  拔插相机 USB 即可恢复。
- **区域扫描提示没有 /cmd_vel 订阅者**：先 `rosrun car_bringup newt.py`。
- **区域扫描提示有其他 /cmd_vel 发布者**：关掉遥控/导航，或确认安全后用 `--force`。
- **没有 /odom**：只跑 newt.py 时没有里程计，脚本会退化为计时平移，步长精度靠重叠容忍；
  想闭环请启动 bringup。
- **平移被拒绝**：雷达检测到运动侧障碍小于净空阈值；清理场地或调小 `--scan-clearance`。
- **拼接失败**：帧太少、重叠不足、纹理太少或运动模糊都可能导致失败；原始帧都保留，
  可以增加帧数/减小步长重拍，或直接用帧。
- **模型测试/全景检测报没有 ultralytics**：`pip3 install ultralytics`，并确认用的是同一个 Python。
- **仪表识别报“不是 pose 模型”**：要用 Pose.pt 这类姿态权重，不能选普通检测模型。
- **仪表读数不对**：先确认量程（`--gauge-temp-min/max`、`--gauge-humidity-min/max`）和
  刻度弧（`long/short/auto`）；默认长弧适合大多数圆形表。第一次模板标定最好正对表盘，
  模板不好时按 `T` 重标；强斜视需要给模型加静态第 5 点后做 4 点单应。
- **CPU 上很慢**：`--imgsz 416`，或关掉其他占 CPU 的程序；有 GPU 用 `--infer-device cuda`。
- **拍照还是糊**：确认终端打印的“摄像头画面”是 1280×720，且锐度默认 3；弱光/固定对焦
  （多数小相机）/运动仍会造成模糊，保持静止、补光，或试 `--sharpness 6`、1080p 看细节。
- **中文类别显示成方块**：脚本自动找 Noto CJK / 文泉驿字体；找不到时只画框、不画文字。
