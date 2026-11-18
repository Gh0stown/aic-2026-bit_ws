# photo_capture —— 拍照 / YOLO 检测 / 区域扫描 / 仪表识别 / 红外热像 / 文字识别

调用本机摄像头的六个模式：

- **拍照模式**：按类型采集照片（默认 1280×720），照片存到 `输出目录/类型/`；
  可设连拍张数，运行中换类型。
- **模型测试模式**：从 `~/AI视觉` 选 YOLO `.pt`，摄像头实时画框，显示类别、置信度、推理耗时；
  可换模型、调置信度、保存带框截图。
- **区域扫描模式**：控制麦轮底盘横向平移，停一下拍一张，最后拼成横向大图；
  也可不用底盘、手动遥控平移，脚本只负责抽帧和拼接。拼接后可用 YOLO 分块检测全景。
- **仪表识别模式**：加载 gauge.pt，识别温度/湿度表的轴、尖端和 5 个刻度点，
  按多刻度插值读数，保存截图和 JSONL 记录。
- **红外热像模式**：通过 `mlx90640_ros.thermal_session` 显示 MLX90640 热图，
  支持 `capture.bin` 回放、串口实机、楼层标定和 PNG+JSON 快照。
- **文字识别模式**：同一个 OCR 分区里提供两个页签——
  PaddleOCR 中文文字识别，和 HyperLPR3 车牌识别；识别结果可保存 JPG+JSON。

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

文字识别额外依赖 PaddleOCR（中文）和 HyperLPR3 + onnxruntime（车牌）：

```bash
python3 -c "import paddleocr, hyperlpr3, onnxruntime; print('ocr deps ok')"
```

本机已安装 PaddleOCR 2.6.1 / paddle 2.6.2、HyperLPR3 0.1.3 / onnxruntime 1.16.3，
PaddleOCR 模型在 `~/.paddleocr/whl`，HyperLPR3 模型在 `~/.hyperlpr3/20230229`。
换机器时按 `requirements-ocr.txt` 安装；首次识别如果模型缺失会自动下载。
中文裁剪模型复用 ultralytics，已随主项目安装。

> OpenCV 编译时没有 GUI 后端（`GUI: NONE`），预览用 tkinter，不用 `cv2.imshow`。

## 运行

```bash
source devel/setup.bash
rosrun photo_capture capture_photos.py

# 或者直接跑脚本
python3 src/photo_capture/scripts/capture_photos.py
```

启动后先弹「选择启动模式」：点按钮或按 `1`（拍照）/ `2`（模型测试）/ `3`（区域扫描）/
`4`（仪表识别）/ `5`（红外热像）/ `6`（文字识别）。想跳过询问可以用 `--mode photo` /
`--mode detect` / `--mode scan` / `--mode gauge` / `--mode thermal` / `--mode ocr`。

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

## 仪表识别模式（gauge.pt）

默认模型：`~/AI视觉/gauge.pt`，`task=pose`，`nc=2`，`kpt_shape=[7,3]`。

| 类别 | 关键点（0~6） | 量程 |
| --- | --- | --- |
| `temperature` | axis, tip, min_value, scale_-10, scale_10, scale_30, max_value | -30 ~ 50 °C |
| `humidity` | axis, tip, min_value, scale_20, scale_40, scale_70, max_value | 0 ~ 100 %RH |

- 2~6 的 **5 个刻度点** 是配准锚点；
- 0 是指针轴心，虽然静态，但**不是刻度圆心**，不参与配准；
- 1 是指针尖端，动态，不参与配准；
- 5 个刻度点已经足够做 **4 点单应**（`cv2.findHomography`），并保留轴心相对刻度圆心的真实偏移。

### 正视化与读数

1. 使用 deli 9011 的固定刻度角模板（湿度 0/20/40/70/100 的实测角，温度用通用
   270° 模板），**不需要现场正对标定**；模型在斜视画面里检测到 5 个刻度点即可；
2. 每帧只用 **5 个刻度点**做单应 `warpPerspective` 到 512×512；轴心不参与配准，
   所以轴心偏离刻度圆心时，偏移会被保留下来；
3. 正视空间里，把「轴心 → 指针尖端」的射线与刻度圆求正向交点 Q；
4. 用 Q 相对**刻度圆心**的角度，在 5 个刻度锚点之间做**分段线性插值**：
   - 温度：-30, -10, 10, 30, 50；
   - 湿度：0, 20, 40, 70, 100。

注意：不是把刻度点绕轴心算角度，也不是把轴心当刻度圆心。

例如指针落在 10 和 30 之间，就按 10~30 这一段插值；比只按 -30~50 线性换算更贴近实际表盘。

现在不需要先正对表盘按 `T`；固定模板直接从斜视帧做单应。`T` 只是重新加载固定模板；
按 `V` 可以查看当前 512×512 正视化图像。换用不同表型时，需要改
`gauge.py` 里的 `CANONICAL_TICK_ANGLES`（或后续增加自动模板模式）。

默认量程已写入代码：温度 `-30~50`，湿度 `0~100`；需要改时：

```bash
rosrun photo_capture capture_photos.py --mode gauge \
  --gauge-temp-min -30 --gauge-temp-max 50 \
  --gauge-humidity-min 0 --gauge-humidity-max 100
```

常见表计的刻度从最小值经顶部绕到最大值，默认用**长弧** `--gauge-arc long`；
如果表盘是短弧，换 `short`，拿不准用 `auto`。指针角度默认 `--gauge-pointer tip`
（模型尖端），在合成数据上误差最小；`radon` 作为可选的图像细化，亮指针/强反光时不要用。

### 模拟数据验证

用 40 张精确几何合成表盘（temperature/humidity 各 20 张，含 0/5/10/15% 偏心、
roll ±15°、tilt ±10/20°、640/480 与 1280/720）做离线验证：

- 固定刻度角模板，不使用任何现场标定；每张图只使用 7 个关键点（不使用 manifest 的 H/dial_center/value）；
- 指针取模型尖端 `tip`，最大绝对误差 **3.5e-6**，40/40 通过；
- 旧「绕轴心求刻度角」方法最大误差约 0.55（湿度 0.55% FS），说明偏心不能忽略；
- 旧「0~100 线性换算」在湿度上最大误差约 8.33，说明非线性刻度必须按 5 个锚点插值。

> 合成数据只验证几何，不代表真实照片的检测/反光/遮挡精度；真实表仍需现场用 `T` 标定。

### 界面操作

| 操作 | 说明 |
| --- | --- |
| 点「更换模型」 / `M` | 换一个 `.pt`（必须是 pose 任务） |
| 点「重载模板」 / `T` | 重新加载固定刻度角模板（通常不需要按） |
| 点「正视化」 / `V` | 查看 512×512 正视化图像 |
| 点「保存当前帧」 / `S` | 保存带 7 个关键点和读数的截图 |
| 点「保存读数」 / `R` | 把当前读数追加到 `readings.jsonl` |
| 点「打开记录目录」 | 打开当前模型的记录目录 |
| `Esc` / 关窗口 | 退出 |

保存位置：

```
~/Pictures/photo_capture/仪表识别/<模型名>/
├── snapshots/<模型名>_YYYYmmdd_HHMMSS_mmm.jpg
├── templates/<类别>.json   # 仅旧/自动模板模式使用
└── readings.jsonl
```

`readings.jsonl` 包含时间、类别、读数、单位、归一化比例、指针角度、目标置信度、
7 个关键点坐标、关键点置信度和模板名。

### 命令行

```bash
rosrun photo_capture capture_photos.py --no-gui --mode gauge \
  --pose-model ~/AI视觉/gauge.pt --gauge-conf 0.5 --gauge-kpt-conf 0.3

# 指定量程、弧和指针算法
rosrun photo_capture capture_photos.py --no-gui --mode gauge \
  --gauge-temp-min -30 --gauge-temp-max 50 \
  --gauge-humidity-min 0 --gauge-humidity-max 100 \
  --gauge-arc long --gauge-pointer tip
```

终端每秒打印一次，`Ctrl+C` 退出。没有图形界面时只打印，不保存截图/JSONL；
7 点模型使用固定刻度角模板，不需要正对标定。

> 模型必须是 pose 任务；选错普通检测模型会提示“所选模型不是 pose 模型”。
> 关键点置信度低于 `--gauge-kpt-conf`、或关键点挤成一团时不会算读数。

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
| 勾选「同时保存 IR+RGB 拼图」 | 快照时额外保存一张 `thermal_..._rgb.png`（默认关） |
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
~/Pictures/photo_capture/红外热像/thermal_YYYYmmdd_HHMMSS_mmm_rgb.png  # 勾选拼图时
```

热像装反时楼层表要整体倒过来：`row=0` 是图像顶部。判层用峰值行，不要用热点框中心。
`stop()` 后不要复用同一个 `ThermalSession` 对象；界面切换数据源时会先 stop 再新建。

### IR → RGB 平面标定窗口

红外热像主窗点击 **「标定 IR → RGB…」**。子窗复用主窗已经取得的 IR/RGB 帧和预览控件，不新开相机、串口或解码器；这个窗口只负责 H，原 IR 行号楼层表不变。RGB 窗户几何匹配另见下节。

1. 车和楼宇静止，热源贴楼宇正面同一平面。默认显示 ROI 内自动热点，也可点击 IR 手动选点；RGB 点击同一个热源中心。第一次点击会冻结这一对画面，避免两个点来自不同时间。
2. 点击「采集当前点」，然后「下一点 / 实时」，移动热源继续采集。至少 4 对，推荐 6～9 对分散覆盖上下左右；不要重复或全共线。
3. 「计算 H」显示重投影 RMS；目标低于 3px，超限会警告，不冒充现场验收。「验证」用黄圈标出 IR 点在 RGB 中的预测落点；主窗也会显示当前热点投影。
4. 「保存」写入输出根目录下的 `红外热像/calibration.json`（默认在 `~/Pictures/photo_capture/`）。新启动等 IR/RGB 有效后自动加载；也可在子窗手动加载。尺寸或 IR 镜像不匹配、缺 H、点数不足、退化几何等会报错拒用。删除/清空/新增点使当前 H 失效，须重新计算。
5. ROI 输入 `r0,c0,r1,c1`，端点半开，空为全图。例如排除楼顶排插时，仅保留实测楼宇范围，勿照搬示例坐标。应用后先停止原会话再重建，保留数据源、色标、楼层表等参数，清除待选旧帧；ROI 在当前 GUI 会话中保留，重启后需重新填写。标定窗根据 `frame.hotspot` 标点，不使用原伪彩图中全图最高温的旧十字。

点击表格可回看已采点和冻结图；从 JSON 加载的点不含历史图像/温度，回看时明确标示为当前图上叠点。保存/加载的坐标一律是原始 IR `(col,row)` 与实际 RGB `(u,v)` 像素，预览缩放/留白不会进入 H；改变显示放大倍数不需要重标。

注意：冻结只保证选点用同一对缓存帧，不代表两相机硬同步。IR 停更超过 3 秒或 RGB 确认断流会暂停新采点。单应矩阵还依赖目标平面相对相机的距离和姿态；跨停位必须实测验证，不能仅因相机支架固定就认定 H 通用。现场的 RMS、跨停位误差、排插排除效果仍需实机验收。

### 标注四层中间窗（独立于 H）

红外主窗点击 **「标注四层中间窗…」**：

1. 点击「冻结 RGB / 识别四角」。使用本地 `~/AI视觉/building_d.pt`（Pose、四点顺序 TL/TR/BR/BL），不额外打开 RGB 或 IR 设备。检查绿色四边形确实是楼宇正面；无楼宇、多个候选、低置信或四角出界时不接受。
2. 按 **1F→2F→3F→4F（从下往上）**，点每层中间窗中心。会自动切到下一层；要改某层，先点对应单选按钮或表格行再点图。四点都取自这张冻结 RGB，无需加热，也不需要重标 H。
3. 「保存四窗模板」将楼面归一化坐标写入输出目录的 `红外热像/window_centers.json`；**不会改写已有 calibration.json**。模板记录 RGB 尺寸/镜像和权重指纹，更换模型、分辨率或镜像后不静默复用。点击「加载模板」后再次冻结识别，会把已存中心映射到当前图，供回看和修改。
4. 关闭标注窗后共享模型后台继续识别；主窗以绿色显示楼体四边形、蓝色标出四个中央窗中心。以后重启会自动加载匹配的模板，需点 **「启用窗户匹配」** 才启动模型。默认不在启动时偷偷加载 YOLO。
5. 已有 H 和有效 IR 时，黄色热像投影圆圈与四个窗中心在正视楼面坐标中比较，显示 **RGB 中央窗几何候选**。只有投影在楼宇内且距离不超过最小层间距的 45% 才判层；离窗太远、楼外、模型结果超过 1.2 秒、角点不可靠时显示未确认，不强行选最近层。

原「IR 行号楼层」读数仍保留，二者明确分开。几何匹配只回答位置属于哪层，不自动宣称高温异常；实际温度/温差报警阈值未在此功能中设定。IR/RGB 异步采集，仍需车辆和目标稳定，且原 H 在当前工作距离/姿态下有效。新模型和四窗模板的真实判层效果需标注后实机复测。

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


## 文字识别模式（中文 OCR / 车牌 OCR）

一个 OCR 模式、两个页签，共用摄像头预览和“保存结果”按钮：

- **中文文字**：PaddleOCR（`lang=ch`），检测中文/英文/数字/标点，结果按 `#序号` 列出；
  可以选择一个 YOLO 模型先检测楼宇/文字区域，只对裁剪图做 OCR，省掉背景干扰；
- **车牌识别**：两种模式——
  - **沙盘/非标（通用 OCR，默认）**：用 PaddleOCR 读车牌框内文字，不要求首字是省份缩写；
  - **真实车牌（HyperLPR3）**：输出车牌号、车牌类型（蓝牌/黄牌/绿牌等）和置信度。

### 操作

| 操作 | 说明 |
| --- | --- |
| 空格 / 回车 | 识别当前画面；用当前选中的页签决定走中文还是车牌 |
| 「选择裁剪模型…」 | 中文页签可选：先跑 YOLO 检测楼宇/区域，再对裁剪图 OCR |
| 「清除裁剪模型」 | 回到整图 OCR |
| 车牌页签「选择车牌裁剪模型…」 | 可选训练好的 `plate.pt`，先裁车牌再识别；模型存在时默认自动选上 |
| 「清除车牌裁剪模型」 | 回到整图车牌检测 |
| 车牌页签「通用 OCR」 / 「HyperLPR3」 | 切换裁剪后的识别后端 |
| 「恢复实时预览」 | 从识别结果图回到摄像头实时画面 |
| 「保存当前结果」 | 保存带框 JPG + JSON 到 `~/Pictures/photo_capture/OCR/` |
| `Esc` / 关窗口 | 退出并释放摄像头 |

识别是**手动触发**的，不逐帧跑：按一下抓当前帧，后台线程加载模型/推理，完成后在预览里
画框并在侧栏列文字。PaddleOCR 中文模型第一次加载约 1 秒；HyperLPR3 首次加载约 1 秒。
启动时不要连续按，等状态栏显示完成再按。

### 中文 OCR + YOLO 裁剪

中文页签点「选择裁剪模型…」，选任意 Ultralytics `.pt`（例如楼宇检测/`building_d.pt`），
识别流程变成：

```text
当前帧 → YOLO 检测框 → 按框外扩 6% 裁剪 → 每个裁剪图跑 PaddleOCR
       → 文字框坐标映射回原图 → 画框 + 侧栏列表
```

最多取面积最大的 8 个框。模型没检测到框、或裁剪图没识别到文字时，会**自动回退整图 OCR**，
状态栏会显示“已回退全图”。模型路径只在当前 GUI 会话里保留，重启后重新选择。

### 车牌 YOLO 裁剪

车牌页签和中文页签一样，可以选训练好的车牌检测模型（默认自动使用
`~/AI视觉/plate.pt`，不存在时显示“未选择：整图检测”）：

```text
当前帧 → plate.pt 检测车牌框 → 外扩 6% 裁剪
       → 裁剪后通用 OCR（默认）或 裁剪后 HyperLPR3（先加 35% 背景）
       → 坐标映射回原图 → 画框 + 侧栏文本
```

- **裁剪后通用 OCR（默认）**：PaddleOCR 直接读裁剪图，不要求首字是省份；
- **裁剪后 HyperLPR3（加背景）**：给纯车牌裁剪图外加 35% 灰色背景再喂 HyperLPR3，
  缓解纯车牌图检测不到的问题；但它仍可能把非省份首字纠正成省份。
- 车牌模型没框、或裁剪图没文字时，自动回退整图检测。

### 车牌：沙盘/非标为什么用通用 OCR

HyperLPR3 的识别字典按真实中国车牌设计，第一字期望是省份简称。比赛沙盘的车牌首字可能
是任意汉字/字母，HyperLPR3 会把首字“纠正”成某个省份，导致读错。

所以沙盘场景建议 **裁剪后通用 OCR**：

1. `plate.pt` 或 HyperLPR3 只负责找车牌框；
2. PaddleOCR 读取框内文字，不限制首字必须是省份；
3. 框内没文字时，整图 OCR 后按长宽比 1.8–8.0、长度 5–12 过滤车牌行。

实测合成沙盘牌 `沙B12345`：
- 裁剪后通用 OCR：`沙B12345`，置信度约 0.94；
- 裁剪后 HyperLPR3 加背景：仍可能读成 `沪B12345`，说明沙盘牌应优先用通用 OCR。
真实车牌切到 HyperLPR3 可保留车牌类型和置信度。

### 为什么选这两个

- 中文文字：本机已有 PaddleOCR + 中文模型，`ch_PP-OCRv4` 对中文/数字/标点效果稳定，
  比 EasyOCR 更贴近中文场景，CPU 也能到约 1–2 帧/秒；
- 车牌：HyperLPR3 是专门做中国车牌的检测+识别+颜色分类流水线，比通用 OCR 直接认车牌更稳，
  对蓝色/绿色/黄色车牌都有对应类型输出。

### 输出

```text
~/Pictures/photo_capture/OCR/
├── 中文_YYYYmmdd_HHMMSS_mmm.jpg
├── 中文_YYYYmmdd_HHMMSS_mmm.json
├── 车牌_YYYYmmdd_HHMMSS_mmm.jpg
└── 车牌_YYYYmmdd_HHMMSS_mmm.json
```

JSON 包含 `kind`、`text`、`message`、每行 `text/confidence/points` 和保存时间。
中文结果画青蓝色框，车牌结果画橙色框；不使用绿色框。

### 依赖

```bash
python3 -c "import paddleocr, hyperlpr3, onnxruntime; print('ok')"
```

本机模型已缓存；若换机器首次运行会自动下载。PaddleOCR 模型目录
`~/.paddleocr/whl`，HyperLPR3 模型目录 `~/.hyperlpr3/20230229`。
`onnxruntime` 对 Python 3.8 固定用 1.16.x。

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
| `--pose-model` | `~/AI视觉/gauge.pt` | 仪表 Pose 权重 |
| `--gauge-conf` | `0.5` | 仪表目标置信度 |
| `--gauge-kpt-conf` | `0.3` | 关键点的最低置信度 |
| `--gauge-arc` | `long` | `long` / `short` / `auto` 刻度弧 |
| `--gauge-pointer` | `tip` | `tip` 模型尖端（默认）/ `radon` 正视化后射线搜索 |
| `--gauge-temp-min` | `-30` | 温度量程下限 |
| `--gauge-temp-max` | `50` | 温度量程上限 |
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
- **仪表识别报“不是 pose 模型”**：要用 gauge.pt 这类姿态权重，不能选普通检测模型。
- **仪表读数不对**：先确认量程（默认温度 -30~50、湿度 0~100）和刻度弧
  （`long/short/auto`）；默认长弧适合大多数圆形表。固定模板不要求正对；
  中间刻度按 -10/10/30 和 20/40/70 插值，如果表盘刻度值/角度不是这套，
  需要改 `gauge.py` 里的 `gauge_spec_for()` / `CANONICAL_TICK_ANGLES`。
- **CPU 上很慢**：`--imgsz 416`，或关掉其他占 CPU 的程序；有 GPU 用 `--infer-device cuda`。
- **拍照还是糊**：确认终端打印的“摄像头画面”是 1280×720，且锐度默认 3；弱光/固定对焦
  （多数小相机）/运动仍会造成模糊，保持静止、补光，或试 `--sharpness 6`、1080p 看细节。
- **中文类别显示成方块**：脚本自动找 Noto CJK / 文泉驿字体；找不到时只画框、不画文字。
