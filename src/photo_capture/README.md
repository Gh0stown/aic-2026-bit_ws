# photo_capture —— 拍照 / YOLO 检测 / 区域扫描 / 仪表识别

调用本机摄像头的四个模式：

- **拍照模式**：按类型采集照片，照片存到 `输出目录/类型/`；可设连拍张数，运行中换类型。
- **模型测试模式**：从 `~/AI视觉` 选 YOLO `.pt`，摄像头实时画框，显示类别、置信度、推理耗时；
  可换模型、调置信度、保存带框截图。
- **区域扫描模式**：控制麦轮底盘横向平移，停一下拍一张，最后拼成横向大图；
  也可不用底盘、手动遥控平移，脚本只负责抽帧和拼接。拼接后可用 YOLO 分块检测全景。
- **仪表识别模式**：加载 Pose.pt，识别温度/湿度表的旋转轴、指针尖端、最小/最大刻度，
  按量程换算读数，保存截图和 JSONL 记录。

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

> OpenCV 编译时没有 GUI 后端（`GUI: NONE`），预览用 tkinter，不用 `cv2.imshow`。

## 运行

```bash
source devel/setup.bash
rosrun photo_capture capture_photos.py

# 或者直接跑脚本
python3 src/photo_capture/scripts/capture_photos.py
```

启动后先弹「选择启动模式」：点按钮或按 `1`（拍照）/ `2`（模型测试）/ `3`（区域扫描）/
`4`（仪表识别）。想跳过询问可以用 `--mode photo` / `--mode detect` / `--mode scan` /
`--mode gauge`。

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
- 低速（默认 0.06 m/s）、短步长、每步归零，退出/异常都会发零速度；
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

默认参数：向左 1.0m、5 帧、每步 0.25m、速度 0.06 m/s。界面会显示实际步长。

### 手动录制

1. 扫描方式选「手动录制（自己移动）」，设抽帧帧率；
2. 点「开始录制」，用你自己的遥控/节点移动（现有键盘遥控不能横移）；
3. 点「停止」（或回车），脚本按已录帧拼接。

### 拼接与结果

- 优先用 `cv2.Stitcher_SCANS`（平移扫描）；失败自动换 `PANORAMA`，再失败走
  相邻帧 ORB + RANSAC 单应兜底；
- 拼接失败不会丢帧，原始帧都在 `frames/`，可以调参数重拼；
- 拼接时每帧会缩到 `--stitch-max-width`（默认 1600）以内，控制内存。

保存结构：

```
~/Pictures/photo_capture/区域扫描/<区域名>_<时间戳>/
├── frames/
│   ├── 0001.jpg
│   └── ...
├── sweep.avi                 # 勾选“保存扫掠视频”时
├── panorama.jpg              # 拼接成功时
├── panorama_detected.jpg     # 全景检测后
└── meta.json                 # 参数、帧、位姿、拼接/检测结果
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
  --scan-speed 0.06 --scan-label 大厅 --yes

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

- 0 = 指针旋转轴
- 1 = 指针尖端
- 2 = 量程最小值刻度
- 3 = 量程最大值刻度

类别：`temperature` / `humidity`。读数计算：

1. 以旋转轴为圆心，求 尖端、最小刻度、最大刻度 的角度；
2. 把指针投影到 最小 → 最大 的刻度弧上，得到 0~1 的比例；
3. 按温度/湿度量程换算：`value = min + ratio * (max - min)`。

默认量程是 `0–100`（温度 °C、湿度 %RH），**请按实际表盘改**：

```bash
rosrun photo_capture capture_photos.py --mode gauge \
  --gauge-temp-min -20 --gauge-temp-max 60 \
  --gauge-humidity-min 0 --gauge-humidity-max 100
```

常见表计的刻度从最小值经顶部绕到最大值，默认用**长弧** `--gauge-arc long`；
如果表盘是短弧，换 `short`，拿不准用 `auto`。

### 界面操作

| 操作 | 说明 |
| --- | --- |
| 点「更换模型」 / `M` | 换一个 `.pt`（必须是 pose 任务） |
| 点「保存当前帧」 / `S` | 保存带关键点和读数的截图 |
| 点「保存读数」 / `R` | 把当前读数追加到 `readings.jsonl` |
| 点「打开记录目录」 | 打开当前模型的记录目录 |
| `Esc` / 关窗口 | 退出 |

截图和读数保存到：

```
~/Pictures/photo_capture/仪表识别/<模型名>/
├── snapshots/<模型名>_YYYYmmdd_HHMMSS_mmm.jpg
└── readings.jsonl
```

`readings.jsonl` 每行是一条记录，包含时间、模型、类别、读数、量程比例、指针角度、
目标置信度、关键点坐标和关键点置信度。

### 命令行

```bash
rosrun photo_capture capture_photos.py --no-gui --mode gauge \
  --pose-model ~/AI视觉/Pose.pt --gauge-conf 0.5 --gauge-kpt-conf 0.3

# 指定量程和弧
rosrun photo_capture capture_photos.py --no-gui --mode gauge \
  --gauge-temp-min 0 --gauge-temp-max 50 --gauge-humidity-min 20 --gauge-humidity-max 90 \
  --gauge-arc long
```

终端每秒打印一次，`Ctrl+C` 退出。没有图形界面时只打印，不保存截图/JSONL。

> 模型必须是 pose 任务；选错普通检测模型会提示“所选模型不是 pose 模型”。
> 关键点置信度低于 `--gauge-kpt-conf` 时不会算读数，界面会显示原因。

## 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--mode` | `ask` | `ask` 每次询问 / `photo` / `detect` / `scan` / `gauge` |
| `-d, --output-dir` | `~/Pictures/photo_capture` | 照片、模型截图、区域扫描、仪表记录的根目录 |
| `-t, --type` | 无（启动时询问） | 预填照片类型 |
| `-c, --count` | `1` | 一次连拍张数（1-99） |
| `--device` | `0` | 摄像头编号 / `/dev/video0` / 视频文件（测试用） |
| `--model` | 无 | YOLO 权重路径或文件名 |
| `--model-dir` | `~/AI视觉` | 模型目录 |
| `--conf` | `0.25` | 检测置信度阈值 0.01–0.99 |
| `--imgsz` | `640` | 推理输入尺寸，CPU 慢可用 `416` |
| `--infer-device` | `auto` | `auto` / `cpu` / `cuda` |
| `--width / --height` | 摄像头默认 | 采集分辨率 |
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
| `--scan-speed` | `0.06` | 平移速度 m/s（上限 0.10） |
| `--scan-settle` | `0.5` | 每步停稳时间（秒） |
| `--scan-fps` | `3.0` | 手动录制抽帧帧率 |
| `--scan-label` | 时间戳 | 区域名称 |
| `--scan-feedback` | `auto` | `auto` / `odom` / `time` |
| `--scan-clearance` | `0.25` | 平移前雷达净空（米），0 关闭 |
| `--stitch-max-width` | `1600` | 参与拼接的单帧最大宽度 |
| `--scan-record-video` | 关 | 保存 `sweep.avi` |
| `--no-stitch` | 关 | 只采集不拼接 |
| `--scan-dry-run` | 关 | 干跑，不连接 ROS、不产生真实运动 |
| `--detect-panorama` | 关 | 拼接后用 YOLO 分块检测 |
| `--yes` | 关 | CLI 自动扫描免确认 |
| `--pose-model` | `~/AI视觉/Pose.pt` | 仪表 Pose 权重 |
| `--gauge-conf` | `0.5` | 仪表目标置信度 |
| `--gauge-kpt-conf` | `0.3` | 4 个关键点的最低置信度 |
| `--gauge-arc` | `long` | `long` / `short` / `auto` 刻度弧 |
| `--gauge-temp-min` | `0` | 温度量程下限 |
| `--gauge-temp-max` | `100` | 温度量程上限 |
| `--gauge-humidity-min` | `0` | 湿度量程下限 |
| `--gauge-humidity-max` | `100` | 湿度量程上限 |
| `--force` | 关 | 忽略 `/cmd_vel` 上其他发布者的冲突 |

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
<输出目录>/仪表识别/<模型名>/readings.jsonl
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
  刻度弧（`long/short/auto`）；默认长弧适合大多数圆形表。
- **CPU 上很慢**：`--imgsz 416`，或关掉其他占 CPU 的程序；有 GPU 用 `--infer-device cuda`。
- **中文类别显示成方块**：脚本自动找 Noto CJK / 文泉驿字体；找不到时只画框、不画文字。
