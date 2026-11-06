# photo_capture —— 本机摄像头手动拍照

调用本机摄像头手动拍照的小工具：

- **启动时先问类型**：每次启动都会询问这次要拍哪种类型的照片（图形界面弹窗，
  命令行模式下用提示），照片保存到 `输出目录/类型/` 下，比如
  `~/Pictures/photo_capture/苹果/`。
- **一次连拍多张**：启动时和进入主界面后都能改“一次连拍几张”，按一下按钮就连续拍 N 张。
- **运行中可换类型**：主界面点「更换类型」或按 `T`，换完自动建新文件夹，不用重启。
- **手动触发**：点「拍照」按钮或按空格/回车才会拍，不会自动连拍。
- **两种界面**：有桌面时用 tkinter 预览窗口；没有图形界面（SSH、机器人上）时
  自动退化成命令行交互，功能一样。

## 依赖

Ubuntu / ROS Noetic 下一般只需要：

```bash
sudo apt install python3-opencv python3-tk python3-pil.imagetk
```

检查是否齐全：

```bash
python3 -c "import cv2, tkinter, PIL; print('deps ok')"
```

> 说明：本机 OpenCV 编译时没有带 GUI 后端（`cv2.getBuildInformation()` 里
> `GUI: NONE`），所以预览用 tkinter 实现，不用 `cv2.imshow`。

## 运行

```bash
# 在 catkin 工作区里
source devel/setup.bash
rosrun photo_capture capture_photos.py

# 或者直接跑脚本
python3 src/photo_capture/scripts/capture_photos.py
```

启动后会先出现「拍摄设置」窗口：填照片类型（可以直接输入新类型，也可以双击已有类型）、
选一次连拍张数，点「开始拍照」进入预览。

### 主界面操作

| 操作 | 说明 |
| --- | --- |
| 点「拍照」 / 空格 / 回车 | 按当前张数连拍，画面左上角会闪一下绿框 |
| 点「更换类型」 / `T` | 换照片类型，自动切到对应文件夹 |
| 改「一次连拍」数字 / `N` | 改一次拍几张（1-99） |
| 点「打开保存目录」 | 用文件管理器打开当前类型的文件夹 |
| `Esc` / 关窗口 | 退出 |

### 命令行模式

没有图形界面（或加 `--no-gui`）时：

```
直接回车 / p   拍一次（按当前张数）
t [类型]       换类型，例如：t 侧面
n [张数]       改连拍张数，例如：n 3
q              退出
```

也可以用管道喂命令，适合批量采集：

```bash
printf '正面\n\n\nq\n' | python3 src/photo_capture/scripts/capture_photos.py \
  --no-gui --count 3 --output-dir ~/Pictures/photo_capture
```

上面表示：类型选「正面」，拍两次（每次 3 张），然后退出。

## 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `-d, --output-dir` | `~/Pictures/photo_capture` | 照片根目录 |
| `-t, --type` | 无（启动时询问） | 预填照片类型 |
| `-c, --count` | `1` | 一次连拍张数（1-99） |
| `--device` | `0` | 摄像头编号 / `/dev/video0` / 视频文件（测试用） |
| `--width / --height` | 摄像头默认 | 采集分辨率，例如 `1280 720` |
| `--fps` | 摄像头默认 | 采集帧率 |
| `--format` | `jpg` | `jpg` 或 `png` |
| `--quality` | `95` | JPEG 质量 1-100 |
| `--interval` | `0.4` | 连拍两张之间的间隔（秒） |
| `--mirror` | 关 | 水平镜像，预览和保存都镜像（自拍用） |
| `--no-gui, --headless` | 关 | 不打开窗口，用命令行交互 |

例子：

```bash
# 指定第二个摄像头、720p、每次 5 张
rosrun photo_capture capture_photos.py --device 1 --width 1280 --height 720 --count 5

# 自拍镜像，存成 png
rosrun photo_capture capture_photos.py --type 自拍 --mirror --format png
```

## 保存规则

```
<输出目录>/
└── <类型>/
    └── <类型>_YYYYmmdd_HHMMSS_mmm_序号.jpg
```

例如：

```
~/Pictures/photo_capture/苹果/苹果_20261106_202408_640_0001.jpg
```

- 文件夹名就是启动时填的类型；`/ \ : * ? " < > |` 等字符会换成 `_`；
- 文件名带毫秒时间戳和序号，同一秒连拍也不会重名、不会覆盖旧照片；
- 类型文件夹是懒创建的：第一次拍该类型时自动建好。

## 常见问题

- **打不开摄像头**：确认 `ls /dev/video*` 有设备；换个编号试 `--device 1`；
  确认没有别的程序（浏览器、Zoom 等）占着摄像头。
- **没有预览窗口**：检查是否在 SSH 里、`DISPLAY` 是否为空；
  没有桌面时会自动用命令行模式，也可以显式加 `--no-gui`。
- **中文类型显示成方块**：预览叠加文字需要中文字体，脚本会自动找 Noto CJK /
  文泉驿；找不到字体时只是不画左上角信息，不影响拍照保存。
