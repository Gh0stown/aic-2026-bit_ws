# 新摄像头（/dev/video0，Fic760x）画质调研

2026-11-19 对本机新装的 UVC 摄像头做的能力探测与画质实测。结论可直接用于
`photo_capture` 的取流参数；未做标准靶标 MTF 标定，所有“清晰度”都是相对比较。

设备：`eba4:6579` / `Fic760xUsbCamera`（Image+），UVC 1.00，USB2 高速，
挂在 `usb 1-1.3`（视频节点 `/dev/video0`，`/dev/video1` 是同设备的 metadata 节点）。

## 结论（先看这个）

1. **这是 5MP/4K 摄像头，默认取流只用到了 720p。** `photo_capture` 默认
   `--width 1280 --height 720`，对这颗模组只用了约 1/9 的像素。拍仪表、
   拍文字/车牌这类需要“目标上像素多”的场景，建议：
   - **2592×1944（5MP，4:3）**：传感器中心裁切，等效约 1.33× 数码变焦（ORB 特征匹配实测），
     仪表/目标占的像素更多，适合仪表读数；
   - **3840×2160（4K，16:9）**：视野与 720p/1080p 相同，细节最多，适合大场景/楼宇。

2. **镜头是手动对焦，软件调不了。** 固件虽然暴露了 `Focus, Absolute`（0–1023）和
   `Focus, Automatic Continuous` 两个控制项，但实测把 focus 从 0 扫到 960，
   画面 Laplacian 方差始终 6744–6782（1280×720，差值 <1%），**完全没有变化**。
   说明书第 1 条也写的是“不清晰可旋转镜头调整焦距”，并提醒**不要拧到最里面**。
   清晰度的第一杠杆就是：对准实际拍摄距离，手动拧镜头环对焦。

3. **锐度控制范围是 1–64（默认 32），不是旧文档写的 0–6。**
   `photo_capture` 的 `--sharpness` 默认 3，只是这颗相机的最小值附近。实测
   3 与 32 的差别远小于“对焦/分辨率”带来的差别；建议用相机默认值
   （`--sharpness -1` 完全不改，或显式 `32`），不要再用 3。

4. **帧率够用。** 用 `cap.grab()`（不解码）实测：720p/1080p/5MP/4K 在 30fps 档都是
   ~29fps；5MP/4K 走 MJPG 压缩流，Python 解码每帧更慢，GUI 实时预览会慢一些（沙箱里
   用 `read()` 测到 10–15fps，机上还没实测）。
   USB2 带宽能撑到 4K30（约 18–20 MB/s），但已接近等时传输上限，和 Orbbec 等其他
   USB 设备同时高负载时要留意掉帧。

5. **视野差异**：720p/1080p/4K 是同一视野（16:9）；5MP（4:3）是中心裁切，视野比
   16:9 窄约 1.33×（同距离目标更大）。拍仪表用 5MP，装不下就退 4K/1080p。

## GUI 默认参数（可直接抄）

下面是给 `photo_capture` GUI / `Camera` 类写默认设置用的参数表。取值来自 2026-11-19
对 `/dev/video0`（Fic760x）的实测；换别的 UVC 摄像头（如 Orbbec RGB）时不要套用。

### 1. 设备识别

`/sys/class/video4linux/videoN/name` 里带 `Fic760x` 的才是这颗新相机：

| video 节点 | sysfs name | 用途 |
| --- | --- | --- |
| `/dev/video0` | `Fic760xUsbCamera: Usb HDCamera` | 新白光相机，用下面预设 |
| `/dev/video1` | 同上 | 同设备 metadata 节点，不取流 |
| `/dev/video2` | `Orbbec DaBai DCW2 RGB Camera` | 深度相机 RGB，保持旧参数 |

```python
# 选完设备后（CameraSelectDialog 已有 list_video_cameras() 读到 name）
is_new_cam = "Fic760x" in device_name
```

### 2. 全局默认（新相机）

| 参数 | 默认值 | 允许范围/备注 |
| --- | --- | --- |
| 采集分辨率 | **2592×1944** | 5MP，4:3，中心裁切约 1.33× 数码变焦（拍仪表首选） |
| 备用分辨率 | 3840×2160 | 4K，16:9，与 720p 同视野、细节最多，仅拍照模式可选 |
| 帧率 | **15** | 5MP/4K 走 15fps 省 USB2 带宽；预览/检测用 720p@30 |
| 像素格式 | **MJPG** | 5MP/4K 只能 MJPG；YUYV 最高 1280×720@10 |
| 锐度 | **32** | 范围 1–64，相机默认即 32；不要用 3（旧默认） |
| 自动曝光 | **开**（`CAP_PROP_AUTO_EXPOSURE=3`） | 手动=1；静态场景需要时再开给用户 |
| 曝光时间 | 不设 | 自动模式下写 `EXPOSURE` 无效（读回值不变） |
| 增益 | **0** | 0–15；弱光下别靠拉增益，先补光 |
| 自动白平衡 | 开（`CAP_PROP_AUTO_WB=1`） | 手动时色温 5000 起步（1800–10000） |
| 背光补偿 | 0 | 逆光场景可试 1 |
| 电源频率 | 50Hz | 本机已是 50Hz（V4L2 值 1），不用改 |
| 对焦 | **不要设** | `Focus, Absolute`/AF 实测无效，镜头手动对焦 |
| JPEG 保存质量 | 95 | 脚本现默认，拍照模式保持 |
| 镜像 | 关 | 拍照场景不需要 |

### 3. 分模式默认

| 模式 | 分辨率 | FPS | 格式 | 锐度 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 拍照 `photo` | 2592×1944 | 15 | MJPG | 32 | 照片像素多，仪表/文字细节好 |
| 仪表 `gauge` | 2592×1944 | 15 | MJPG | 32 | 表盘占像素约 2.7× 线性（相对 720p） |
| 文字/车牌 `ocr` | 2592×1944 | 15 | MJPG | 32 | 裁剪后 OCR 更稳；慢可退 1920×1080 |
| 检测 `detect` | 1280×720 | 30 | MJPG | 32 | 实时 YOLO，5MP 解码太慢 |
| 区域扫描 `scan` | 1280×720 | 30 | MJPG | 32 | 拼接上限 `--stitch-max-width 1600` |
| 拍照-4K 备选 | 3840×2160 | 15 | MJPG | 32 | 全视野，单帧解码约 2× 5MP |

> 注意：2592×1944 是 4:3 中心裁切，取景比 720p/1080p/4K（16:9）窄约 1.33×，
> 换到 5MP 后 GUI 里的取景提示/预览框要留余量，别按 16:9 的边距画 ROI。

### 4. OpenCV / V4L2 对应关系

| 项目 | 建议值 | OpenCV 属性 | V4L2 控制 |
| --- | --- | --- | --- |
| 分辨率 | 2592×1944 | `CAP_PROP_FRAME_WIDTH/HEIGHT` | `VIDIOC_S_FMT` |
| 帧率 | 15 | `CAP_PROP_FPS` | 帧率枚举 |
| 格式 | MJPG | `CAP_PROP_FOURCC`（`'MJPG'`） | `VIDIOC_S_FMT` |
| 锐度 | 32 | `CAP_PROP_SHARPNESS` (20) | `V4L2_CID_SHARPNESS` 0x0098091b |
| 自动曝光 | 3 | `CAP_PROP_AUTO_EXPOSURE` (21) | `V4L2_CID_EXPOSURE_AUTO` 0x009a0901 |
| 曝光时间 | 不设/手动 100–250 | `CAP_PROP_EXPOSURE` (15) | `V4L2_CID_EXPOSURE_ABSOLUTE` 0x009a0902 |
| 增益 | 0 | `CAP_PROP_GAIN` (14) | `V4L2_CID_GAIN` 0x00980913 |
| 自动白平衡 | 1 | `CAP_PROP_AUTO_WB` (44) | `V4L2_CID_AUTO_WHITE_BALANCE` 0x0098090c |
| 色温 | 5000 | `CAP_PROP_WB_TEMPERATURE` (45) | `V4L2_CID_WHITE_BALANCE_TEMPERATURE` 0x0098091a |
| 背光补偿 | 0 | 无对应属性，需 ioctl | `V4L2_CID_BACKLIGHT_COMPENSATION` 0x0098091c |
| 电源频率 | 50Hz | 无对应属性，需 ioctl | `V4L2_CID_POWER_LINE_FREQUENCY` 0x00980918（1=50Hz） |
| 对焦 | 不设 | `CAP_PROP_FOCUS/AUTOFOCUS` 实测无效 | 0x009a090a / 0x009a090c |

设置顺序建议：`FOURCC → FRAME_WIDTH/HEIGHT → FPS → 其它控制`。现有 `Camera.open()`
就是这个顺序，照抄即可。

### 5. 代码落点（现有 `capture_photos.py`）

- `parse_args()` 里 `--width` 默认 1280（约 L6611）、`--height` 默认 720（L6613）、
  `--sharpness` 默认 3（L6616）——GUI 各模式都直接复用 `args.*`，**改这三处即可改
  GUI 默认值**；10 处 `Camera(...)` 调用不需要逐个改。
- `Camera.__init__()` 的 `sharpness: int = 3`（L387）同步改 32，命令行之外直接
  用 `Camera` 的调用才不会退回 3。
- `--sharpness` 的 help 文本还写着“0-6”，建议改成“1-64（本机默认 32），-1 保持原值”。
- 建议把 `--width/--height` 默认改成 `None`，然后按选中的设备名填：

```python
# run_gui() 选完设备之后（args.device 已确定）
name = device_name_of(args.device)          # 复用 list_video_cameras() 的 name
if args.width is None and args.height is None:      # 用户没显式传 --width/--height
    if "Fic760x" in name:
        args.width, args.height, args.fps, args.sharpness = 2592, 1944, 15, 32
    else:                                    # 旧相机/Orbbec RGB 保持原行为
        args.width, args.height, args.fps, args.sharpness = 1280, 720, 30, -1

# 实时模式再覆盖（5MP 解码撑不住实时检测/拼接）
if mode in (MODE_DETECT, MODE_SCAN):
    args.width, args.height, args.fps = 1280, 720, 30
```

> 把 `--width/--height` 默认改成 `None` 后，CLI 入口 `run_cli()` 也要补默认值，
> 否则命令行不传 `--width` 时会退到相机默认（640×480）。不想动 CLI 的话，就只在
> `run_gui()` 里覆盖 GUI 用的参数，CLI 的 1280×720 保持原样。

### 6. 验收清单（改完 GUI 默认后）

- 启动后终端“摄像头画面”应打印 **2592×1944**（或按模式 1280×720 / 3840×2160），
  和 `camera.frame_size` 一致；不一致说明该分辨率没被接受，代码会读回实际值。
- `CAP_PROP_FPS` 读回 30 是相机枚举值，不代表实际；实测 `cap.grab()` 约 29fps，
  5MP/4K 设 15fps 时约 14–15fps。
- 锐度读回应为 32；退出后 `Camera.release()` 会恢复打开前的原值（当前相机原值也是 32）。
- 5MP/4K 下预览明显变卡时：把预览显示帧率降到 10、或只对预览用 720p 另开一路
  （不建议同一台相机开两路，UVC 只能一个流）。
- 别把 `focus` 控件接进 GUI：接上也没效果，会误导用户。

### 7. 不建议在 GUI 里暴露/自动设置的

- 亮度/对比度/饱和度/伽马/背光补偿：保持相机默认（32/32/32/32/0）即可，没有实测收益；
- 色温/自动白平衡：保持自动；强光/灯管环境再手动 5000K；
- 4K 只建议作为“拍照”模式的备选档，不建议做实时检测默认（解码开销约 2× 5MP）。

## 设备能力（V4L2 实测）

| 格式 | 分辨率 | 帧率 |
| --- | --- | --- |
| MJPG | 640×480 / 800×600 / 1280×720 / 1600×1200 / 1920×1080 / 2592×1944 / 3840×2160 | 30 / 15 |
| YUYV | 1280×720 / 800×600 / 640×480 | 10 / 5 / 30 |

控制项（用 VIDIOC_QUERYCTRL 读出，单位即 UVC 原始值）：

| 控制 | 范围 | 默认 | 当前 |
| --- | --- | --- | --- |
| Brightness / Contrast / Saturation / Hue | 1–64 | 32 | 32 |
| Gamma | 0–64 | 32 | 32 |
| Gain | 0–15 | 0 | 0 |
| Sharpness | 1–64 | 32 | 32 |
| Backlight Compensation | 0–1 | 0 | 0 |
| White Balance, Automatic | 0–1 | 1 | 1 |
| White Balance Temperature | 1800–10000 | 5000 | 5000 |
| Power Line Frequency | 0–2 | 1（50Hz） | 1 |
| Auto Exposure | 0–3 | 3 | 3 |
| Exposure Time, Absolute | 1–5000 | 100 | 100（自动模式下不生效） |
| Focus, Absolute | 0–1023 | 200 | 370 |
| Focus, Automatic Continuous | 0–1 | 1 | 1 |

对焦两个控制项实测无效（见上），说明是纯手动镜头；不要指望 `--autofocus` 类设置。

## 实测记录

### 对焦控制扫描（1280×720 MJPG，AF 关，手动设 focus）

focus = 0/64/128/…/960 → Laplacian 方差 6757.6/6745.9/6762.2/…/6777.1，最强 0.5%
梯度 640.0–641.4。**全平**，确认软件对焦无效。

### 锐度扫描（固定手动曝光，同一会话内交替切换，避免场景漂移）

- 1280×720@30：sharpness 1/3/32/48/64 → Laplacian 方差
  11482/12217/12168/16800/22096（单调变锐，64 最强，但边缘更硬、噪点更明显）。
- 2592×1944@15：sharpness 1/3/32/48/64 → 3069/1446/1582/1409/2157，
  不再单调，且数值受噪点影响；900×900 原生裁片肉眼看差别很小。

结论：锐度保持相机默认 32 最省心；720p 下要更硬的边缘可试 48，不建议长期 64
（5MP/4K 下锐度不是主要瓶颈，先解决对焦和分辨率）。

### 分辨率/帧率（`cap.grab()` 不解码，30fps 档）

| 分辨率 | 实测 grab fps | 备注 |
| --- | --- | --- |
| 1280×720 | 29.1 | 默认；预览/检测最省 |
| 1920×1080 | 29.1 | 同视野，细节更多 |
| 2592×1944 | 29.1 | 4:3 中心裁切（≈1.33× 数码变焦） |
| 3840×2160 | 29.1 | 同视野，细节最多；解码开销最大 |

（早期用 Python `read()` 测到 10–22fps，是沙箱里 JPEG 解码 + Python 开销造成的，
不代表相机只能跑那么慢。）

### 720p vs 5MP（同场景示意）

测试产物在 `.camera_quality_review/`（本地 review 目录，未入库）：
`fov_compare.jpg`（四种分辨率视野对比）、`compare_720p_vs_5mp.jpg`（仪表裁片对比）、
`samples3/`（各参数样张）。ORB 匹配实测 5MP 相对 720p 是 1.33× 数码变焦，加上 2× 分辨率，
同距离目标上的线性像素约 2.7×（面积约 7×）；但 5MP 每像素更柔，实际有效细节增益会小于纸面值。

## 推荐用法

```bash
# 仪表读数 / 需要目标占更多像素（5MP，4:3，约 1.33× 数码变焦）
rosrun photo_capture capture_photos.py --mode photo \
    --width 2592 --height 1944 --sharpness 32

# 全视野最大细节（4K，16:9；预览/解码会慢一些）
rosrun photo_capture capture_photos.py --mode photo \
    --width 3840 --height 2160 --sharpness 32

# 实时预览 / YOLO 检测 / 区域扫描：保持 720p
rosrun photo_capture capture_photos.py --mode detect --width 1280 --height 720
```

对焦步骤（重要）：

1. 把车/相机摆到实际拍摄距离（仪表约 40–60cm，按现场定）；
2. 用 5MP 预览启动：`--width 2592 --height 1944`，画面会实时刷新；
3. 一边看预览一边轻拧镜头环（说明书：不要拧到最里面），到最锐为止；
4. 拍一张看 100% 原图确认；换拍摄距离要重新对焦（手动镜头没有自动对焦）。

拍仪表/文字时另外两点：

- **补光通常比调参数有效**（经验判断，未做对照实验）：逆光（背后是窗户）时表盘会暗、
  噪点高；顺光或加一盏柔光灯，通常比拉锐度/改曝光更明显。
- **增益保持 0、曝光默认自动**：静态场景觉得忽明忽暗时可以手动曝光
  （脚本目前没有给 CLI 暴露曝光控制，需要在代码里设），但先解决对焦。
- 如果同时跑 Orbbec 深度相机/其它 USB 设备时掉帧，先把白光摄像头换到机身另一个
  USB 口，避开同一个 USB2 Hub（当前挂在 `usb 1-1.3` 的 Hub 上）。

## 未验证/风险

- 没有标准分辨率靶标，清晰度指标是 Laplacian/梯度统计，只做相对比较；
- 5MP/4K 在 GUI 里长时间预览的 CPU 占用没实测，实时检测建议仍用 720p；
- 4K30 接近 USB2 等时带宽上限，和 Orbbec/其他 USB 设备同时高负载时可能掉帧；
- 本次没有改 `photo_capture` 的默认值（仍是 1280×720 + 锐度 3），推荐参数用命令行传。
