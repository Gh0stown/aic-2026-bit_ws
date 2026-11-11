# 把红外热像接进采集 GUI（交接说明）

面向 `src/photo_capture/scripts/capture_photos.py`（tkinter 采集 GUI）。
热像模块已经写好并测过，GUI 侧**不需要**改 `mlx90640_ros` 内部，只调用公开接口。

## 1. 先明确边界

- 热像只回答“画面里哪个像素最热、对应哪一层”，**不提供世界坐标**。
- 热像走串口（ESP32-S3），白光相机走 `/dev/video0`，两者互不影响。
- 一个进程只能有一个 `ThermalSession`（厂商库把校准放在进程全局）；也不能和
  `roslaunch mlx90640_ros` 同时抢同一个串口。

## 2. 现成接口

```python
from mlx90640_ros.thermal_session import ThermalSession, FloorTable
```

运行前先 `source devel/setup.bash`，并确认包已 `catkin_make`。

### 2.1 创建 / 启动

```python
# 实机
session = ThermalSession(
    port="/dev/serial/by-id/确认属于ESP32的设备",
    scale=12,                 # 伪彩放大倍数，12 → 384×288，再加 42 行文字区
    delta_c=2.0,              # 热点框：比峰值低 2℃ 以内算同一热区
    roi=None,                 # 或 (r0, c0, r1, c1) 半开，排除背景
    floor_table="1F:16-23,2F:8-15,3F:0-7",
)
session.start()

# 无硬件开发：回放抓包，loop=True 会一直循环
session = ThermalSession(capture=".../test/data/capture.bin", loop=True).start()
```

`start()` 起后台线程，1 Hz 出完整图；不阻塞 Tk 主循环。

### 2.2 轮询最新帧（Tk `after` 里调用）

```python
frame = session.latest()      # 可能为 None（还没收到第一帧）
if frame is not None:
    frame.bgr                 # 已画好热点框和楼层标签的 BGR 图
    frame.temps               # 24×32 float32 摄氏度原始矩阵
    frame.summary()           # dict：peak_c/row/col/box/area/mean_c/floor/ambient
    frame.floor               # "1F" / "2F" / ""（没配表时为空）
```

`summary()` 和 ROS 的 `/mlx90640/hotspot` 字段一致，直接给上报/日志用。

### 2.3 现场标定

```python
session.record_floor("2F")          # 用当前帧的峰值行记录 2F
session.record_floor("1F", 20)      # 或显式给行号
session.floor_table.to_text()       # → "3F:0-7,2F:8-15,1F:16-23"
session.set_floor_table("...")      # 载入已有表
```

`record_floor` 会按“相邻层中心的中点”自动重算连续区间，不需要手算边界。
标定方法：车停 12 号点，暖宝宝一次只贴一层，等两秒点“记录 N 层”。

### 2.4 截图

```python
png, json_path = session.save_snapshot(directory=None, prefix="thermal", temps=False)
```

默认存 `~/Pictures/mlx90640/`，PNG + 同名 JSON。`temps=True` 时 JSON 里带 24×32 温度矩阵（调试用）。

### 2.5 退出

```python
session.stop()    # set stop → 关串口 → join(2s)，窗口关闭时调用
session.error()   # 后台线程异常（串口断开等），None 表示正常
```

## 3. GUI 落点建议

`capture_photos.py` 已经是“启动选模式 → 每个模式一个 App 类”的结构，最小改动是
**加第 5 个模式**，而不是塞进 `PhotoApp`：

1. `MODE_THERMAL = "thermal"`（放在 `MODE_GAUGE` 后面，约 129 行）。
2. `ModeDialog` 的按钮列表加一项“红外热像”（约 985 行那段可用性判断）。
3. 新增 `ThermalApp`，照 `PhotoApp` 的骨架：`root`、`_workspace`、`_CapturePreview`、
   `root.after(50, self._tick)`、`WM_DELETE_WINDOW` → `session.stop()`。
4. `main()` 里按 mode 分派到 `ThermalApp`。

预览直接复用 `_CapturePreview`：

```python
from PIL import Image

def _tick(self):
    frame = self.session.latest()
    if frame is not None and frame is not self._shown:
        self._shown = frame
        rgb = frame.bgr[:, :, ::-1]
        photo = self.preview.fit_image(Image.fromarray(rgb))
        self.preview.configure(image=photo, text="")
        self.peak_var.set("{:.2f} ℃".format(frame.hotspot.peak_c))
        self.row_var.set("row {:.1f} / col {:.1f}".format(frame.hotspot.row, frame.hotspot.col))
        self.floor_var.set(frame.floor or "—")
    elif self.session.error() is not None:
        self.status_var.set("热像异常：{}".format(self.session.error()))
    self._after_id = self.root.after(50, self._tick)
```

注意 `ImageTk.PhotoImage` 要保留引用（`_CapturePreview.fit_image` 已经做了）。

## 4. 界面建议（右侧设置区）

| 控件 | 作用 |
|---|---|
| 串口输入框 + “应用” | `/dev/serial/by-id/...`，改完重建 session |
| “回放抓包”按钮 | 无硬件调界面用，`capture=...` |
| 楼层表输入框 | 直接改 `1F:16-23,...`，`set_floor_table` |
| `hotspot_delta` 输入 | 默认 2.0，热区框大小 |
| 三个“记录 N 层”按钮 | 1F/2F/3F，调 `record_floor` |
| 当前读数 | 峰值温度、row/col、楼层 |
| “保存快照” | `save_snapshot` |

写回楼层表时用 `to_text()`，不要自己拼字符串。

## 5. 如果不想开串口、想走 ROS 话题

先跑节点：

```bash
roslaunch mlx90640_ros mlx90640.launch port:=/dev/serial/by-id/... floor_map:="1F:16-23,2F:8-15,3F:0-7"
```

GUI 里订阅（`photo_capture` 已经依赖 rospy）：

```python
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json, numpy as np

def _on_thermal(self, msg):
    self._thermal_bgr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)

def _on_hotspot(self, msg):
    self._hotspot = json.loads(msg.data)

rospy.Subscriber("/mlx90640/thermal_image", Image, self._on_thermal, queue_size=1)
rospy.Subscriber("/mlx90640/hotspot", String, self._on_hotspot, queue_size=1)
```

这条路线的好处是标定/参数都在 launch 里，GUI 只显示；坏处是采集时要多开一个节点。
二选一，**不要两套同时开**。

## 6. 坑

- 串口身份必须先 `ls -l /dev/serial/by-id/` 确认，不能用 `/dev/carserial`、`/dev/ttyUSB0` 猜。
- 相机装反了楼层表要整体倒过来：`row=0` 是图像顶部。
- 12 号点停位差几厘米，行号会整体平移；标定要按多次停车的中间值录。
- 热层会往上下邻层扩散，判层用峰值行（`frame.hotspot.row`），不要用框中心或阈值区域。
- `stop()` 后再 `start()` 同一个对象会报错，换串口请重建 session。

## 7. 验收清单

- [ ] 无硬件：`capture=` 回放能出图、能改楼层表、能保存快照。
- [ ] 实机：确认串口后 1 Hz 出图，拔掉串口 GUI 不崩、显示错误。
- [ ] 三层各贴一次暖宝宝，“记录”后 `to_text()` 与预期一致。
- [ ] 关窗口时 `stop()` 无异常，进程能正常退出。
- [ ] 连续跑 10 分钟不涨内存（`latest()` 每帧替换，不累积）。
