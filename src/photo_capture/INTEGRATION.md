# 视频识别/计数接口（给导航脚本）

这个文件定义 `photo_capture` 对外稳定的视频计数接口。导航脚本只依赖这里的
`count_video()` / `--json` 输出，不要解析人类可读的 stdout。

## 1. 输入

一段由区域扫描录制的连续视频：

```
~/Pictures/photo_capture/区域扫描/<区域名>_<时间戳>/sweep.avi
```

视频要求：

- 1280×720 / 15fps（默认配置即可）；
- 相机在移动，目标在画面里连续出现；
- 同一目标会跨多帧，所以**不能逐帧相加**。

默认模型：`~/AI视觉/Cycle.pt`，类别 `UP` / `DOWN`。

## 2. Python 调用

```python
import sys
sys.path.insert(0, "/home/mowen/bit_ws/src/photo_capture/scripts")

from count_video import count_video

result = count_video(
    video="/home/mowen/Pictures/photo_capture/区域扫描/电动车采集_01_.../sweep.avi",
    model="/home/mowen/AI视觉/Cycle.pt",
    min_frames=20,
)

if result.ok:
    print(result.count)          # 唯一目标总数
    print(result.by_class)       # {'UP': 4, 'DOWN': 1}
    print(result.output_dir)     # 标注视频/裁剪图/JSON 的目录
else:
    print(result.message)
```

`count_video()` 返回 `CountResult`，关键字段：

| 字段 | 含义 |
|---|---|
| `ok` | 是否成功 |
| `count` | 唯一目标数 |
| `by_class` | 分类计数，如 `{"UP": 3, "DOWN": 2}` |
| `tracks` | 每条唯一轨迹的明细 |
| `total_frames` / `fps` | 视频信息 |
| `output_dir` | 输出目录 |
| `summary_path` | `track_summary.json` 路径 |
| `tracked_video` | 标注视频路径（可能为空） |
| `contact_sheet` | 目标缩略图路径（可能为空） |
| `message` | 失败原因 |

接口不抛业务异常；失败时 `ok=False`、`message` 说明原因。

## 3. 子进程 / JSON 接口

```bash
python3 /home/mowen/bit_ws/src/photo_capture/scripts/count_video.py \
  --video /path/to/sweep.avi \
  --model /home/mowen/AI视觉/Cycle.pt \
  --min-frames 20 \
  --json
```

`--json` 时 stdout **只有一行 JSON**，可以直接 `json.loads`；进度和人类日志不会混进去。

```python
import json, subprocess, sys

cmd = [
    sys.executable,
    "/home/mowen/bit_ws/src/photo_capture/scripts/count_video.py",
    "--video", video_path,
    "--model", "/home/mowen/AI视觉/Cycle.pt",
    "--min-frames", "20",
    "--json",
]
proc = subprocess.run(cmd, capture_output=True, text=True)
data = json.loads(proc.stdout)
if proc.returncode == 0 and data["ok"]:
    total = data["count"]
    by_class = data["by_class"]
```

返回码：`0` 成功，`2` 参数/视频/模型/依赖错误。

JSON 顶层字段：

```json
{
  "schema_version": 1,
  "ok": true,
  "count": 5,
  "by_class": {"DOWN": 2, "UP": 3},
  "tracks": [
    {"id": 3, "class": "UP", "frames": 138, "first_frame": 28,
     "last_frame": 165, "max_conf": 0.975, "best_frame": 55}
  ],
  "total_frames": 198,
  "fps": 15.0,
  "output_dir": "...",
  "summary_path": ".../track_summary.json",
  "tracked_video": ".../sweep_tracked.avi",
  "contact_sheet": ".../tracks_contact.jpg",
  "message": ""
}
```

## 4. 采集侧的约定

导航脚本不要和 `capture_photos.py` 同时控制 `/cmd_vel`。推荐：

1. 导航到目标点并停车/暂停导航；
2. 启动 `newt.py` 和 `bringup.launch`（如果还没启动）；
3. 调区域扫描产生 `sweep.avi`；
4. 恢复导航前调用 `count_video()`。

采集命令：

```bash
rosrun car_bringup newt.py
roslaunch /home/mowen/bit_ws/src/car_bringup/launch/bringup.launch

rosrun photo_capture capture_photos.py --mode scan \
  --scan-frames 8 --scan-distance 2.0 \
  --scan-speed 0.15 --scan-feedback odom \
  --scan-record-video --scan-video-fps 15 \
  --scan-label 导航采集_01
```

输出目录按 `区域扫描/<label>_<时间戳>/` 匹配即可。

## 5. 调参

- `min_frames`：默认 20（15fps 约 1.3 秒）。数量偏多就调大，偏少调小。
- `conf`：默认 0.25；漏检多可降到 0.20。
- `classes`：只统计指定类别，例如 `UP,DOWN`。
- 单帧结果不能当数量；必须看 `count` / `tracks`。

## 6. 依赖

- `ultralytics`
- `lap`（ByteTrack 需要；本机已安装）
- OpenCV / numpy

## 7. 已验证结果

01~04 四次测试总数均为 **5**；不同摆放状态下分类分别为
`UP4/DOWN1`、`UP3/DOWN2`、`DOWN5`、`UP5`，与缩略图一致。
