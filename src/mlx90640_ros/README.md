# MLX90640：ESP32-S3 串口热成像

独立 ROS1 Noetic/catkin 包。MLX90640 → ESP32-S3 → UART 460800 → 厂商 C API 解算 → 32×24 温度图。只移植 ESP32 方案，不包含其他 MCU 工程；不控制底盘、不自动加入导航，也不实现楼层判定或裁判上传。

## 1. 依赖和编译

Ubuntu 20.04 / Python 3.8 / ROS Noetic：

```bash
sudo apt install build-essential python3-serial python3-numpy python3-opencv python3-nose
cd ~/bit_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg mlx90640_ros -j2
source devel/setup.bash
```

动态库由源码编译为 `libmlx90640.so`，通过 source 后的 `LD_LIBRARY_PATH` 找到，也可显式指定库路径。不要使用交付 ZIP 内的 x86-64 SO 或 Windows DLL；ARM64 应在目标机重新编译。本包 C 包装的校准参数是进程全局状态，**每个进程只使用一个 Decoder/一台传感器**；多传感器应分进程。

ESP32 接线、Arduino 库、开发板选项和烧录见[固件说明](<../../firmware/mlx90640/MLX90640_ESP32S3/README.md>)。ROS 编译不编译或烧录固件。串口程序和 Arduino 串口监视器不能同时占用设备。

## 2. 无硬件先验证

终端 1：

```bash
roscore
```

终端 2（已 source 本工作区）：

```bash
roslaunch mlx90640_ros replay.launch loop:=true
```

终端 3：

```bash
rostopic hz /mlx90640/temperature_image
rostopic echo /mlx90640/ambient_temperature
# 如已安装 rqt_image_view，可查看伪彩热图
rosrun rqt_image_view rqt_image_view /mlx90640/thermal_image
```

默认回放[真实抓包](<test/data/capture.bin>)；`loop:=false` 时输出两张完整图后结束。抓包含四个子页，回放默认每秒 2 个子页，完整图约 1Hz。两张新合成完整图的 MAX/MIN 分别约为 33.05/21.29℃、32.85/21.38℃，AMB 均约 29.56℃；已与交付包原始 SO 对照一致。交付说明中的截图统计来自滑动子页缓冲，不是这两对独立新图。抓包不含原始时间戳，板端时间按标称 500ms 子页间隔合成，`replay_rate` 只控制播放墙钟速度；它验证算法/接口，不代表当前实物温度，也不证明 ESP32 已接通。

## 3. ESP32 实机启动

**先确认设备身份，再指定端口；程序不自动探测，也没有默认串口。**

```bash
ls -l /dev/serial/by-id/
# 将下面占位符换成确认属于 ESP32 UART USB-C 的设备路径
roslaunch mlx90640_ros mlx90640.launch port:=/dev/serial/by-id/你的ESP32设备
```

不能把 `/dev/carserial`、`/dev/imu`、`/dev/lslidar` 或未经确认的 `/dev/ttyUSB0` 作为测温设备。CH340 的 by-id 也可能不唯一；必要时用拔插对照确认，并建立针对实际硬件的稳定设备名。

权限不足时：

```bash
sudo usermod -aG dialout "$USER"
# 注销并重新登录后生效
```

参数采用节点私有参数，可用 launch 同名参数覆盖（以 launch 暴露项为准），或在自定义 launch 中设定：

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | 必填 | 明确指定 ESP32 串口 |
| `baud` | 460800 | 8N1，与固件一致 |
| `library_path` | 空 | 空值通过系统库搜索路径加载 |
| `frame_id` | `mlx90640_link` | 不带前导斜杠，不自动发布 TF |
| `emissivity` | 0.95 | 发射率，范围 (0,1] |
| `tr_shift` | 8.0 | 反射温度使用 Ta−tr_shift |
| `bad_pixels` | false | 厂商 broken/outlier 坏点修正 |
| `scale` | 12 | 伪彩图放大倍数，不影响温度矩阵 |
| `tmin` / `tmax` | 自动 | 显示色标，不改变真实温度 |
| `capture_file` | 空 | 非空则回放，不打开串口 |
| `replay_rate` | 2.0 | 回放子页/秒，不是完整图/秒 |
| `loop` | false | 回放是否循环 |
| `floor_map` | 空 | 行号→楼层标定表，如 `1F:16-23,2F:8-15,3F:0-7`（行含端点、不可重叠） |
| `hotspot_delta` | 2.0 | 比峰值低不超过该 ℃ 的连通像素算同一热点 |
| `roi` | 空 | 只在 `r0,c0,r1,c1`（半开）内找热点，用于排除背景 |

## 4. ROS 数据接口

默认命名空间为 `/mlx90640`，均不锁存：

| 相对话题 | 类型 | 含义 |
|---|---|---|
| `temperature_image` | `sensor_msgs/Image` | width=32，height=24，encoding=`32FC1`，step=128；float32 摄氏度，行优先 |
| `thermal_image` | `sensor_msgs/Image` | `bgr8` 伪彩、热点十字与绿色热点框，32×scale 列、24×scale+42 行（底部文字区） |
| `ambient_temperature` | `sensor_msgs/Temperature` | 传感器内部环境温度，非目标/中心/最高温；variance=0 表示未知 |
| `hotspot` | `std_msgs/String` | JSON：峰值温度、亚像素行列、整像素峰值、像素框、面积、平均温度、楼层 |

同批消息使用相同 ROS 主机接收时间与 frame_id，不把板端毫秒数冒充 ROS 时间。温度图单位特意为摄氏度（不是 K）；伪彩图只供观察，算法应读取 `temperature_image`。

图像坐标 row=0..23、col=0..31，保持传感器原始方向，不自动旋转/镜像；安装方向、视场和楼层映射需现场标定。未标定安装位姿，因此不发布 `base_link → mlx90640_link`，也不提供虚假的 CameraInfo。

### 4.1 热点与楼层（`hotspot`）

`hotspot` 每帧随温度图一起发布，示例：

```json
{"peak_c": 33.046, "row": 16.0, "col": 16.0, "row_index": 16, "col_index": 16,
 "box": [16, 14, 19, 17], "area": 9, "mean_c": 31.4, "floor": "2F"}
```

- `peak_c` 是峰值整像素的真实温度；`row`/`col` 是 3×3 二次曲面拟合的亚像素中心（边界或非极大时退回整像素）。
- `box` 是含端点的像素框 `[r0, c0, r1, c1]`，只圈比峰值低 `hotspot_delta` 以内的 8 连通区域，伪彩图上以绿框画出。
- `floor` 由 `floor_map` 按 **亚像素行号** 查表；落在表外时取最近楼层，因此标定偏差不会让答案变空。不配 `floor_map` 时为空字符串。
- 32×24 没有深度，`hotspot` 只回答“热像里的哪个像素最热”，**不是**世界坐标。要落到实物/`map` 坐标，需另做相机外参、距离（雷达或已知平面）或热像↔可见光单应标定。

固件约每秒发 2 个棋盘子页，每个只更新一半像素。本包收齐两个不同的新子页才发布完整图（约 1Hz）；重复子页、启动首个半图不会发布。每次输出是独立副本，不复用上一张图未更新的一半。

## 5. 独立查看器（不需要 ROS master）

仍需先 source 工作区以找到 Python 模块和编译库：

```bash
# 离线查看
rosrun mlx90640_ros mlx90640_viewer.py --capture "$(rospack find mlx90640_ros)/test/data/capture.bin"
# 实机查看，替换为确认后的端口
rosrun mlx90640_ros mlx90640_viewer.py --port /dev/serial/by-id/你的ESP32设备
# SSH 无桌面回放并保存每张图
rosrun mlx90640_ros mlx90640_viewer.py --capture "$(rospack find mlx90640_ros)/test/data/capture.bin" --headless --save-every 1
```

按 `q` 退出、`s` 保存、`r` 重新请求 EEPROM。默认输出目录 `~/Pictures/mlx90640/`；可用 `--save-dir` 覆盖。更多参数见 `--help`。保存失败会报告错误，不假报成功。

### 5.1 给采集 GUI 的会话接口

`mlx90640_ros.thermal_session.ThermalSession` 在后台线程读串口或回放，向界面暴露
`latest()`（已画框的 BGR 图 + 热点 + 楼层）、`record_floor()`（现场标定）和
`save_snapshot()`（PNG + JSON），不依赖 tkinter、PIL 或 ROS。接入位置和验收清单见
[采集 GUI 接入说明](<../../../docs/thermal_gui_integration.md>)。
一个进程只能有一个 `ThermalSession`，也不要和 `roslaunch mlx90640_ros` 抢同一串口。

## 6. 协议、恢复与限制

```
AA 55 | TYPE:u8 | SEQ:u8 | LEN:u16LE | PAYLOAD | CRC:u16LE
```

CRC16-CCITT-FALSE（poly=0x1021、init=0xFFFF），覆盖 TYPE 至 PAYLOAD 末尾。

- TYPE 1 EEPROM：1664 bytes，832 个小端 uint16。
- TYPE 2 FRAME：1674 bytes，`ms:u32LE + subpage:u8 + reserved:u8 + 834*u16LE`。
- TYPE 3 STATUS：11 bytes，`ms:u32LE + frames:u32LE + errors:u16LE + reserved:u8`。
- 原始 words[0..831] 含像素及辅助 RAM；words[832] 是单独读取的控制寄存器；words[833] 是 0/1 子页号，与包头字段一致。
- SEQ 是所有包共用的 8 位序号，255→0 正常回绕，不据此推断温度帧数量。

主机打开串口后发送单字节 0x01 请求校准；未收到 EEPROM 每秒重试。ESP32 打开串口可能 DTR 复位，等待校准和两种子页后才出图。串口异常关闭、清空缓存和温度状态，每 2 秒重连；连续 3 秒无有效温度帧会重新请求校准，不发布旧图或零值。板端毫秒数正常 UINT32 回绕不当作重启；真正回退则重新等待校准。

错误长度/CRC、未知类型、噪声后重新同步；校准失败、子页非法、非有限温度不发布。无桌面 ROS 默认不调用窗口。测温超时不会移动机器人；本模块没有 `/cmd_vel` 发布或导航依赖。

**楼宇 D 任务尚未接入**：热点只表示温度最高像素，不等于异常区域或楼层。比赛前需要发射率/距离检查、相机安装方向标定、楼层 ROI、背景排除和异常阈值，不能用本模块默认色标直接判高温。

## 7. 测试

```bash
cd ~/bit_ws
catkin_make run_tests_mlx90640_ros -j2
catkin_test_results build/test_results/mlx90640_ros
# 也可直接执行核心测试
MLX90640_LIBRARY="$PWD/devel/lib/libmlx90640.so" \
  PYTHONPATH="$PWD/src/mlx90640_ros/src:$PYTHONPATH" \
  python3 -m unittest discover -s src/mlx90640_ros/test -v
```

覆盖 UART 分段/粘包/CRC/长度/重同步、校准与子页状态、坏点修复、无效温度、设备计数回绕、真实抓包回归；附加测试验证消息转换及模拟串口恢复。

端到端 ROS 回放测试默认跳过，显式启用时会启动并清理自己的本地随机端口 ROS master，不使用现有 11311、不访问实机串口：

```bash
MLX90640_ROS_INTEGRATION=1 python3 -m unittest discover \
  -s src/mlx90640_ros/test -p test_ros_replay.py -v
```

初次移植验证（2026-11-11，Ubuntu 20.04 / x86-64 / Noetic）：catkin 源码编译通过；35 项离线测试通过，端到端 ROS 回放测试另行启用并通过；独立 install 空间库查找、rosrun 查看器、launch 解析及两张 PNG 输出通过。解算数值与交付包原 SO 一致。

实机串口和 Arduino 固件编译/烧录须在明确硬件身份与工具链后单独验收。本仓库初次移植未打开任何真实串口，未烧录固件，未修改原固件的采集逻辑。

[来源与许可证](<THIRD_PARTY.md>)；[交付参考热图](<test/data/示例热图.png>)。
