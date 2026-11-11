# MLX90640 ESP32-S3 固件

来自 MLX90640 交付包 v1.0（2026-11-10）。本仓库仅接入 ESP32-S3，固件逻辑保留交付版本；温度解算在 ROS 主机完成，板端解算仅用于 OLED 自检。

## 接线

| 模块 | 接到 ESP32-S3-N16R8 |
|---|---|
| MLX90640 VCC/GND/SDA/SCL | 3V3/GND/GPIO8/GPIO9 |
| 可选 SSD1306 OLED VCC/GND/SDA/SCL | 3V3/GND/GPIO8/GPIO9 |
| 上位机 | 板载 CH340 的 UART USB-C 口 |

同一 I2C 总线：MLX90640 地址 0x33，OLED 地址 0x3C。传感器供电用 3.3V，线尽量短（不超过 50cm），不要带电插拔。不要用 GPIO19/20（原生 USB）、GPIO0/45/46（启动配置）、GPIO26–37（Flash/PSRAM）接 I2C。

## Arduino IDE 编译与烧录

1. 开发板管理器地址：`https://espressif.github.io/arduino-esp32/package_esp32_index.json`，安装 **esp32 by Espressif Systems**。
2. 选择 **ESP32S3 Dev Module**；Flash Size **16MB**，PSRAM **OPI PSRAM**，USB CDC On Boot **Disabled**，Upload Speed **921600**。
3. 库管理器安装 **Adafruit SSD1306**、**Adafruit GFX Library**、**Adafruit BusIO**。
4. 打开 [MLX90640_ESP32S3.ino](<MLX90640_ESP32S3.ino>)，通过板子标注 UART 的 USB-C 口上传。仓库未附预编译 ESP32 固件。

没有安装 Arduino 工具链时，ROS 主机编译不会编译或烧录此工程。不要为了试运行软件重刷板子。

## 数据与调试

UART 参数为 **460800、8N1**。插拔顺序不影响校准：主机主动发送单字节 `0x01` 请求 EEPROM，`0x02` 请求状态。打开串口可能触发 ESP32 DTR 复位。

- 上电 EEPROM：832 个字，1664 字节。
- 原始子页：每秒约 2 包，每包 1674 字节。完整 32×24 温度图需要两个不同子页，主机完整图约 1Hz。
- 状态：每秒约 1 包，11 字节。
- `frame[0..831]` 是从 0x0400 读取的 832 字 RAM（含像素及辅助数据）；`frame[832]` 单独读取控制寄存器 0x800D，`frame[833]` 为状态寄存器 0x8000 的子页位，不能连续读 834 字代替。

OLED 显示 MAX/MIN/AMB 和心跳，仅作本地自检；上电首个子页还不构成完整图。ROS 软件不会发布这种半幅结果。

协议、编译和 ROS 使用见[测温模块说明](<../../../src/mlx90640_ros/README.md>)。请明确确认测温串口，不要把示例端口直接用于底盘、雷达或 IMU。
