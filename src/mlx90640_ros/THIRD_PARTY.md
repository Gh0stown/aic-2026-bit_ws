# 来源与第三方代码

移植来源为工作区提供的 MLX90640 交付包 v1.0（2026-11-10）。本次仅采用 ESP32-S3 方案，不纳入其他 MCU 工程、Windows DLL 或预编译 Linux SO。

- [MLX90640_API.c](<vendor/MLX90640_API.c>)、[MLX90640_API.h](<vendor/MLX90640_API.h>)、[MLX90640_I2C_Driver.h](<vendor/MLX90640_I2C_Driver.h>)：保留交付版本及原版权声明；Melexis N.V.，Apache License 2.0，见[许可证](<LICENSE-APACHE-2.0>)。
- [mlx_wrap.c](<vendor/mlx_wrap.c>)：来自交付包上位机包装，移植时改进坏点处理；具体变更写在源码注释中。
- [capture.bin](<test/data/capture.bin>)、[示例热图](<test/data/示例热图.png>)：交付包中的真实测试资料；抓包容器含 EEPROM 和四个子页，不包含 UART 帧头/CRC、原始采集时间或设备来源标识。它用于验证上位机算法，不能替代本机 ESP32 实物验收。
- ESP32 工程保留交付源码，固件说明已改为本仓库 ESP32 单方案操作入口。固件内的 Melexis 源码保留原始许可声明。
- 新编写的 ROS 接口、协议适配、工具和测试使用 [BSD-3-Clause](<LICENSE>)。交付包装及资料若无独立授权声明，本文件不替其重新授权；对外再分发前由交付方确认权限。

计算算法沿用厂商实现，温度结果受发射率、反射温度、目标距离、视场覆盖和硬件校准影响，非计量认证设备。
