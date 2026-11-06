# bit_ws 工作区约定

本文件位于仓库根目录，每次对话会自动加载。开始任何任务前请先遵守下面的规则。

## 一、必须遵守：改动随时提交并推送 GitHub

本仓库已经配置好 SSH，远端和分支如下：

- `origin` = `git@github.com:Gh0stown/aic-2026-bit_ws.git`
- 主分支 = `main`

**任何**文件改动（新增、修改、删除）完成后，都要立即提交并推送到远端，不要等用户提醒：

```bash
git status                 # 确认改了哪些文件
git diff                   # 提交前检查改动内容
git add <本次改动的文件>
git commit -m "中文说明改了什么"
git push origin main
git status -sb             # 确认显示 ## main...origin/main，且没有 ahead/behind
```

补充约定：

- 提交信息用中文，写清楚改了什么、为什么改。
- 只提交本次任务相关的文件，不要把无关改动混进同一个提交。
- `build/`、`devel/`、`install/`、`logs/`、`*.pyc` 已在 `.gitignore` 中，不要提交。
- 地图文件 `src/nav_demo/map/*.pgm` / `*.yaml` 属于建图成果，按用户要求一并入库（文件偏大也照常提交）。
- 桌面上的 `~/桌面/readme.txt` 等工作区之外的文件不在仓库内，只能在回复里告知用户，不要强行复制进仓库。
- 如果 push 失败（网络、权限、冲突），要把具体报错告诉用户，不要静默跳过；冲突时先 `git pull --rebase` 再推送。
- 唯一例外：用户当次明确说“先不要提交/不要上传”时，才不提交，并在回复里说明改动仍未入库。

## 二、工程速览（ROS1 Noetic catkin 工作区）

- 机器人：两轮差速小车，车宽约 30cm；场地道路宽度约 60cm。
- 传感器：镭神 lslidar（`/scan`，frame `laser_link`）、WIT IMU（`/wit/imu`）。
- 定位链：轮式里程计 `/odom_raw` + IMU 经 `robot_localization` EKF 融合，发布 `odom -> base_footprint`。
- 建图：
  - gmapping：`src/gmapping.launch`、`src/car_bringup/launch/gmapping.launch`
  - karto：`src/car_bringup/launch/karto.launch`（主文件，参数在这里改），入口 `src/karto.launch`
- 导航：`src/nav777.launch`，默认加载基线地图 `src/nav_demo/map/mymap_20261106_1916.yaml`（karto，0.025m），可用 `map:=` 覆盖。
- 参数目录：`src/nav_demo/param/`（代价地图、DWA、全局规划器）。
- 保存地图：`src/nav_demo/scripts/save_map.sh`，按时间戳命名，不覆盖旧图。
- 使用说明速查：`~/桌面/readme.txt`。

## 三、改动前的习惯

- 改 `launch` / `yaml` 后，用 `roslaunch --nodes <pkg> <file>.launch` 或 `python3 -c "import xml.etree.ElementTree as ET; ET.parse(...)"` 做一次语法校验。
- 涉及坐标系的改动（`odom_frame`、`base_frame`、`map_frame`）要和 EKF、URDF、雷达驱动保持一致：`odom`、`base_footprint`、`map`，不要带前导斜杠。
- 大范围改参数前先说明预期效果和风险，改完把关键数值写进回复。
