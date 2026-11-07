# bit_ws 工作区约定

本文件位于仓库根目录，每次对话会自动加载。开始任何任务前请先遵守下面的规则。

## 一、必须遵守：改动先检查，再询问是否上传，不自动推送

本仓库已经配置好 SSH，远端和分支如下：

- `origin` = `git@github.com:Gh0stown/aic-2026-bit_ws.git`
- 主分支 = `main`

**不要**在每次改动后自动 commit / push，避免频繁上传污染仓库。正确流程是：

1. 每次任务做完、准备给最终回复前，执行 `git status` / `git diff` 检查有没有变化。
2. 如果有变化，在回复里汇总改动清单（改了哪些文件、大致内容、有什么影响），然后**询问用户是否需要上传**。
3. 只有用户明确同意后，才执行提交和推送：

```bash
git add <本次改动的文件>
git commit -m "中文说明改了什么"
git push origin main
git status -sb             # 确认显示 ## main...origin/main，且没有 ahead/behind
```

4. 用户说“先不用/暂不上传”时，保持改动在本地不提交，并在回复里说明仍未入库、下次可以再问。
5. 同一次对话里的多个小改动可以攒到最后一起问、一起提交，不必改一次问一次。

补充约定：

- 提交信息用中文，写清楚改了什么、为什么改。
- 只提交本次任务相关的文件，不要把无关改动混进同一个提交。
- `build/`、`devel/`、`install/`、`logs/`、`*.pyc` 已在 `.gitignore` 中，不要提交。
- 地图文件 `src/nav_demo/map/*.pgm` / `*.yaml` 属于建图成果，用户同意上传时一并提交（文件偏大也照常提交）。
- 桌面上的 `~/桌面/readme.txt` 等工作区之外的文件不在仓库内，只能在回复里告知用户，不要强行复制进仓库。
- 如果 push 失败（网络、权限、冲突），要把具体报错告诉用户，不要静默跳过；冲突时先 `git pull --rebase` 再推送。

## 二、工程速览（ROS1 Noetic catkin 工作区）

- 机器人：两轮差速小车，车宽约 30cm；场地道路宽度约 60cm。
- 传感器：镭神 lslidar（`/scan`，frame `laser_link`）、WIT IMU（`/wit/imu`）。
- 定位链：轮式里程计 `/odom_raw` + IMU 经 `robot_localization` EKF 融合，发布 `odom -> base_footprint`。
- 建图：
  - gmapping：`src/gmapping.launch`、`src/car_bringup/launch/gmapping.launch`
  - karto：`src/car_bringup/launch/karto.launch`（主文件，参数在这里改），入口 `src/karto.launch`
- 导航：`src/nav777.launch`，默认加载基线地图 `src/nav_demo/map/mymap_20261106_1916_clean.yaml`（karto，0.025m），可用 `map:=` 覆盖。
- 场地与航点：场地 4.4m × 4.4m，pgm 里只有四周墙体、道路（60cm）没有内部特征；航点记录 `src/nav_demo/scripts/record_waypoint.py`，巡检点位图形化记录 `src/nav_demo/scripts/waypoint_gui.py`（预设点位 `src/nav_demo/config/inspection_points.yaml`，坐标输出 `src/nav_demo/waypoints/inspection_waypoints.yaml`），收敛到预设路线 `src/nav_demo/scripts/snap_waypoints.py`（路线文件 `src/nav_demo/config/safe_routes.yaml`，目前为空）。方案说明与暂缓原因见 `docs/waypoint_route_plan.md`。
- 参数目录：`src/nav_demo/param/`（代价地图、DWA、全局规划器）。
- 保存地图：`src/nav_demo/scripts/save_map.sh`，按时间戳命名，不覆盖旧图。
- 使用说明速查：`~/桌面/readme.txt`。

## 三、改动前的习惯

- 改 `launch` / `yaml` 后，用 `roslaunch --nodes <pkg> <file>.launch` 或 `python3 -c "import xml.etree.ElementTree as ET; ET.parse(...)"` 做一次语法校验。
- 涉及坐标系的改动（`odom_frame`、`base_frame`、`map_frame`）要和 EKF、URDF、雷达驱动保持一致：`odom`、`base_footprint`、`map`，不要带前导斜杠。
- 大范围改参数前先说明预期效果和风险，改完把关键数值写进回复。
