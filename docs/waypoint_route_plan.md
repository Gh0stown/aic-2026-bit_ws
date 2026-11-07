# 航点 / 路线方案（暂缓推进）

> 状态：2026-11-07 记录，方向保留，暂不推进。先用手动脚本标点。

## 场地事实

- 场地 pgm 地图就是 `src/nav_demo/map/mymap_20261106_1916_clean.pgm`
  （0.025 m/格，图像 289×242 格，场地本身是 **4.4 m × 4.4 m** 的方形区域）。
- 地图上**只有四周墙体**，场地内部道路没有任何障碍物、标注或可匹配特征。
- 场地道路宽约 **60 cm**，小车宽约 **30 cm**（理论每侧余量 15 cm，
  再扣掉定位误差 ±3~5 cm 和墙线误差，实际更紧）。
- 比赛给两张路线图（左 / 右），**点位相同、经过顺序不同**；
  示意图右上角是起点，对应 pgm 地图左下角（两者相差 180°）。

## 已经写好的东西

| 文件 | 作用 |
|---|---|
| `src/nav_demo/scripts/record_waypoint.py` | 遥控到满意位置后按回车（或调 `/record_waypoint` 服务）记录 map 坐标下的 x/y/yaw，写入 `nav_demo/waypoints/waypoints.yaml` |
| `src/nav_demo/scripts/waypoint_gui.py` | 巡检点位图形化记录：左侧 20 个预设点位，右侧地图实时显示位姿，选中点位后一键记录；支持自定义点位；输出 `nav_demo/waypoints/inspection_waypoints.yaml` |
| `src/nav_demo/config/inspection_points.yaml` | 巡检点位预设（0~19 号，来源《巡检点位记录.md》） |
| `src/nav_demo/scripts/snap_waypoints.py` | 把录好的点投影到预设路线上，输出 `waypoints_snapped.yaml`；超过 `--max-dist` 的点保持原样并告警 |
| `src/nav_demo/config/safe_routes.yaml` | 预设路线文件（目前是空模板，等真实路线坐标） |

`record_waypoint.py` 已用模拟 TF 做过功能验证（x/y/yaw 正确），尚未实车测试。
`waypoint_gui.py` 已用模拟 TF 自测：建界面、记录 0 号点、写 YAML 均正常，尚未实车测试。
`snap_waypoints.py` 已用合成数据验证投影逻辑。

### 巡检点位图形化记录

```bash
roslaunch ~/bit_ws/src/nav777.launch          # 地图 + AMCL
roslaunch mbot_teleop mbot_teleop.launch      # 遥控
rosrun nav_demo waypoint_gui.py               # 图形界面
```

- 左侧选中点位 → 点“记录当前位姿”；已记录的点状态列显示“已记”；
- 预设点位括号里的并列任务（6/7/8/15/17）如果实测发现不是同一位置，
  用底部“预设里没有的点位”输入名字单独记录，例如 `6-B街区北方`；
- 0 号与 19 号是同一位置，两个都记一遍即可（或记完手动复制坐标）；
- 输出文件的每条记录带 id / name / x / y / yaw / yaw_deg / 时间戳。

## 暂缓的方案：把标点收敛到预设安全路线

思路：录完点后，把每个点投影到提前在 map 坐标系里画好的路线（车道中线）上，
避免手动标点太靠墙导致 move_base 目标无效、原地打转。

暂缓原因：

1. 场地道路没有地图特征，路线只能按比赛示意图手写成坐标折线，工作量和误差都不小；
2. 先手动标点看效果，如果够用就不折腾。

## 恢复推进时要做的

1. 按 map 坐标在 `safe_routes.yaml` 里写 `route_left` / `route_right`
   （两条路线点位相同、顺序不同）；
2. `rosrun nav_demo snap_waypoints.py --route route_left`，在 rviz 里核对吸附结果；
3. 如果需要沿固定路线行驶（而不仅是把目标点吸到安全位置），再加一个
   “按路线中间点依次发目标”的节点；
4. 定位风险：4.4×4.4 场地只有四周墙，AMCL 可匹配的特征很少，
   建议放几根反光柱/纸箱当临时 landmark（建图时一起扫进去），否则长距离导航会漂。

## 备选做法（如果连手动标点也不稳）

- 用 `record_waypoint.py` 的 `--yaw route` 选项把朝向也对齐到路线切线；
- 把点直接吸到“墙内 20 cm 的等距线”上（对这张只有墙的地图，可以用距离变换自动生成中线）。

## 巡航脚本（已实现：cruise_waypoints.py）

按路线顺序自动跑点位：普通点用 move_base 开到 `(x, y, 到达朝向)`，识别点再原地转向
`tasks` 里的每个朝向；**14→18 段不用 move_base**，改成参考 photo_capture 的麦轮平移：
只发 `linear.y`、0.06 m/s、`/odom` 闭环、到距离停，车头保持 180°；到 15/17 再原地
转向识别，识别完转回 180° 继续平移。

```bash
# 只看计划（不动车）
rosrun nav_demo cruise_waypoints.py --route fast --dry-run

# 实跑，每个识别朝向停 3 秒
rosrun nav_demo cruise_waypoints.py --route fast --dwell 3
```

- 路线顺序：`src/nav_demo/config/inspection_routes.yaml`（left / right / fast）
- 点位与识别朝向：`src/nav_demo/waypoints/inspection_waypoints.yaml`
- 平移相关参数：`--strafe-ids 14,15,16,17,18`、`--strafe-speed 0.06`、`--no-strafe`
- 退出（Ctrl-C）会取消 move_base 目标，避免脚本停了车还在跑
- 配套：`newt.py` 以 20Hz 重发最后速度、0.5s 无指令自动归零、退出发零速
