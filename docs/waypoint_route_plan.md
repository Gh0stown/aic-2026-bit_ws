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
| `src/nav_demo/scripts/snap_waypoints.py` | 把录好的点投影到预设路线上，输出 `waypoints_snapped.yaml`；超过 `--max-dist` 的点保持原样并告警 |
| `src/nav_demo/config/safe_routes.yaml` | 预设路线文件（目前是空模板，等真实路线坐标） |

`record_waypoint.py` 已用模拟 TF 做过功能验证（x/y/yaw 正确），尚未实车测试。
`snap_waypoints.py` 已用合成数据验证投影逻辑。

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
