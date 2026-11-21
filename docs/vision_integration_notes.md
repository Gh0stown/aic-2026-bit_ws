# 视觉接入笔记（2026-11-21）

## 统一服务

`/inspection_vision`（`nav_demo/InspectionVision.srv`）

```
string task       # crowd/sign/trash/fire/plate/temperature/meter/ebike/light
int32  point_id
string sub_id
string params
---
bool success
string result
string extra
string[] images
```

## 任务与播报

- **crowd**：社区内共有 XX 人，其中 X 街人数 X 人，发现 X 名非社区人员在 X 街
  - **街区内汇总**：A 街（2/4/6 视角）人数相加、B 街（6/8/13 视角）人数相加，最终汇总一条上报；
  - 非社区人员（bad）按街汇总。
- **sign**：前方是禁止直行指示牌
- **trash**：XX 垃圾桶状态为（打开/关闭），垃圾桶里的垃圾为 XX，投放（正确/错误）
- **fire**：XX 大厦发现火灾隐患 X 个
  - **楼宇名 XX 由 OCR 识别**（house.pt 裁剪 + PaddleOCR），不是写死。
- **temperature**：XX 大厦在 X 楼处发现高温情况
  - 对应楼宇 D；楼宇 D 似乎没有名称，具体播报内容待用户再确认。
- **meter**：XX 表读数为 XXXXX（站房，模式 7，最后接）
- **plate**：X 号停车场车牌号为 XXXXX（14/15/16，plate.pt 裁剪 + OCR）
- **ebike**：X 街区有 X 辆电动车违停，X 街区无违停，停车区内电动车正常 X 辆，倒伏 X 辆
  - **街区内汇总**：街区违停按 A/B 街汇总（2/13 号点）；停车区正常/倒伏在 17→18 平移时单独出；
  - 终端分开显示；裁判系统先按分开上报。
- **light**：红绿灯只在终端输出，不上报裁判；1/8 号点，最后判断，绿灯通过。

## 路线

- 4 号点检测到禁止立牌**正着放 → left**，否则 → **straight**；
- 映射暂定：**A = 正面 → left，B = 侧面 → straight**（待实测确认）；
- 兜底：5 号前没看到牌按默认路线，进入 6/13 前再扫。

## 时间

- 导航执行约 2m41；识别 + 等灯后 5 分钟预计够用；
- 红绿灯等待：1 号 8~30 s，8 号 12~40 s；超时策略待定。

## meter（仪表，最后接）

来源：`docs/gauge_exp16_navigation_handoff.md`，GUI 模式 7（`--mode gauge-exp16`）。

- 识别链路：`Exp16Reader`（YOLOv8n 裁 `wendu`/`shidu` + OpenCV 读数）+ `StabilityBatch`（10 帧起，最多 20 帧，主簇 ≥5 帧且 ≥70%，前后半段各 ≥2 帧；`reference` 不算 `ok`）；
- 模型 `best.pt` 不在 Git，部署用交接包文件，SHA-256 `5415e9ab...d824cc3`，不要用旧 `Pose` 权重；
- 点位：10 号 `(2.484, 3.44)`，任务「站房识别」-90°，先现场确认取景；
- 节点：先做无 Tk 的 ROS1 识别节点，复用 `Exp16Reader`/`StabilityBatch`；相机保持单进程，不发布 `/cmd_vel`；
- 服务入口：沿用本 srv 的 `task=meter`，`params` 传 `request_id`/停稳后最早取帧/截止时间；`result` 放播报文本，`extra` 放结构化 JSON（温湿度各自 `value/status/valid_count/inlier_count`、耗时、日志路径），`images` 放代表帧；
- 巡航侧：到点→转向→**用新鲜 `/odom` 确认停稳**→触发→结果或截止；ROS1 service 不可取消，超时/暂停时丢弃旧响应，识别端自己执行截止；先联调不报分，缺数/超时记录原因，不填 0、不拿部分冒充完整。

## 待办/待定

1. 楼宇 D 的具体播报内容；
2. ebike 裁判侧是否合并；
3. crowd 最终汇总时机；
4. `Signal.pt` A/B ↔ left/straight 映射；
5. GUI：裁判 IP/队名、连通测试、自动上报、手动最小化；
6. 视觉节点启动：`vision_nodes.conf` 填新节点名，一个终端。
