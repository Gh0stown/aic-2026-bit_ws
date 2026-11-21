# 视觉识别节点交接（inspection_vision）

状态：导航侧已接入初版（commit `80755cb`），**视觉节点已实现**（2026-11-21，
见 [inspection_vision_node.md](inspection_vision_node.md)）；离线/服务级验证通过，实车到点联调未做。
本文保留为接口与验收口径的交接依据。

## 1. 目标

实现一个**无 Tk 的 ROS1 节点**，提供 `/inspection_vision` 服务。导航到点、转向、停稳后调用它，节点用现有相机和模型做识别，返回：

- `result`：终端/裁判用的播报文本；
- `extra`：结构化计数（JSON 字符串），供导航做街区内汇总；
- `images`：带 YOLO/OCR 框的图片路径（GUI 的“识别图像”页显示）。

节点**不发布 `/cmd_vel`**，不控制底盘。

## 2. 接口

srv：`src/nav_demo/srv/InspectionVision.srv`（已编译）

```
string task
int32 point_id
string sub_id
string params
---
bool success
string result
string extra
string[] images
```

- `task`：`crowd / sign / trash / fire / plate / temperature / meter / ebike / light`
- `params` 可选 JSON：`request_id`、`frames`、`deadline`、`pan` 等；
- 导航侧同步调用；**ROS1 service 不可取消**，节点必须自己执行截止时间；
- 超时/暂停时导航会丢弃晚到响应，节点不能把上一站结果当本次。

## 3. 任务、模型与输出

| task | 模型/模式 | result 模板 | extra 建议 |
|---|---|---|---|
| crowd | `~/AI视觉/people.pt`（people/bad/cycle） | 社区内共有 XX 人，其中 X 街人数 X 人，发现 X 名非社区人员在 X 街 | `{"person": N, "bad": N}` |
| sign | `~/AI视觉/Signal.pt`（A/B） | 前方是禁止直行指示牌 | `{"side": "A"}` |
| trash | `trash_brest.pt`/`模型/lesetong.pt`（开/关 8 类）+ `catalog.pt`/`模型/lese.pt`（具体垃圾 36 类） | XX 垃圾桶状态为（打开/关闭），垃圾桶里的垃圾为 XX，投放（正确/错误） | `{"type": "...", "state": "open/close", "correct": true}` |
| fire | `fire.pt`（火源/fire）+ `house.pt`（1 类）裁剪 + PaddleOCR 楼宇名 | XX 大厦发现火灾隐患 X 个 | `{"building": "...", "count": N}` |
| temperature | `building_d.pt` + 红外 `mlx90640_ros.thermal_session` | XX 大厦在 X 楼处发现高温情况 | `{"building": "D", "floor": N}` |
| meter | `gauge_exp16`（`Exp16Reader` + `StabilityBatch`，见 `docs/gauge_exp16_navigation_handoff.md`） | XX 表读数为 XXXXX | `{"temp": {...}, "humidity": {...}}` |
| plate | `plate.pt`（1 类）裁剪 + `ocr_engine`（PaddleOCR/HyperLPR3） | X 号停车场车牌号为 XXXXX | `{"plate": "..."}` |
| ebike | `Cycle.pt`（UP/DOWN）+ `count_video.py` 视频流；停车区 `best.pt`（倒伏/正常） | X 街区有 X 辆电动车违停，X 街区无违停，停车区内电动车正常 X 辆，倒伏 X 辆 | 街区 `{"illegal": N}`；停车区 `{"normal": N, "fall": N}` |
| light | `light.pt` / `模型/ryg.pt`（红/黄/绿） | 红/黄/绿 | `{"color": "red/yellow/green"}` |

补充：

- **红绿灯不上报裁判**，只返回灯色，导航自己等灯（45 s 上限）；
- **指示牌**：A=正面→left，B=侧面→straight（暂定，需实测确认）；
- **crowd/ebike 街区内汇总由导航做**，节点每次只返回当前视角的计数；
- **ebike 停车区**在 17→18 平移时识别，平移默认 0.15 m/s、步长 ≤0.35 m（`photo_capture/scripts/scan.py`）；
- **仪表最后接**，先联调不报分；缺数/超时记录原因，不填 0。

## 4. 点位与 task 对应

导航侧映射在 `src/nav_demo/scripts/vision_tasks.py`，当前为：

| 点 | 任务名 | task | sub_id |
|---|---|---|---|
| 1 | 红绿灯 1 识别 | light | 红绿灯1 |
| 2 | A 街区北方识别 | crowd | A街区 |
| 4 | 禁止通行指示牌识别 | sign | A街区 |
| 4 | A 街区西方识别 | crowd | A街区 |
| 6 | A 街区南方识别 | crowd | A街区 |
| 6 | B 街区北方识别 | crowd | B街区 |
| 7 | 楼宇 A 识别 | fire | 楼宇A |
| 8 | B 街区东方识别 | crowd | B街区 |
| 8 | 楼宇 C 识别 | fire | 楼宇C |
| 8 | 红绿灯 2 识别 | light | 红绿灯2 |
| 10 | 站房识别 | meter | 站房 |
| 12 | 楼宇 D 识别 | temperature | 楼宇D |
| 13 | B 街区西方识别 | crowd | B街区 |
| 14 | 1 号停车位识别 | plate | 1号停车位 |
| 15 | 2 号停车位识别 | plate | 2号停车位 |
| 15 | 垃圾桶识别 | trash | 垃圾桶 |
| 16 | 3 号停车位识别 | plate | 3号停车位 |
| 17 | 楼宇 B 识别 | fire | 楼宇B |
| 17 | 电动车识别起始 | ebike | 停车区 |

## 5. 可参考的现有代码

- `src/photo_capture/scripts/capture_photos.py`：GUI 模式 2 实时 YOLO、3 区域扫描、5 红外、6 OCR、4/7 仪表；
- `src/photo_capture/scripts/ocr_engine.py`：PaddleOCR 中文 + HyperLPR3 车牌，支持 YOLO 先裁剪；
- `src/photo_capture/scripts/count_video.py`：`count_video(video, model=Cycle.pt, ...)` 返回唯一数量/分类/轨迹；
- `src/photo_capture/scripts/gauge_exp16.py`、`gauge_exp16_stability.py`：仪表读数与多帧稳定；
- `docs/gauge_exp16_navigation_handoff.md`：仪表接入细节；
- `docs/recognition_plan.md`：19 个识别任务、时间预算、路线分叉。

## 6. 约束

- 相机只能有一个采集进程；节点启动前不要同时开 `photo_capture` GUI；
- 节点不发布 `/cmd_vel`；
- 识别前车必须停稳（导航会用新鲜 `/odom` 判断，不能只靠定时 dwell）；
- 服务不可取消，节点必须支持截止时间、丢弃过期请求；
- `images` 必须是带框图的实际文件路径，供 GUI 显示和裁判上传；
- 视频流（ebike）需要固定平移参数，参考 `scan.py` 默认 0.15 m/s、步长 ≤0.35 m；
- 启动入口：把节点名写进 `src/nav_demo/scripts/vision_nodes.conf`，用 `start_vision.sh` 一个终端启动。

## 7. 验收

1. `rosservice call /inspection_vision` 手动调用，各 task 能返回 result/extra/images；
2. 导航 `cruise_debug.py` 到点调用，终端能看到识别结果，GUI“识别图像”页能看到带框图；
3. 用 `tools/mock_judge` 验证结束自动上报；
4. 仪表先只做单点联调，不自动报分；
5. 记录每类任务的耗时、出结果率、失败原因。

进度（2026-11-21）：

- [x] 1：离线逐 task 调通（2026 道具实拍 + 历史图，见新文档 §6）；
- [ ] 2：实车到点联调（相机取景、停稳触发、GUI 图片页）；
- [ ] 3：mock_judge 整轮上报；
- [x] 4：meter 只做单点、超时/缺数不报分；
- [x] 5：每次请求自动写 `recognitions.jsonl`（request_id/耗时/成功与否/失败原因），
      节点退出时打印按 task 汇总。

## 8. 待定（2026-11-21 更新）

- ~~具体垃圾是 36 类模型还是另有 24 类模型~~：已确认用 `trash.pt` 的 24 类，
  1–6 厨余垃圾 / 7–12 可回收物 / 13–18 其他垃圾 / 19–24 有害垃圾，
  逐类中文名见新文档 §4，已内置到节点；
- 楼宇 D 的播报名称/内容：先用 `D大楼在 X 楼处发现高温情况`；
- `Signal.pt` A/B 与 left/straight 的实测映射：仍**没有 B 摆位样本**，现场必验；
  节点已额外用 `stop.pt` 复核“有没有牌”，A/B 分不出时 `route=unknown`；
- crowd/ebike 街区内汇总后的最终播报措辞：节点只回当前视角计数
  （`extra` 带 `block/person/bad/illegal/normal/fall`），汇总仍由导航做；
- 仪表超时预算与是否报分：节点默认 15 s 截止、`reference` 不算成功、
  不自动报分；现场按实测耗时再调。
