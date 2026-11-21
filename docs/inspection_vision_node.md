# inspection_vision 视觉识别节点（实现说明）

状态：**已实现，离线/服务级验证通过；实车到点联调未做**（2026-11-21）。
对应交接文档：[vision_node_handoff.md](vision_node_handoff.md)。

## 1. 文件与入口

| 文件 | 作用 |
|---|---|
| `src/photo_capture/scripts/inspection_vision.py` | ROS1 节点入口：`/inspection_vision` 服务、`--self-test` 离线自测 |
| `src/photo_capture/scripts/iv_common.py` | 配置、帧源、模型缓存、画框（PIL 中文）、结果落盘、截止时间 |
| `src/photo_capture/scripts/iv_service.py` | task 分发、params 解析、超时/异常处理、日志 |
| `src/photo_capture/scripts/iv_tasks_detect.py` | crowd / sign / light |
| `src/photo_capture/scripts/iv_tasks_trash.py` | trash |
| `src/photo_capture/scripts/iv_tasks_ebike.py` | ebike |
| `src/photo_capture/scripts/iv_tasks_ocr.py` | fire / plate |
| `src/photo_capture/scripts/iv_tasks_thermal.py` | temperature |
| `src/photo_capture/scripts/iv_tasks_meter.py` | meter |
| `src/photo_capture/config/inspection_vision.yaml` | 模型路径、阈值、超时、点位默认名、红外楼层表等 |
| `src/photo_capture/tests/test_inspection_vision.py` | 24 个离线单测（不连 ROS/相机/真实权重） |

启动配置：`src/nav_demo/scripts/vision_nodes.conf`（已填 `inspection_vision system`），
由 `src/nav_demo/scripts/start_vision.sh` 一个终端拉起。

节点**不发布 `/cmd_vel`**，不控制底盘；只读相机，返回文本/结构化 JSON/带框图路径。

## 2. 服务接口

`nav_demo/InspectionVision.srv`（已编译）：

```
string task
int32  point_id
string sub_id
string params
---
bool   success
string result
string extra       # JSON 字符串（ensure_ascii=False）
string[] images    # 带框图 / OCR 图 / 热像图的实际文件路径
```

`params` 是可选的 JSON 对象，节点认这些键（未知键忽略）：

| 键 | 含义 |
|---|---|
| `request_id` | 请求 ID，原样回填到 `extra.request_id`；不传自动生成 |
| `deadline` / `timeout` | 本次允许的总秒数（相对值）；不传用 `config.timeouts` |
| `deadline_unix` | 绝对 Unix 时间戳，和 `deadline` 取更早者 |
| `frames` | 覆盖默认连拍帧数 |
| `conf` | 覆盖本任务的置信度阈值（现场调参不用改 YAML） |
| `pan` / `duration` / `fps` | ebike 停车区平移录像（见 §4） |
| `mode` | ebike 强制 `street` / `parking` |
| `ocr_mode` | plate 的 OCR 模式（`generic` / `hyperlpr`，沙盘牌必须 generic） |
| `street_model` | ebike 街区模型（`crowd` / `best` / `cycle`） |

语义约定：

- 服务回调**串行执行**（节点内部锁），一次只跑一个识别；ROS1 service 不可取消，
  所以**节点自己执行截止时间**：到点返回 `success=false`、`extra.error` 写原因，
  不把上次结果当本次（每次请求都重新取帧，`FrameSource.grab` 先丢缓冲帧）。
- 只有 `success=true` 的结果才适合上报裁判；失败时 `result` 可能是“未识别/超时”等
  提示文本，`extra.error` 给具体原因。**缺数不填 0**。
- `images` 里只有实际存在的 JPEG/PNG；GUI“识别图像（带框）”页直接读这些路径。
- 每次请求追加一行 JSONL：`<输出根目录>/<启动时间戳>/recognitions.jsonl`，
  含 `request_id/point/task/sub_id/ok/result/extra/images/models/elapsed/error`。

## 3. 任务一览

| task | 模型 | result 示例 | extra 关键字段 | 默认超时 / 帧数 |
|---|---|---|---|---|
| crowd | `people.pt`（people/bad/cycle） | `A 街区当前视角：社区人员 3 人，非社区人员 1 人` | `person` `bad` `block` `per_frame` | 6 s / 2 |
| sign | `Signal.pt`（A/B）+ `stop.pt` 复核 | `前方是禁止直行指示牌` | `side` `route` `votes` `confirm_hit` | 6 s / 3 |
| light | `light.pt`（红灯/黄灯/绿灯） | `红灯`（短词，导航 `is_green()` 解析） | `color` `text` `votes` | 3 s / 3 |
| trash | `trash_brest.pt`（4 类 × 开/关）+ `trash.pt`（24 类） | `可回收物垃圾桶状态为关闭，垃圾桶里的垃圾为塑料瓶，投放正确；…` | `bins[]`（`type/category/state/item/correct`）`detected/expected` | 8 s / 2 |
| fire | `fire.pt`（火源）+ `house.pt` 裁名牌 + PaddleOCR | `A大厦发现火灾隐患 2 个` | `building` `count` `building_source` | 8 s / 2 |
| plate | `plate.pt` + PaddleOCR（`PlateOcr`） | `1号停车场车牌号为 学R00136` | `parking` `plate` `ocr_mode` | 8 s / 2 |
| temperature | MLX90640 `thermal_session` + 楼层表 | `D大楼在 3 楼处发现高温情况` | `building` `floor` `peak_c` `floor_table` | 8 s / 2（红外取 3 帧） |
| meter | `gauge_exp16.pt` + `Exp16Reader` + `StabilityBatch` | `温度表读数为 15 度。湿度表度数为 42%。` | `temperature/humidity{value,status,valid_count,inlier_count}` | 15 s / 10–20 |
| ebike | 街区 `people.pt` cycle；停车区 `Cycle.pt`（UP/DOWN）或平移视频 `count_video` | 街区：`A 街区有 1 辆电动车违停`；停车区：`停车区内电动车正常 3 辆，倒伏 0 辆` | 街区 `illegal/block`；停车区 `normal/fall` | 8 s / 2；平移 150 s |

说明：

- **crowd / ebike 街区内汇总由导航做**：节点每次只返回当前视角计数，不做多视角相加
  （`extra` 已带 `block`/`person`/`bad`/`illegal` 供导航汇总）。
- 红绿灯结果**不上报裁判**，只给导航等灯用。
- 仪表先只做单点联调、不自动报分；`reference` 级结果不算成功（`success=false`）。
- 停车区平移：节点只录像/计数，**平移仍由导航完成**；节点不发速度指令。

## 4. 垃圾 24 类映射（2026-11 用户确认）

`trash.pt` 的 24 个类别名是纯数字 `"1".."24"`，节点内置下表转成“四分类 + 中文名”，
配置 `trash.item_map` 可覆盖。四分类规范名：**厨余垃圾 / 可回收物 / 其他垃圾 / 有害垃圾**。

| 类名 | 四分类 | 垃圾 | 类名 | 四分类 | 垃圾 |
|---|---|---|---|---|---|
| 1 | 厨余垃圾 | 果核 | 13 | 其他垃圾 | 烟头 |
| 2 | 厨余垃圾 | 鱼骨 | 14 | 其他垃圾 | 用过纸巾 |
| 3 | 厨余垃圾 | 菜叶 | 15 | 其他垃圾 | 破陶瓷 |
| 4 | 厨余垃圾 | 面包 | 16 | 其他垃圾 | 贝壳 |
| 5 | 厨余垃圾 | 茶渣 | 17 | 其他垃圾 | 用过的一次性筷子 |
| 6 | 厨余垃圾 | 剩饭 | 18 | 其他垃圾 | 废弃眼镜 |
| 7 | 可回收物 | 塑料瓶 | 19 | 有害垃圾 | 纽扣电池 |
| 8 | 可回收物 | 报纸 | 20 | 有害垃圾 | 荧光灯 |
| 9 | 可回收物 | 玻璃瓶 | 21 | 有害垃圾 | 废胶卷 |
| 10 | 可回收物 | 易拉罐 | 22 | 有害垃圾 | 油漆 |
| 11 | 可回收物 | 旧衣物 | 23 | 有害垃圾 | 药片 |
| 12 | 可回收物 | 纸箱 | 24 | 有害垃圾 | 水银温度计 |

分类区间：**1–6 厨余垃圾，7–12 可回收物，13–18 其他垃圾，19–24 有害垃圾**。

垃圾桶模型 `trash_brest.pt` 的 8 类自带桶类型与开/关；旧 `catalog.pt`（36 类中文名）
仍可切回（`models.trash_items: catalog.pt`）。垃圾分配按空间位置：整图检测后把垃圾框
分给中心落入的桶，桶没检到垃圾时只对该桶放宽裁图补检一次；类别前缀与桶类型一致判
“投放正确”，否则“错误”，没识别到垃圾判“未判定”（不填 0）。

## 5. 启动与自测

### 5.1 车上（一个终端）

```bash
source /opt/ros/noetic/setup.bash
source ~/bit_ws/devel/setup.bash
~/bit_ws/src/nav_demo/scripts/start_vision.sh
```

`start_vision.sh` 按 `vision_nodes.conf` 找脚本：`~/AI视觉` → `photo_capture/scripts`
→ `nav_demo/scripts`；每行可写解释器（`venv` / `system` / 绝对路径）。
**`inspection_vision` 必须用 `system`**（`/usr/bin/python3`）：系统环境才有
`lap`（ebike 视频计数）、`hyperlpr3`（车牌 OCR 兜底）、`pyserial`（红外串口）；
`~/baidu-aip-env` 缺这几个包。旧视觉节点仍可用 venv。

单独启动节点：

```bash
/usr/bin/python3 ~/bit_ws/src/photo_capture/scripts/inspection_vision.py \
  --device 0 --output-dir ~/Pictures/inspection_2026
```

注意：相机只能有一个采集进程，节点运行时不要再开 `photo_capture` GUI。

### 5.2 离线自测（不连 ROS 也行）

```bash
# 列出任务
/usr/bin/python3 inspection_vision.py --list-tasks

# 用图片/图片文件夹/视频代替相机
/usr/bin/python3 inspection_vision.py --self-test trash \
  --device ~/Pictures/photo_capture/垃圾1 --point 15 --sub-id 垃圾桶 \
  --output-dir /tmp/iv_test

# 红外回放（无红外硬件）
/usr/bin/python3 inspection_vision.py --self-test temperature \
  --thermal-capture ~/bit_ws/src/mlx90640_ros/test/data/capture.bin \
  --output-dir /tmp/iv_test

# 覆盖阈值 / 帧数 / 截止
--params '{"conf": 0.2, "frames": 3, "deadline": 10}'
```

图片文件夹模式：**同一次请求内所有帧都取同一张图**，下次请求换下一张，
避免多帧任务混用不同现场照片。

### 5.3 ROS 服务手动调用

```bash
rosservice call /inspection_vision \
  "{task: 'crowd', point_id: 2, sub_id: 'A街区', params: '{}'}"
```

## 6. 已验证 / 未验证（2026-11-21）

已验证（离线，2026 道具实拍 + 历史图）：

- 单元测试 24 个全过：`python3 -m unittest src/photo_capture/tests/test_inspection_vision.py`。
- ROS 服务联调（本机 roscore + 图片文件夹作相机）：crowd / light / sign / trash /
  ebike / fire 均 `success=true` 且返回的图片文件存在；plate 在错误取景下按预期失败；
  meter 传 `deadline=2.0` 时节点自己超时返回（`已用 2.1s`）；未知 task、非法 params 均被拒。
- `start_vision.sh` 一键启动：能解析仓库内脚本并用系统 python3 起服务。
- 实拍读数样例：light `红灯`；sign `A/left`（确认模型 0.95）；plate `学R00136`；
  fire `A大厦发现火灾隐患 2 个`（火源 0.83/0.80）；trash 4 桶 + 垃圾名；
  ebike 停车区 `正常 3 / 倒伏 0`；meter `15 度 / 42%`（10 帧稳定）。

未验证（需上车）：

- 2/4/6/8/13 号点 crowd、7/8/17 号点 fire 的实际取景识别率；`people.pt` 在远景只检出
  部分立牌，近处（0.6–1.0 m）成功率需实测。
- **sign 的 A/B ↔ left/straight 映射没有任何 B 摆位样本**，必须现场两种摆位各验一次；
  代码里 `sign.route_map` 是暂定值，错了就是 −10 分红线。另外 `Signal.pt` 在圆牌/表盘上
  可能误报 A，当前只做了 `stop.pt` 复核“有没有牌”，**位置先验 ROI 还没标**。
- **红外楼层表不落盘**：`thermal.floor_table` 默认四层等分，12 号点必须现场重录；
  当前 `building_d.pt`（楼体四角 pose）与窗模板对不齐，未参与判层。
- 楼宇名牌 OCR：`house.pt` 在 2026 实拍图上 4/6 裁不到字，已做“候选名单模糊匹配 +
  配置默认名兜底”；现场把 `fire.buildings.<点位>.candidates` 填成实际楼宇名。
- 停车区 17→18 **边平移边录像**：导航侧目前没有在平移中调用视觉的触发点，
  `params.pan=true` 的接口已实现，但要导航配合开线程调用（服务阻塞期间由别的线程发
  `/cmd_vel`）；节点不自己发速度。
- 仪表不上报裁判、不自动报分；相机实际协商分辨率（2592×1944@15）需上车核对。

### 6.1 上车前必须改（导航侧，不在本节点内）

1. **sign 的路线决策没接进导航**：`cruise_debug._recognize` 只把 `resp.result` 交给
   `_sign_route`，而 `_sign_route` 只打印日志；节点把方向放在 `extra.route`
   （`left`/`straight`，A/B 暂定），导航要解析它并真正切路线。A/B 映射本身也未经 B 摆位验证。
2. **crowd/ebike 街区内汇总没实现**：导航把每个视角的结果各上报一条，没有按
   `extra.block` 汇总成 handoff §3 的整句；`vision_tasks.py` 也缺 A/B 街区的 ebike 任务项。
   节点已按“每次只回当前视角”实现并给出 `extra.block/person/bad/illegal`。
3. **params 链路未通**：`_call_vision` 固定 `params='{}'`，所以 `params.pan/conf/frames/deadline`
   都传不进去；ebike 停车区平移录像要导航在 17→18 平移线程里带 `{"pan": true}` 调服务，
   同时由别的线程发 `/cmd_vel`。导航还应给 service 调用加线程+超时（ROS1 服务没有超时）。
4. **现场必填**：`thermal.port` 用 `ls -l /dev/serial/by-id/` 的实际设备，
   `fire.buildings` 填 7/8/17 号点实际楼宇名候选，sign 两种摆位各验一次。

节点侧已处理：`params.frames/conf/deadline` 生效、失败原因写 `recognitions.jsonl` 的
`error` 字段、启动预热全部 YOLO 模型 + PaddleOCR（约 2–3 s）、trash 低置信垃圾自动裁图
复检、`thermal.port` 不存在时启动警告、`start_vision.sh` 在未预置 ROS 的终端也能给出
可读错误而不是 `unbound variable`。

## 7. 现场标定清单（按顺序做一遍）

1. `camera.width/height/fps`：确认 Fic760x 实际协商值；车牌/仪表取景不清就升分辨率。
2. `thermal.port`（`ls -l /dev/serial/by-id/`）与 `thermal.floor_table`（按 4F/3F/2F/1F
   热点行重录），空表或默认表只适合演示。
3. `sign.route_map`：两种摆位各摆 10 次，记录 side→route；必要时给 4/5 号点补 ROI
   先验（当前未实现 ROI，靠置信度 0.6 + 3 帧投票）。
4. `fire.buildings`：填 7/8/17 号点的实际楼宇名候选与默认名。
5. 阈值：用 `params.conf` 现场试（light/trash/crowd/ebike），稳定后再写回 YAML。
6. `trash.item_map` 如与实际标签不一致再覆盖；确认用的是 `trash.pt` 24 类。
7. meter：`src/photo_capture/models/gauge_exp16.pt`（不入 Git）SHA-256 已内置校验，
   现场先单点联调、不自动报分。

## 8. 排障

| 现象 | 排查 |
|---|---|
| `任务不可用：...` | 看节点启动日志里 `任务 X 不可用`：缺 paddleocr/ocr_engine/lap 等；确认用 `/usr/bin/python3` |
| `/inspection_vision 不可用` | `roscore` / `devel/setup.bash` 是否 source；`rosservice list` |
| 相机打不开 | `/dev/video*` 是否存在、是否被 photo_capture GUI 占用 |
| 红外一直没帧 | `session.error()` 的内容会写进 `extra.error`；检查串口权限/`/dev/thermal`、`pyserial` |
| 图片页没图 | 看 `extra.raw_image` 与本次 `recognitions.jsonl`；输出目录是否可写 |
| 识别超时 | `params.deadline` 太小或模型首次加载；节点启动 `--warmup`（默认开）会预热 |
