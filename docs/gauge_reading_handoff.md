# 仪表识别：决策、派工与审核交接

> 建立日期：2026-11-15（Asia/Shanghai）｜文档版本：v13（固定机位隔离实验R5独立复核）
> 当前阶段：**原默认路径仍0/10；边缘+针身中心线实验已恢复环/针，最终1/8内部评估满足归一化2°保守稳定门，其余拒识。实验非可信读数、未通过自动读取验收；固定No.9011、相机设置和距离不变，未接车。**
> 当前结论：数学、标定工具及指定ROI单帧离线开发链路已复核；**没有独立实拍精度/现场性能验收，没有生产正式标定或实机放行。**
> 本文件是决策与任务状态的唯一台账；代码、数据和长日志通过链接引用，不在两个对话中重复粘贴。

## 0. 本会话内派工入口（用户已批准Workflow方式）

用户不再需要把指令搬到另一对话；由本会话Lead派Flash执行并独立审核。以下保留为任务边界摘要，不代表又创建了第二个并行实现者：

```text
请先读取仓库根目录 AGENTS.md 和 docs/gauge_reading_handoff.md。
先读A15：固定机位隔离实验R5已独立复核，停在可review补丁，无代理继续写入。
实物与参考均为同一No.9011，旧No.0011是误记；相机/距离固定，不再以靠近或调硬件换取通过。
新增隔离实验模块/测试与本地证据，不被默认读数器导入。1/8只是内部稳定候选，不是精度或自动读取验收。
所有value=null、trusted=false；不将低标准差或一次通过转换成最终RH，不以手机候选调门。
后续优先研究同burst的中心线几何联合估计，而不是平均条带峰/读数；需另行冻结范围与独立验证要求。
本轮未提交，需用户明确同意上传；不自行接车或解锁G2–G4，不改旧YOLO/导航。
```

当前采用**本会话Lead + 显式指定Flash的Workflow**，不再要求用户手动搬运两个对话的消息；实际任务/回报/审核仍记录在本文。内置团队工具可通信，但当前spawn_teammate接口不能指定provider/model，实测执行模型为gpt-6.1-sol，故该队友已暂停且未进行代码工作。用户已明确选择改用Workflow，调用固定`provider=deepseek-official`、`model=deepseek-flash`（DeepSeek V4.1 Flash），不静默换模型。共享工作区不等于文件锁；Lead独占本文，Flash仅写已批准的代码/测试范围。没有定时后台接车或自动提交。

## 1. 角色、权限与更新规则

| 角色 | 负责 | 不得代做 |
|---|---|---|
| 决策/审核方（本对话） | 方案取舍、发布任务、审查补丁和证据、签发实机测试许可 | 不替 DS 承担常规实现；不把推测当验收 |
| 实现方（本会话Workflow中的Flash） | 执行已发布任务、实现和自测、返回紧凑证据；Lead转录台账 | 不自行扩范围、不写Lead拥有的文档、不自行审核通过或接车 |
| 用户 | 提供缺失材料、执行实机测试、决定提交和上传 | 无需搬运整段日志或反复解释任务背景 |

- 决策方维护本文全部状态/派工/审核区，并将Flash返回的回报及证据链接追加到DS回报区；Flash不同时写本文，避免共享文件冲突。实机区由用户提供事实后记录。
- 回报和审核**追加，不覆盖历史结论**。修改已冻结决策时新增决策编号、理由和受影响任务，旧结论注明已替代。
- 一次只放行一个可审查阶段；已发布不等于已完成，代码能跑不等于识别正确，离线通过不等于实机通过。
- 按依赖决定“什么时候做”，不虚构工期；只有用户指定日程后才记录日期和时区。
- 不自动 commit/push。完成后检查 Git 差异，只汇总本任务文件并询问用户是否上传，不混入无关本地修改。

## 2. 起点与证据等级

### 2.1 初始核查事实（78b1984阶段；当前代码基线见A03）

1. 附件为《温湿度计读数 · 技术方案（交接版）》，目标实际上是 **deli 得力 No.0011 的湿度读数**，不能据标题认定温度读数已解决。
2. 已读取的附件只读副本：[技术方案](</home/mowen/.dsh/attachments/v1/files/a9/a9812dc10efc98354f5f0e7fa7b88af66f01bd3a877ec761888d030e0976b52d/技术方案.md>)；SHA-256：`a9812dc10efc98354f5f0e7fa7b88af66f01bd3a877ec761888d030e0976b52d`。该路径仅作来源记录，不保证另一环境可访问；任务执行所需摘要已写入本文。
3. 现有 [gauge.py](<src/photo_capture/scripts/gauge.py>) 使用 Pose 关键点、三静态点仿射正视化、Radon/针尖估角和量程比例换算；不是附件的新算法，也不能直接表达非线性刻度表。
4. [现有仪表说明](<src/photo_capture/README.md#L268-L351>)与 [GUI/CLI 入口](<src/photo_capture/scripts/capture_photos.py>)可作为以后集成入口；当前保留原功能与默认行为。
5. 本轮在仓库内按文件名查找，未发现附件列出的 `read_dial.py`、`calib.json`、`synth_test.py`、`tick_probe.py`。尚未取得对应原始样张、逐样本结果和硬件基准。这不代表用户其他目录或另一对话里没有，交给 G0 核实。
6. [站房任务口径](<docs/recognition_plan.md#L230-L248>)：温湿度对外不带小数，湿度误差目标 ±2 %RH，原图必须画识别指针；温度、电表是另外的任务，不在本轮实现范围。

### 2.2 附件声称，不视为已验收

- 报告了 900/450/288 px 下的误差、低分辨率跳峰、CPU 单帧 11–28 ms 等结果；未取得原始验证材料，不复制为本工程成绩。
- 合成指针实验不等于真实相机、真实表针和真实透视实验。
- “同型号无需逐台标定”“滚转不限”“倾斜 ≤20–25°”均先作为待验证假设。
- 禁止把未取得的素材写成已存在路径，把未执行的命令写成通过，把模拟通过写成实机通过。

## 3. 决策区：首轮冻结范围

### D01：先做湿度独立离线验证，旧功能不动

优先路线为：**彩色环定位 → 椭圆近似归一化 → 蓝色段定滚转 → 径向暗度找针 → 非线性查表 → 多帧拒错**。

首轮不训练 YOLO 关键点，不做每帧刻度提取，不增加模板图 warp 或单应矩阵，不做端到端湿度回归。不是宣称这些方法永远无效，而是本轮不扩路线；如果证据否定主线，先交决策方裁决。

不修改现有 Pose 模式；不增加 ROS 服务，不接自动巡航，不改底盘控制、点位和相机配置。以后尽量复用现有相机和落盘能力，但不得让新算法因导入旧 Pose 模块而强制依赖 ultralytics。

### D02：附件参数仅作可追溯初值

| 项目 | 附件初值 | 使用限制 |
|---|---|---|
| 坐标角度 | x 向右、y 向下，顺时针为正 | 全链路统一，不能混用数学坐标 |
| 0/100 %RH 角度 | 314.70° / 587.15°（100 端已展开） | 必须和实际标定坐标系一致 |
| 标定表 | 11 个主刻度，值为 0、10、…、100 | 正式角度表尚缺，不从取整跨度拼造 |
| 蓝色段中点参考 | `roll_ref_deg=196.4` | 须确认这是相同归一化坐标系中的参考 |
| 彩色环采样半径 | `colour_rho=0.94` | 相对环半轴，不是表壳半径 |
| 指针采样半径 | `[0.14, 0.70]`，70 个径向点 | 上限须避开印刷数字 |
| 角度采样数 | 3600 | 采样步长 0.1°不等于估计精度 0.1° |
| 暗度/覆盖率 | `max(150-gray,0)`；暗度>15；覆盖率≥0.70 | 光照适应策略须验证，不能私自调到测试全过 |
| 多帧初值 | K=5–10；3°剔除；至少3帧；IQR<0.5° | 阈值待离线验证，不是成功保证 |
| 成像条件 | 直径≥300 px，建议400+ | 不把放大后的像素数当真实分辨率 |

正式标定须包含：版本、表型、参考图及哈希/来源、坐标约定、归一化定义、严格递增且有限的角度表和数值表、滚转参考及采样参数。参考图上的像素轴心不是其他尺寸图像可以直接复用的常量。

### D03：优先暴露失效，不追求每次有值

- **几何**：椭圆圆化只是近似，不能消除一般透视。环中心不必等于指针真实轴心；禁止用表壳中心替代轴心。验证近圆情况下椭圆长轴方向不稳定、轴交换和角度跳变；标定与运行时必须使用同一几何定义。
- **选盘**：不能只按“最小椭圆”盲选；应有形状/残差/颜色等质量检查。多候选难以确定目标则拒识；未来 ROI 也必须配置和记录，不能藏在代码里。
- **滚转**：蓝段的首尾可能跨 360°；按环形连续段处理。蓝段不可见或不可信时拒识，不默认滚转为零。
- **角度**：每帧先扣滚转，再相对零刻度展开；在这个一致坐标系聚合，最后查表。覆盖 359°/1°、量程端点及空白区，不直接对原始环形角度求普通中位数。
- **指针**：输出覆盖率、主/次峰竞争等可解释质量信息。反光、文字、针尾、其他轴的结构都可能抢峰；“暗度法免疫反光”不成立。
- **多帧**：必须是不同帧标识且来自本次采集的新鲜帧，不重用缓存结果充数。稳定错峰不会被中位数修复，必须列入反例测试；仅有低离散度不能判 `ok`。
- **边界**：校验量程后才插值；不得依靠插值库端点夹紧掩盖超量程。落在无刻度区只说明“不在可读量程”，不能据此确定高于100还是低于0。
- **变更**：不静默回退旧算法、不放松阈值掩盖失败、不安装/替换全局 OpenCV。尤其不能盲装 headless 包破坏已有 GUI 环境。

### D04：内部精度与比赛输出分开

内部保留浮点读数和角度，便于评测；未来展示层按比赛口径转为整数，不能用提前取整掩盖算法误差。整数策略在 G4 接入时冻结；不改现有播报与记录格式。

### D05：明确湿度目标与证据边界（A01 新增）

- 代表帧中上方大轴心的长针、外圈蓝红环属于温度部分；目标湿度位于下方独立小轴心和彩色小环。不能把温度长针可见当作湿度指针已可可靠测量。
- G0-R1 复现的约119 px是**整表外径**，不是湿度彩环直径。后续同时记录整表直径、湿环尺度和各自测量依据；附件≥300 px暂按整表口径保留，不能悄悄改成湿度小环≥300 px。
- 不同文件SHA-256仅排除字节完全相同的文件，不能证明传感器帧新鲜、图像内容独立或指针读数稳定。
- 现有进程内 `frame_id` 可以保留，未来增加采集会话/分组标识和采集时刻即可作为最小身份方案的起点；哈希用于溯源/查重，不证明新鲜度。不得仅因重启归零就要求重建全套帧身份系统。G2具体设计仍未发布。
- 后续材料请求只需用户提供清晰原图及来源，SHA-256由代理读取后计算，不把可自动完成的工作转交用户。

## 4. 数据流与最小接口约束

```text
原始帧 + frame_id/采集时刻
 → 环与指针单帧估计 + 质量信息
 → 扣滚转、相对零点展开
 → 按本次采集分组筛选/聚合
 → 量程检查、非线性查表
 → 原图叠加轴心至指针的线 + 结构化记录
```

这是未来 G1/G2 的逻辑契约，**当前没有新增公共 API、ROS 消息或数据文件格式**。不强行套用旧 Pose 的检测结构。具体函数签名随 G1 发布时确定，不允许 DS 在 G0 擅自冻结工程接口。

结果最少包含：

| 字段 | 约束 |
|---|---|
| `status` | `ok` / `out_of_range` / `low_conf` / `no_dial` / `unstable` |
| `value`、`unit` | 单位 `%RH`；只有 `ok` 可提供对外有效数值，其他状态为 null |
| `angle_deg`、`roll_deg` | 补偿后展开角、滚转角；不可取得则 null，坐标定义须固定 |
| `frames_used`、`spread_deg` | 经过筛选的有效帧数、离散度；明确统计量为 IQR |
| `reason` | 稳定的原因码，不能只有无上下文的自然语言 |
| `calib_version`、`frame_ids` | 关联标定与输入；调试图须对应实际输入，不能错配最新帧 |
| `quality` | 覆盖率、峰竞争等具名指标；不把无定义的 `confidence=8.9` 当概率 |

配置/标定非法时明确报告配置错误，不伪装成 `no_dial`，不继续给读数；错误和图像拒识分开记录。`no_dial` 用于无可信目标，`low_conf` 用于存在候选但质量不足，`unstable` 用于多帧无足够一致证据。支持条件内真实超量程使用 `out_of_range`。

## 5. 派工区与任务看板

### 5.1 任务总表

| 编号 | 任务 | 当前状态 | 何时开始/何时停 |
|---|---|---|---|
| G0 | 输入与证据核查 | **审核通过（G0-R2 / A02）** | 核查阶段结束；不代表算法已实现或有效 |
| G1 | 独立离线单帧最小实现 | **G1b1离线开发限定通过（A10）；固定距离实拍默认路径仍0/10** | 保持已通过版本，不改相机/导航，不接实机 |
| E1 | 固定机位低饱和碎环隔离实验 | **R5已独立复核；方向方案部分可行，自动读取未验收（A15）** | 1/8内部稳定，其余拒识；只留诊断，不替换默认路径或要求靠近 |
| G2 | 多帧、反例与离线评测 | 待发布 | G1 审核通过后另行发布 |
| G3 | 停稳小车上的实机采集测试 | 待发布 | G2 证据通过、审核方签发明确实机许可后开始 |
| G4 | CLI/GUI及后续巡航接入 | 待发布 | 实机结果通过后另行冻结接口与修改清单；巡航另设放行点 |

状态：待发布 → 已发布 → 实现中 → 待审核 → 需修改 / 审核通过。涉及硬件再进入待实机 → 实机通过 / 实机失败。阻塞时保留原阶段，另记阻塞原因、负责人、解除条件；不能因缺件直接标记完成。

### 5.2 G0：原发布任务（已由 A02 验收结束）

**目标**：知道现有成果能否复用，缺什么才能实施；不是重复写一篇算法综述。

**允许修改**：只在本文第 7 节追加回报，更新 G0 执行状态。代码、配置、相机、环境均只读；保护已有无关修改。不在私人目录、整个家目录做无边界扫描。

**核查内容**：

1. 记录工作区、HEAD、现有 Git 差异基线；检查相关目录及用户明确提供的素材位置，查找四个缺失脚本/标定和原始样图。
2. 若找到源码，读取内容并辨别是否与附件一致；标明来源和版本，不因文件名相同就认定可复用。若只有文档，明确写“需重实现”。
3. 列出可用图片：表型、是否原图、真实直径、是否有序列、是否能读参考值、拍摄条件；生成清单写在回报中即可，不复制大图、不篡改原数据。
4. 检查 Python、NumPy、OpenCV 当前版本及可导入性；不得安装包、运行来历不明的脚本或调用摄像头。
5. 定位现有相机读取、帧标识、GUI/CLI调用和保存记录位置；回报文件/函数定位及最小未来接入点，不现在改接口。
6. 给出 G1 的建议新增文件范围、可复用函数、最小测试命令；实际实现路径待审核方发布后生效。

**验收**：每条结果有命令/路径或明确“未取得”，缺件有解除条件，未写识别代码且未改运行环境。G0 核查完可以待审核，不要求素材必须齐全。

**缺件处理**：缺旧源码可经审核决定重实现；缺正式标定或样图，仍可在另行获批后做数学/拒识单元测试，但不得生成虚假标定或宣称真实精度通过。只向用户提出当前最关键的一项材料请求，附具体用途。

### 5.3 后续任务的边界与交付

- **G1**：纯离线入口、单帧估计、严格标定校验、调试叠图、单元测试；输入为已授权本地图片。无可用标定时不得输出有效湿度。修改限独立算法/配置/测试，不动现有模式。
- **G2**：本次采集分组、新鲜帧和重复帧检测、多帧聚合、拒识、固定种子合成验证、独立实拍离线评测。给出全部案例及失败叠图；不能仅报告成功样本 MAE。
- **G3**：用户停稳小车采集；保存未经覆盖的原图、时间/帧标识、人工参考、参数、结果和失败样本。识别不发运动命令，测试不得改导航安全配置。若需新采集工具，先单独发布其实现范围，不能用实机许可代替代码变更审核。
- **G4**：复用现有相机，独立选择新后端，保留旧模式和默认行为；配置错误/超时可见、线程不阻塞 GUI、图像与结果对齐。再定整数输出、落盘兼容、延迟预算及旧模式回归；自动巡航和比赛部署另行审核，不随 GUI 接入自动放行。

### 5.4 G1a：数学内核与量程拒绝（已由A05验收通过，保留接口约定）

**目标**：在没有正式标定图时，先把角度环绕、滚转扣除、非线性查表和非法输入拒绝做成可审核的小补丁。G1拆为G1a数学内核、G1b完整单帧图像识别；G1a通过不等于G1完成。G1b仍待素材核对及另行发布。

**允许修改范围**：仅新增 `src/photo_capture/scripts/dial_math.py`、`src/photo_capture/tests/test_dial_math.py`，以及在本文DS回报区追加G1a-R1并更新G1a状态。不创建正式标定JSON、图像CLI、README或新依赖文件，不修改已有代码/配置，不把仓库外原图搬入仓库。只需Python标准库与unittest；不得导入旧gauge、yolo_detect、ultralytics、ROS、Tk，不打开相机或联网。

**冻结本子任务接口**（可有私有辅助函数；不做新框架）：

| 函数 | 约定 |
|---|---|
| `validate_scale(table_deg, table_val) -> None` | 两个一维list/tuple，各11个有限实数，排除bool/字符串/嵌套元素；角度严格递增且总跨度大于0、小于360°；数值恰为0、10、…、100。非法抛`ValueError`。这仅验证表结构，不证明真实标定有效。 |
| `wrap_deg(angle_deg) -> float` | 将有限实数角度归一化到[0,360)，支持负数和多圈角；非法输入抛`ValueError`。 |
| `relative_angle(needle_deg, roll_deg, zero_deg) -> float` | 返回`zero_deg + wrap_deg(needle_deg - roll_deg - zero_deg)`；roll_deg已是蓝段观测相对标定的滚转差，本函数不检测蓝段。输入角度均为图像顺时针正向。 |
| `lookup_value(angle_deg, table_deg, table_val) -> Optional[float]` | 先校验表及有限角度；angle_deg必须是已扣滚转并展开后的角度。闭区间内分段线性插值，量程外返回None；不自行wrap，不将空白区夹紧至0/100。 |

- 有限输入的中间运算若溢出，也抛`ValueError`；不返回NaN/Infinity。按A04/D08澄清：仅当角度在量程**外侧**且距对应端点≤1e-9°时，允许舍入误差吸附；量程**内侧**始终保留分段插值和全部合法节点，不按固定端点带吞并它们。除此之外不加量程宽容带。非法参数与正常超量程明确区分。
- 直接插值测试可以输入数学角度以验证纯函数；测试输出的浮点数不是实测湿度。合成11点表放在测试文件内，显式命名/注释为`synthetic`；不得填入附件取整跨度并声称正式标定，更不得创建可被真实识别入口加载的默认标定。
- 不在G1a实现环/指针/蓝段检测、椭圆变换、多帧投票、置信度或图像拒识。这里的“拒绝”仅指非法数学输入和量程外，不得以本阶段测试通过宣称反光/错峰已解决。

**必测清单与预期**：

1. wrap覆盖负角、0/360、359/1边界、多圈；扣滚转在正/负旋转下回到同一标定角，覆盖跨0°情形。
2. 使用明确非等距合成角度表，手工给出预期值，测11个节点、各段中点及湿端较密刻度；不能用被测插值函数生成期望值。
3. 测0/100闭区间端点、舍入误差、真正越界、空白区及“同方位但未展开”的输入；验证lookup不会自动wrap或端点夹紧。
4. 测错误长度/顺序、重复角度、跨度≥360或≤0、错误数值表、NaN/Infinity、bool/字符串/嵌套、算术溢出，均按契约拒绝。
5. 固定种子生成多组纯角度样本，独立按定义验证滚转周期性、正向表的插值单调性与节点正确性；用测试名称说明这不是图像合成或真实精度评测。
6. 新模块可在无相机/无ROS/无模型环境导入，无顶层读文件、写文件、运行线程或其他副作用。

**验收命令**（工作区根目录）：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -v
git diff --check
```

DS须报告实际测试数、退出码、补丁/新文件SHA-256以及无关改动基线未变的检查结果。另检查新增未跟踪文件的尾随空白；普通git diff --check不会覆盖未跟踪文件。所有必测项通过、修改不越界即可提交待审核；不设凑数量的测试指标，不安装pytest。回报正文尽量控制在一张表加必要失败说明，不再复制整轮核查。

### 5.5 G1b0：基于用户原图的离线候选标定（A06当前唯一发布项）

**目标/前置**：A05数学内核已通过，A06原图已取得。G1b进一步拆为G1b0离线候选标定、G1b1完整单帧图像检测；本轮只验证“这张图能否产出几何自洽、可复核的正式标定候选”，不承诺实拍精度，不提前写完整检测器。

**允许修改**：仅新增 `src/photo_capture/scripts/dial_calibration.py`、`src/photo_capture/tests/test_dial_calibration.py`、`src/photo_capture/config/dial/deli_0011_annotations.json`，以及本文DS回报/状态。不得改已通过的数学内核、旧YOLO及现有GUI/ROS/配置。原图继续只读，不入Git；本地复核输出放 `.gauge_calibration_review/`，不得提交该目录或擅自改gitignore。

**最小实现与标注约定**：

- 标注JSON记录版本、型号、原图路径/哈希/1279×1706尺寸、图像坐标约定、湿度轴心、11个带数值标签的主刻度点、两侧蓝段边界点、至少12个分布于可见湿度彩环同一边缘的拟合点。手工/半自动均可，但每个点均有原图坐标和对应叠图；不要混用温度环、数字文字或针尾。拿不准的点标明不确定性，不能造精确坐标。
- 工具CLI限定为`python3 src/photo_capture/scripts/dial_calibration.py --image <原图> --annotations <标注JSON> --output-dir .gauge_calibration_review`。读取后先校验哈希/尺寸、坐标有限且在图内、11点标签为0至100、环点数量/分布与拟合是否退化。配置/几何失败返回非零并说明原因，不继续输出有效候选。只用现有NumPy/OpenCV及标准库，不安装依赖，不import旧识别模块。
- 拟合湿度环椭圆，明确长短半轴定义。采用对称正定圆化矩阵`A=R diag(1/a,1/b) R^T`，保留图像方向，不额外把长轴转到水平；近圆和轴交换不得造成角度跳变。记录环中心、实测湿度轴心及二者归一化偏移，不直接假定两者相同，不复用附件像素坐标。
- 用同一矩阵变换从**湿度实测轴心**指向刻度/蓝段边界的向量，计算顺时针角；按0→100顺序展开，调用已通过的`validate_scale`校验11点候选表。蓝段中点按跨360°的短弧计算，给出参考角，不用普通端点算术平均。仅把附件常量作为差异对照，不能据其自动“校正”实测结果。
- 输出目录中生成`candidate_calib.json`和`calibration_overlay.png`。候选含`status: candidate`、来源哈希、标注/工具版本、原始标注、椭圆与矩阵定义、轴心偏移、蓝段参考、11点表及采样初值来源；叠图在原图标出湿度轴心、环拟合、11个刻度编号/值及蓝段边界，另给归一化视图。读者必须能从原图复核0/100方向和空白区；不把候选存成默认正式`calib.json`。
- 提交标注±1px扰动对各角度/湿端读数的敏感性、椭圆拟合残差和可疑点；这些是误差诊断，不是实测精度。如果关键点不可可靠标注或预算明显不足，回报风险，不能调参掩盖。

**测试/验收**：

1. unittest覆盖哈希/尺寸错配、非法/缺失/越界点、退化环、刻度顺序或几何不合法、蓝段跨0°、圆/近圆与椭圆轴交换不产生额外旋转；独立核对手算几何案例，不用被测函数生成期望值。
2. 保留G1a的27项回归；新增测试用合成几何夹具明确标注，不伪装为实拍。无相机/ROS/Tk/模型导入与顶层写文件副作用。
3. 实图产物须逐项复核11刻度、轴心、蓝段。原图湿度针在顶部空白区；如标出该针角，仅验证候选表量程拒绝，不报有效湿度，不能以这张图评估量程内读数MAE。
4. 回报附源图/标注/新文件哈希、精确命令和退出码、候选JSON与叠图的本地链接、与附件常量的差异和未验证项。候选表结构通过不等于正式标定通过，审核方签发后才能进入G1b1。当前不发布G1b1的运行时API。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_calibration.py' -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -v
```

### 5.6 G1b1：指定湿度ROI的离线单帧验证（A08当前唯一发布项）

**目标与范围**：先验证“环/蓝段/针能否从像素里被可靠分开”，不再扩展标定工具。首版由命令行显式传入湿度ROI，避免温度长针/大环干扰；ROI属于可见场景先验，必须记录，不能宣称全图自动识别。仅新增 `src/photo_capture/scripts/dial_reading.py`、`src/photo_capture/tests/test_dial_reading.py`，追加本文回报/状态；不改已通过文件、标注、旧YOLO及GUI/ROS，不安装包，不接相机/底盘，不联网或commit/push。

**入口/数据约定**：

- CLI：`python3 src/photo_capture/scripts/dial_reading.py <image> --calib <A08候选JSON> --allow-candidate --roi x y w h --debug-dir <新的空目录>`。ROI为原图像素矩形，正整数且完整在图内，叠图坐标回到原图。不得按图像文件名、已知参考轴心或已知针角返回结果；原图ROI可由标注环点包围盒加边距显式计算并记录，但检测轴心/角度必须来自本次图像。
- 冻结纯处理入口`estimate_frame(image_bgr, calibration, roi) -> dict`，不读文件、不写文件/开线程；CLI负责读图、配置与落盘。校准缺失/结构错误、ROI不合法、图不可读返回非零且不伪装无盘。加载候选须显式opt-in并校验A08候选内容哈希，继续调用已通过数学内核；不再依赖参考原图可用，不把候选标记篡改为正式。
- stdout输出一个JSON；正常图像拒识不是程序异常。沿用五类状态及原因码，但增加诊断字段`frame_status`、`candidate_value`。即使单帧质量通过，也只能`frame_status=ok`、顶层`status=low_conf`、`value=null`、`reason=multi_frame_required`；候选浮点值只在`candidate_value`，不能冒充最终读数。单帧失败时`candidate_value=null`，顶层反映失败；`spread_deg=null`，记录帧标识/哈希、ROI和标定版本。

**冻结算法主线与拒绝条件**：

1. ROI内HSV饱和度/颜色掩膜定位湿度环，拟合与候选一致的**彩带外边缘**；保留SPD圆化，不用长轴方向作滚转参考。若无环、拟合退化、残差过大或有多个同等可信候选，明确拒识，不按最小椭圆盲选。阈值集中为具名常量，回报中列出，不隐式按测试图切换。
2. 不能混用外缘和内缘的归一化半径。以拟合环中心先按rho=0.94采样蓝段，最长连续蓝段按环形处理；丢失/断裂歧义时拒识。参考中点必须来自A08候选，不固定用附件196.4°。
3. 轴心不得拿表壳中心替代。最小偏移迁移方法：候选给出`d0=A_ref*(axis_ref-center_ref)`，先用环心测运行时/参考蓝段中点差得初始roll；按`axis=c+inv(A)*R(roll)*d0`迁移轴心，再相对该轴心重测蓝段roll并更新一次轴心。参考环心中点由候选中蓝段边界/环心实测计算，不误用轴心中点；两次roll差>0.5°则拒识。回报局限：这是近正视仿射近似，非通用透视解；真实轴心定位/强透视另审，不擅自加模型。
4. 围绕更新轴心，在归一化rho=[0.14,0.70]、70个径向点、3600角度上计算暗度/径向覆盖率与高通后的候选峰；初值暗度`max(150-gray,0)`、有效暗度>15、覆盖率≥0.70。曝光不合适则报告质量不足，不为出值隐式改变阈值。记录主次峰、峰间角差和覆盖率；近邻峰先合并，竞争峰无法判别则拒识，不能仅取最暗线而不报告竞争。
5. 扣roll、相对零点展开、量程检查及非线性查表复用G1a。空白区返回`out_of_range`，不夹紧。调试叠图须画实测环/蓝段/迁移轴心/轴→针的线，图旁标出补偿前后角度、候选值及拒识原因。输出沿用A07安全原则：新空目录，写失败非零，不覆盖源图或旧成果。

**验收与证据**：

- 保留24项标定和27项数学回归；新增无盘/多环歧义、蓝段跨0°/缺失、近圆/轴交换、ROI坐标回映、配置错误、伪针竞争、量程端点/空白区、输出失败测试。合成指针须在源图副本中去除原针再绘制，固定种子和参数，明确不是独立实拍。
- 源参考图在顶部空白区：应检出并给`frame_status=out_of_range`；若质量不足可拒识，但须交失败叠图并写“参考图检测未完成”，不能把拒识当识别成功。
- 合成角度覆盖0/10/…/100及15湿度附近，滚转至少-6/0/+6°，整表缩放目标至少900/450/300 px（湿环尺度另记），模糊至少两档；记录每例角度误差、候选值误差、拒识及错峰，不设未经独立实拍证明的成功率承诺。量程内质量通过的合成案例，候选值误差应≤2 %RH；若达不到，先交证据，不靠放松质量门槛通过。
- 原有200张约119 px整表图仅作低分辨率挑战，至少处理代表帧并记录可见性/拒识；没有真值不计算真实MAE。单帧源码/测试/合成规则与完整指标清单及成功/失败叠图组成G1b1-R1。候选输出仅供离线审核，G2多帧、自动ROI和实机均未发布。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_reading.py' -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_calibration.py' -q
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -q
```

## 6. 验收、审核包与放行

### 6.1 必测与指标

- 数学/配置：角度环绕、蓝段跨界、0/100端点和空白区、非单调/缺失/NaN标定、坐标变换一致性、近圆与椭圆轴交换。
- 图像/拒识：无盘、多候选、蓝段缺失、低分辨率、反光遮挡、印刷文字/针尾伪峰、真实指针不突出。
- 聚合：少帧、重复帧、旧帧、各帧滚转不同、离群跳峰、双峰、持续稳定错峰、全部超量程。不能以重复同一张图片冒充多帧鲁棒性。
- 合成：已知指针角覆盖全量程和低湿端数字重叠区域，结合滚转、分辨率、模糊；固定随机种子，公开生成规则。二维仿射合成不能证明真实透视补偿有效。
- 实拍离线：标定集与验收集分开；使用至少10组独立采集序列、每组5–10帧，覆盖至少5种拍摄条件组合。连续30帧拆成数组不算独立采集，多次缩放同一原图也不算独立实拍。
- 条件至少覆盖正常正视、近最低分辨率、滚转、斜视、不同光照；另列遮挡/反光/蓝段缺失等故障集。湿度范围如未覆盖全量程，必须写清覆盖区间，放行仅限已验证条件。
- 真值以清晰参考图按刻度人工核读，记录参考区间和标注者；看不清的图片只用于拒识/鲁棒性，不作为精度真值。评价读针正确性，不把表本身物理测湿误差混进算法误差。

每份评测报告列出：数据来源及划分、总序列/帧数、有效率、拒识率、错误 `ok` 数、有效读数 MAE/最大误差、适用条件、失败清单。精度按序列最终浮点输出评价，拒识不能计作零误差；单帧指标另列。耗时注明 CPU、输入尺寸、版本、预热、样本数及是否包含读图/采集，至少报告中位数和 P95，不只报最快值。

**首轮 G2→G3 放行目标**：独立正常验收集的序列有效率≥90%，所有 `ok` 浮点读数距人工参考区间不超过2 %RH，故障案例不得输出误导性 `ok`；必测项通过且没有未解释的稳定错峰。±1 %RH为优化目标，不承诺附件亚百分比精度。小样本零错误不等于证明真实错误率为零。

达不到时如实回报并停止，不能现场降低门槛；若需要取舍，由决策方记录新决策。性能先测量，附件“≤30 ms”暂不作为已实现承诺；G4 再结合整站任务时间预算裁决。

### 6.2 紧凑审核包模板（DS 每轮必须填写）

```text
任务/回报编号：Gx-Rn
本轮结论：待审核 / 需材料（不能写“审核通过”）
版本：基线HEAD + 提交ID；未提交则提供本轮补丁及相关文件SHA-256
改动：文件链接、目的、影响（不超过5条）
验证：精确命令 → 退出码 → 测试数/关键指标；没跑写未验证
数据/标定：版本、来源、划分、哈希或清单链接
证据：完整报告链接 + 代表性成功/失败叠图；长日志仅链接
风险：已知失败、未覆盖条件、是否触及原有功能
下一步：建议，不自行越级执行
需要裁决：最多一个当前关键问题；无则写无
```

G0 没有代码补丁/图像指标时写“不适用”，提交核查命令及材料清单，不能为凑模板生成空成果。未提交代码可以审核，不得为方便审核擅自 commit。

### 6.3 审核与实机许可

审核方独立核对补丁、测试可复现性、失效处理与范围；结论只能是“需修改”“需材料”“本阶段通过”。

实机许可必须写明：任务/回报编号、代码/补丁哈希、标定版本、允许设备与条件、禁止动作、待采证据。未绑定版本的“看起来没问题”不是实机许可；审核后有代码或关键参数变化必须重新审核。

**离线通过仅允许规定范围的实机测试，不等于可直接比赛。** 实机失败回到相应任务，保存失败样本；不边跑车边盲改参数，不跳过审核。最终提交/上传仍需用户单独同意。

## 7. DS 回报区（实现方追加）

G0-R1 已提交（2026-11-15，Asia/Shanghai），状态：**待审核**。G1–G4 仍未发布，本轮未启动 G1。

G0-R2 已提交（2026-11-15，Asia/Shanghai），状态：**待审核（G0-R2）**。仅按 A01 做定向更正；G1–G4 仍未发布，未启动 G1。

G1a-R1 已提交（2026-11-15，Asia/Shanghai），状态：**待审核（G1a-R1）**。仅新增 G1a 数学内核与 unittest；G1b/G2–G4 仍未发布，未接车。

G1a-R2 已提交（2026-11-15，Asia/Shanghai），状态：**待审核（G1a-R2）**。仅修端点吸附并补回归；G1b/G2–G4 仍未发布，未接车。

G1b0-R1 已提交（2026-11-15，Asia/Shanghai），状态：**待审核（G1b0-R1）**。仅新增离线候选标定工具、标注、测试与本地复核产物；G1b1/G2–G4 仍未发布，未接车。

G1b0-R2 已提交（2026-11-15，Asia/Shanghai），状态：**待审核（G1b0-R2）**。仅修输出安全、原子写入和零向量拒绝并补故障回归；G1b1/G2–G4 仍未发布，未接车。

G1b1-R1 已提交（2026-11-15，Asia/Shanghai），状态：**待审核（G1b1-R1）**。仅新增指定ROI单帧验证工具/测试与本地复核产物；G2–G4 仍未发布，未接车。

<!-- DS 从此处追加 G0-R1；引用现有文件时使用仓库根相对路径链接。 -->

### G0-R1 紧凑审核包

| 字段 | 内容 |
|---|---|
| 任务/回报编号 | G0-R1 |
| 本轮结论 | **待审核**（不是审核通过；未开始 G1） |
| 版本 | 基线 HEAD `78b1984852b362fb86c907aa55c78bb1532fdee6`（`main`，与 `origin/main` 同步）；本轮未 commit。唯一改动为本文第 5.1 节 G0 行与本节。本文基线 SHA-256 `2bd864d9cec4a6fe5f58e7104215a2aa6b8951cbe2b752e53a779f9735f45dc0`；相关文件哈希见下。 |
| 改动 | 1. [G0 状态](<docs/gauge_reading_handoff.md#L130>) 改为“已回报，待审核”。2. 本节追加 G0-R1。3. **无**代码、配置、GUI、ROS、导航、相机、依赖改动；未 commit/push。 |
| 验证 | 见“G0-1”。通过项：Git 基线、附件哈希、缺件检索、样图计数、依赖导入、旧模块导入、unittest 框架基线。**未跑**识别/精度/相机/ROS 测试：本轮无实现、无正式标定，按任务边界不跑。 |
| 数据/标定 | **无正式标定**；附件 `calib.json` 未取得。可用真实图像：`/home/mowen/Pictures/photo_capture/仪表识别/` 200 张（20 组 × 10 帧，1280×720，表盘直径中位约 119 px）。它不是附件 §2 的 900 px 样张，也没有人工参考值。 |
| 证据 | 代表帧：[0001](</home/mowen/Pictures/photo_capture/仪表识别/仪表识别_20261114_215027_542_0001.jpg>)、[0200](</home/mowen/Pictures/photo_capture/仪表识别/仪表识别_20261114_215902_300_0200.jpg>)；整包：[仪表识别.zip](</home/mowen/Pictures/photo_capture/仪表识别.zip>)（SHA-256 `ac9911c769e6528822449a3c4adeb216e765b0af44d8d26fd849ec4f8a5486e6`）。 |
| 风险 | 见“G0-7 风险与未覆盖”。 |
| 下一步 | 交审核方核对；**不自行启动 G1**。 |
| 需要裁决 | 见文末“材料请求/裁决（仅一项）”。 |

**相关文件 SHA-256（本轮只读，未改）**

| 文件 | SHA-256 |
|---|---|
| [gauge.py](<src/photo_capture/scripts/gauge.py>) | `31902de1561ce3b48e885876383c0a1b4c92c7ece4bb4395d4461c4d0101b800` |
| [capture_photos.py](<src/photo_capture/scripts/capture_photos.py>) | `353f4fd8e992ead728618f45e44fad7b59fb1d9cbc02987e7e20b7749928cc65` |
| [README.md](<src/photo_capture/README.md>) | `fcbc3ae7ad628035db4722881d709cbfdf2bb8836fea55b73b52d169f1c74dfb` |
| `~/AI视觉/Pose.pt` | `c010b3a3849a786499ebcbcc5288536bc092fef1348c2d57d9ac1dbaf0cdc071` |
| `/home/mowen/Pictures/photo_capture/仪表识别.zip` | `ac9911c769e6528822449a3c4adeb216e765b0af44d8d26fd849ec4f8a5486e6` |
| 代表帧 0001 / 0200 | `e438c354a82b4e3d24245f6fcdeda3b45b55f41133edd1fbf6a9c4082635e230` / `d83eb683c18186bcb872442102aa0964a28635ef1a71b65dfe78739201dd15b3` |

### G0-1 环境、Git 基线与核查命令

- 工作区 `/home/mowen/bit_ws`；分支 `main`；HEAD `78b1984852b362fb86c907aa55c78bb1532fdee6`（`docs: 新增仪表识别交接台账…`，2026-11-15 15:59:48 +0800）；开始时 `## main...origin/main`，无 ahead/behind。
- 开工前已有、与本轮无关的本地差异（**均未触碰**）：`M src/nav_demo/scripts/waypoint_gui.py`（`024b6948…`，键盘遥控改动）；未跟踪 `docs/electric_scooter_video_counting.md`（`6b7fb865…`）、`src/MLX90640_交付包.zip`（`0f87c690…`）、`src/nav_demo/waypoints/inspection_waypoints_before_straight.yaml`（`3da157dd…`）。
- 结束时工作区只应多出本文改动；复核命令：`git status -s; git diff --stat`。

```bash
cd /home/mowen/bit_ws
git status -sb && git log -1 --format='%H%n%ad%n%s' --date=iso
# exit 0；HEAD 78b1984…；开工前 1 个已修改 + 3 个未跟踪（上表），本轮未改

sha256sum "/home/mowen/.dsh/attachments/v1/files/a9/a9812dc10efc98354f5f0e7fa7b88af66f01bd3a877ec761888d030e0976b52d/技术方案.md"
# exit 0；a9812dc10efc98354f5f0e7fa7b88af66f01bd3a877ec761888d030e0976b52d，与交接文档一致

find . -path ./.git -prune -o -path ./build -prune -o -path ./devel -prune -o -type f \
  \( -name 'read_dial.py' -o -name 'calib.json' -o -name 'synth_test.py' -o -name 'tick_probe.py' \) -print
find /home/mowen -maxdepth 5 \( -path /home/mowen/.cache -o -path /home/mowen/.local -o -path /home/mowen/.npm \
  -o -path /home/mowen/.dsh -o -path /home/mowen/bit_ws/build -o -path /home/mowen/bit_ws/devel \) -prune -o \
  -type f \( -name 'read_dial.py' -o -name 'calib.json' -o -name 'synth_test.py' -o -name 'tick_probe.py' \
  -o -name 'fig1_detect.png' -o -name 'fig2_scale.png' \) -print
# 两条均 exit 0，均无输出（未找到）

find /home/mowen/Pictures/photo_capture/仪表识别 -maxdepth 1 -type f -name '*.jpg' | wc -l
# exit 0；200
unzip -Z1 /home/mowen/Pictures/photo_capture/仪表识别.zip | grep -c '\.jpg$'
# exit 0；200；同一命令 grep -v 后非 jpg 条目为 0

python3 -V
python3 - <<'EOF'
import sys, numpy, cv2
print(sys.version.split()[0], numpy.__version__, cv2.__version__, cv2.getBuildInformation().split('GUI:')[1].splitlines()[0].strip())
EOF
# exit 0；3.8.10 1.24.4 4.12.0 NONE（活动 cv2 为 headless 构建）

python3 -m pytest --version
# exit 1；/usr/bin/python3: No module named pytest

cd /home/mowen/bit_ws
python3 -c "import sys; sys.path.insert(0,'src/photo_capture/scripts'); import gauge; print(gauge.__file__)"
# exit 0；旧 gauge 模块可独立导入（不触发 ultralytics 加载）

python3 -m unittest discover -s src/photo_capture/tests -p 'test_thermal_calibration.py' -v
# exit 0；Ran 26 tests … OK（纯几何/JSON 基线，仅证明 unittest 可用，不作为本任务验收）

python3 - <<'EOF'
# 一次性尺寸核对：10 个代表帧，Hough 圆测表盘外径；未保存为工程代码
import cv2, numpy as np, pathlib
files = sorted(pathlib.Path('/home/mowen/Pictures/photo_capture/仪表识别').glob('*.jpg'))
vals = []
for f in [files[i] for i in range(0, 200, 20)]:
    g = cv2.medianBlur(cv2.cvtColor(cv2.imread(str(f))[150:340, 520:720], cv2.COLOR_BGR2GRAY), 3)
    c = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, 1.2, 80, param1=100, param2=30, minRadius=45, maxRadius=80)
    if c is not None:
        vals.append(2 * c[0][0][2])
print('n=%d median=%.1f min=%.1f max=%.1f' % (len(vals), np.median(vals), min(vals), max(vals)))
EOF
# exit 0；n=10 median=118.9 min=113.8 max=129.6（表盘外径约 114–130 px）
```

### G0-2 附件与缺件

| 项 | 结果 |
|---|---|
| 《技术方案.md》 | **已取得**，只读副本路径可用，SHA-256 与交接文档一致；本轮据其核对常量，**未复现其精度/性能**。 |
| `read_dial.py` | **未取得**（仓库、home 深度 ≤5、桌面/下载/文档/图片、附件目录均无匹配）。 |
| `calib.json` | **未取得**；附件 §6 只给了字段名和 11 点表的存在性，没有正式角度表。 |
| `synth_test.py` | **未取得**；合成验证需重写，且二维仿射合成不能证明真实透视。 |
| `tick_probe.py` | **未取得**；无法用附件方法从样张重建 11 点角度表。 |
| `fig1_detect.png` / `fig2_scale.png` | **未取得**。 |
| 900 px 原始样张 | **未取得**；现有 200 帧是 1280×720 远距拍摄（表盘约 119 px），与附件 §2 的 900 px 样张不是同一素材。 |
| 结论 | **附件算法源码为 0 份，只有文档；G1 属于“需重实现”，不是复现附件。** 仓库内现有 [gauge.py](<src/photo_capture/scripts/gauge.py>) 是旧 Pose 关键点 + 三静态点仿射 + Radon 路线，不是附件路线（见 G0-5）。 |

### G0-3 可用图片清单

| 项 | 事实 |
|---|---|
| 路径 | `/home/mowen/Pictures/photo_capture/仪表识别/`（仓库外，未被 Git 跟踪）。 |
| 数量/格式 | 200 张 JPEG，1280×720，单张约 300 KB；200 个 SHA-256 全不同，无重复帧。 |
| 分组 | 20 组 × 10 帧；2026-11-14 21:50:27–21:59:02；组内约 0.4 s，组间约 14–67 s。 |
| 来路 | 文件名符合旧 [Session.save](<src/photo_capture/scripts/capture_photos.py#L491>) 的 `<类型>_<时间戳>_<序号>.jpg`，即旧拍照模式以“仪表识别”为类型保存的相机原帧（JPEG q≈95）；不是新算法产出。 |
| 表型 | 圆形 deli 风格温湿度表，外圈蓝/红彩环、内圈湿度刻度；与附件所述 deli No.0011 外观一致（**目视，未核对型号丝印**）。 |
| 真实直径 | 中位约 **119 px**（10 帧 Hough，范围 114–130 px）；远低于附件“≥300 px，建议 400+”的部署线，也不是 900 px 样张。 |
| 参考值 | **无**人工参考、无标签。所查看帧中表盘下方白色 4 位显示目视为 `1234`，用途未核实，**不得当湿度真值**。 |
| 蓝段/指针 | 可见：彩环左上蓝段、指针约 11 点方向；但该分辨率下刻度数字不可辨，附件所述双轴心/238 px 关系无法核对。 |
| 条件覆盖 | 抽查帧基本同一机位、距离、光照（未逐帧测量），仅“正常正视、远距、小表盘”一类；场景内有人走动。**不覆盖**近最低分辨率、滚转、斜视、不同光照等 5 类条件。 |
| 可用范围 | 只能做冒烟、蓝段/指针可见性、多帧重复性与拒识材料；**不能**作精度真值、不能验证附件 900/450/288 px 指标、不能替代正式标定。 |
| 压缩包 | `仪表识别.zip` 58,801,104 B，SHA-256 `ac9911c7…`；仅 200 张 jpg，**无脚本、无 calib、无真值表**。 |
| 其他候选 | 仓库内 16 张图片均为界面截图/其他任务素材（如 `logs/photo_capture_gauge.png` 是旧 GUI 截图）；`~/Pictures/photo_capture` 其余目录为垃圾、车牌、电动车、站房等，未发现表盘样张。 |

### G0-4 依赖与可导入性（未安装任何包）

| 项 | 版本/状态 |
|---|---|
| Python | 3.8.10（`/usr/bin/python3`） |
| NumPy | 1.24.4，可导入 |
| OpenCV | 活动 `cv2` 4.12.0，可导入；`cv2.getBuildInformation()` 为 `GUI: NONE`（headless 构建） |
| OpenCV 发行包 | `opencv-python 4.12.0.88`、`opencv-python-headless 4.12.0.88`、`opencv-contrib-python 4.10.0.84` **三套同时存在**；不要安装/替换/降级，避免破坏现有 GUI 环境 |
| ultralytics | 8.3.239 可导入（旧 Pose 需要）；**新算法不得依赖它** |
| 其他 | PyYAML 6.0.3、Pillow 10.4.0、tkinter、rospy、cv_bridge 可导入；ROS 为 Noetic |
| 测试框架 | `pytest` **未安装**（`No module named pytest`）；现有 `src/photo_capture/tests/` 使用 `unittest` |
| highgui | `src/photo_capture/scripts/` 无 `cv2.imshow/namedWindow/waitKey` 调用；现有 Tk GUI 不依赖 highgui，故 headless 构建未阻断旧功能。G4 不要新增 `cv2.imshow`。 |

### G0-5 现有集成位置与最小接入点

| 能力 | 位置 | 现状/结论 |
|---|---|---|
| 相机打开/取帧 | [capture_photos.py](<src/photo_capture/scripts/capture_photos.py#L352>) `Camera.open/read`；CLI `--device 0`、`--width 1280`、`--height 720` | 可复用；G1 不接相机，G4 再用。 |
| 帧标识 | [GaugeApp._tick](<src/photo_capture/scripts/capture_photos.py#L2909>) 自增 `_frame_id`；[save_readings](<src/photo_capture/scripts/capture_photos.py#L3155>) 写入 `readings.jsonl`（`frame_id` 在 L3168） | **进程内临时编号，重启归零，无图像哈希**；不满足 G2 的“新鲜帧/重复帧”证据要求。G2 必须另建帧身份，不能复用。 |
| 保存帧 | [Session.save](<src/photo_capture/scripts/capture_photos.py#L491>)；默认输出 [DEFAULT_OUTPUT_DIR](<src/photo_capture/scripts/capture_photos.py#L136>) `~/Pictures/photo_capture` | 命名=类型+毫秒时间戳+序号；G3 需另存原图哈希/采集组。 |
| 旧仪表 GUI | [GaugeApp](<src/photo_capture/scripts/capture_photos.py#L2769>)、[_process_readings](<src/photo_capture/scripts/capture_photos.py#L2942>)、[save_snapshot](<src/photo_capture/scripts/capture_photos.py#L3133>)、[save_readings](<src/photo_capture/scripts/capture_photos.py#L3155>)、`_reading_root` L2833 | 旧 Pose 模式；**本轮不改**，G4 前也不得改默认行为。 |
| 旧算法 | [gauge.py](<src/photo_capture/scripts/gauge.py#L184>) `PoseDetector`（L200 运行时导入 ultralytics）、[GaugeRectifier](<src/photo_capture/scripts/gauge.py#L490>)、[compute_fraction](<src/photo_capture/scripts/gauge.py#L101>)、[estimate_pointer_angle](<src/photo_capture/scripts/gauge.py#L450>) | 三静态点仿射 + Radon + 线性比例；**不是附件算法**。可参考其角度环绕/长弧处理，不可直接用于非线性 11 点表，也不得把 `PoseDetector` 拖进新模块。 |
| CLI/入口 | [MODE_GAUGE](<src/photo_capture/scripts/capture_photos.py#L146>)、[parse_args](<src/photo_capture/scripts/capture_photos.py#L5981>)、[main](<src/photo_capture/scripts/capture_photos.py#L6106>)、`--mode gauge` 与 `--gauge-*` | 新后端将来只增独立模式；G1 只做离线入口，不接此 CLI。 |
| 文档 | [README 仪表识别模式](<src/photo_capture/README.md#L268-L351>) | 旧 Pose 说明；新算法需另写文档。 |
| 测试 | `src/photo_capture/tests/*.py`，`unittest`，无 `pytest` | G1 测试沿用 `unittest`，纯离线、无 Tk/ROS/相机。 |

**最小未来接入点（G4，非本轮）**：在 [capture_photos.py](<src/photo_capture/scripts/capture_photos.py#L91>) 现有 `import gauge as gauge_module` 之外，增加**独立**后端导入/模式选择；旧 `gauge.py` 与旧模式保持原样。G1 不创建 GUI/ROS 入口，也不冻结这里的接口。

### G0-6 G1 建议范围（仅供审核，不是冻结接口）

1. **新增独立离线模块**（候选路径，审核后定）：`src/photo_capture/scripts/dial_reading.py`（或同目录独立包），只依赖 `numpy`、`cv2`（不得 import `gauge.PoseDetector`、`yolo_detect`、`ultralytics`），无 ROS/相机/Tk。
2. **独立标定配置**：`src/photo_capture/config/dial/deli_0011.json`（候选名），必须含版本、表型、参考图来源/哈希、坐标约定、归一化定义、严格递增有限的角度/数值表、`roll_ref_deg`、采样参数；缺失/非单调/NaN 一律报配置错误，不输出读数。
3. **测试**：`src/photo_capture/tests/test_dial_reading.py`（`unittest`），覆盖角度环绕、蓝段跨界、0/100 端点、空白区、非单调/NaN 标定、近圆/轴交换、无盘/多候选/低覆盖拒识、超量程；合成测试固定随机种子并公开生成规则。
4. **证据**：G1 另交 `docs/gauge_g1_report.md`（或本文后续追加），列出命令、样本、失败叠图；不把合成当实拍。
5. **可复用**：算法侧没有可直接复用的附件函数；旧 `compute_fraction`/`_wrap` 只可作角度环绕参考。相机/保存能力留到 G3/G4，复用 `Camera`、`Session`，但需补帧哈希与采集组。

**最小测试命令（G1 落地后）**

```bash
cd /home/mowen/bit_ws
python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_reading.py' -v
# 当前 pytest 未安装；现有 unittest 基线 26/26 通过（见 G0-1）
```

### G0-7 风险与未覆盖

- **无正式标定、无附件源码、无 900 px 样张**：附件的 900/450/288 px 误差、11–28 ms、±6° 滚转补偿等均**未复现**，不得写成已实现。
- **现有 200 帧表盘仅约 119 px**：低于附件 ≥300 px 部署线，印刷刻度不可辨；不能用于精度验收，也不能证明附件低分辨率跳峰结论。
- **条件单一**：抽查未见多机位/多光照变化，不满足 G2 的 5 类条件与独立验收集；不能把“20 组”当 20 种条件。
- **无真值**：白色显示 `1234` 不是湿度参考；没有人工核读区间，任何读数都无精度意义。
- **OpenCV 三包并存 + headless 构建**：不要安装/替换；新代码只用 `cv2` 算法 API，不碰 highgui。
- **旧 Pose 耦合 ultralytics**：新模块若 import 旧 `gauge`/`yolo_detect`，会把 YOLO 依赖带入离线测试；G1 明确隔离。
- **帧身份不足**：旧 `frame_id` 进程内自增、无哈希；G2 必须另建。
- **素材在仓库外**：200 张图和 zip 不得擅自搬入仓库/提交；需要时只提供清单与哈希。（本轮未复制、未改原图。）

### 材料请求/裁决（仅一项）

**请提供附件 §2 所用的 900 px 原始样张**（或等效的 ≥400 px、正面、能看清刻度的 deli No.0011 原图），附来源与 SHA-256。用途：核对轴心/椭圆/蓝色段常量，并据此建立正式 11 点角度表；没有它，G1 只能做重实现 + 数学/拒识/合成测试，不能输出真实 `%RH`。

若无法提供，请裁决：**G1 是否允许在“无正式标定”前提下先做合成/拒识/数学测试，并明确禁止输出有效湿度值**（仍不得宣称真实精度）。

### G0-R2 定向更正（A01 P1–P3）

> 2026-11-15，Asia/Shanghai。**G0-R1 原文保留**；本节只就近更正，冲突处以本节、A01 和 D05 为准。未新增算法/配置/相机/环境变更，未 commit/push，未启动 G1。

#### 紧凑审核包

| 字段 | 内容 |
|---|---|
| 任务/回报编号 | G0-R2 |
| 本轮结论 | **待审核**（不是审核通过；G1 未发布、未启动） |
| 版本 | 基线 HEAD `78b1984852b362fb86c907aa55c78bb1532fdee6`；未提交。本轮开始前本文 SHA-256 `be4afab5077b3e15de6d14888707a97cad25d055f8c30419c2185519b52594a0`；本轮只改本文（G0 状态行 + 本节）。代码/配置未变，相关哈希同 G0-R1。 |
| 改动 | 1. [G0 状态](<docs/gauge_reading_handoff.md#L139>) 改“待审核（G0-R2）”。2. 本节按 P1/P2/P3 逐项更正。3. **无**代码/配置/GUI/ROS/导航/相机/依赖改动；未 commit/push。 |
| 验证 | 见下方“R2 验证命令”；结果：ZIP 201 条目（200 JPG + 1 目录）、两张代表帧 EXIF 0 条。**未跑**识别/精度/相机/ROS/无关单元测试。 |
| 数据/标定 | 无变化：无正式标定、无 900 px 样张；200 张 1280×720，整表外径约 119 px；湿环未定量。 |
| 证据 | 代表帧 [0001](</home/mowen/Pictures/photo_capture/仪表识别/仪表识别_20261114_215027_542_0001.jpg>)、[0200](</home/mowen/Pictures/photo_capture/仪表识别/仪表识别_20261114_215902_300_0200.jpg>)；本轮只做目视复核，未生成新叠图。 |
| 风险 | 湿环/短针在整表约 119 px 下过小，未证明可稳定测角；哈希/文件名不能证明新鲜或相机直出；G2 接口未定。 |
| 下一步 | 交审核方复核；**不启动 G1**。 |
| 需要裁决 | 无新增；材料请求措辞按 A01/D05 更正（见 R2-3）。 |

**R2 验证命令**

```bash
unzip -Z1 /home/mowen/Pictures/photo_capture/仪表识别.zip | wc -l
# exit 0；201
unzip -Z1 /home/mowen/Pictures/photo_capture/仪表识别.zip | grep -v '\.jpg$'
# exit 0；仪表识别/（唯一的非 JPG 条目，目录）
python3 - <<'EOF'
from PIL import Image
for p in ('/home/mowen/Pictures/photo_capture/仪表识别/仪表识别_20261114_215027_542_0001.jpg',
          '/home/mowen/Pictures/photo_capture/仪表识别/仪表识别_20261114_215902_300_0200.jpg'):
    print(p.split('/')[-1], len(Image.open(p).getexif()))
EOF
# exit 0；两张均 0 条 EXIF
```

#### R2-1（P1）整表、温度部分、湿度目标分开

| 对象 | 代表帧中的位置/含义 | 可辨认性 | 结论 |
|---|---|---|---|
| 整表 | 圆形表体外缘，约 **119 px** 外径 | 清楚 | 只是整表尺度；**不等于湿环尺度** |
| 温度部分 | 上方大轴心 + 长针 + 外圈蓝/红弧 | 清楚；0001 长针约 11 点，0200 约 12 点 | 不是本任务目标；不能当湿度可用证据 |
| 湿度目标 | 下方独立小轴心 + 小彩色环 + 短针 | 小环和短针可见（0001 短针约水平向左，0200 约左下），但**明显小于整表、未定量**；与印刷刻度/色带重叠 | 只能说明“存在且可目视区分”；**未证明可稳定测角**；不猜当前湿度值 |

- G0-R1 的“蓝段/指针可见”只对温度部分成立；湿度目标改为“可见但未验证”，撤回其作为可用蓝段/指针证据。
- 目视方法：对 0001/0200 的表盘 ROI 作 8× 放大后对照；临时放大图已删除、未入库，未做定量湿环测量，也未生成算法叠图。
- 附件 ≥300 px 仍按**整表外径**口径保留，不悄悄改成湿环 ≥300 px；湿环没有可靠量取，不填数字。
- 0001/0200 的湿度短针方向不同，只能说明它可能在变化；没有人工参考，不能给湿度值或精度。

#### R2-2（P2）哈希、帧身份与新鲜度

- 200 个 SHA-256 不同：**只排除字节完全相同的文件**；不能证明传感器帧新鲜、内容独立、采集条件独立或指针读数稳定。组内 10 帧时间连续，可能高度相关；“20 组”不等于 20 种条件。
- 现有进程内 `frame_id` **可以保留**。最小扩展建议（不冻结 G2 接口）：保留 `frame_id`；采集侧增加 `session_id`/采集组标识 + 采集时刻（墙钟 + 单调时钟）；落盘记录 `frame_id`、session/组、采集时刻、文件哈希。哈希用于溯源/查重，**不证明新鲜度**。
- G0-R1 的“G2 必须另建帧身份，不能复用”和“帧身份不足……必须另建”收窄为：**可扩展现有编号，不必重建全套系统**；G2 具体设计仍未发布。

#### R2-3（P3）ZIP 与素材来源的证据等级

- ZIP 实际 **201 个条目**：200 个 JPG + 1 个目录条目 `仪表识别/`；没有脚本/标定。G0-R1 的“非 jpg 条目为 0”更正为“非 JPG **文件**为 0，但含 1 个目录条目”。
- 文件名与旧 `Session.save` 命名规则一致，只能支持“来源推测为旧拍照模式”；不能单凭文件名证明相机直出、确切 JPEG 质量（q≈95）或采集参数。代表帧 `getexif()` 为 0 条，无采集元数据；1280×720 与文件时间只作旁证。
- 因此“相机原帧（JPEG q≈95）”降级为“与旧保存规则一致的 1280×720 JPEG，来源待采集日志/记录确认”。

#### 被替代/收窄的 G0-R1 表述

| G0-R1 位置 | 旧表述（摘要） | G0-R2 处理 |
|---|---|---|
| G0-3 “蓝段/指针” | 彩环左上蓝段、指针约 11 点，作为可用证据 | 仅温度部分成立；湿度小环/短针“可见但未验证” |
| G0-3 “数量/格式” | 200 个 SHA-256 全不同，无重复帧 | 仅排除字节相同；不支持“无重复帧/新鲜” |
| G0-3 “来路” | 相机原帧（JPEG q≈95） | 降级为命名规则一致的 JPEG，来源待证 |
| G0-3 “真实直径” / G0-7 | 约 119 px 被当作表盘尺度 | 明确为整表外径；湿环明显更小、未定量；≥300 px 仍按整表口径 |
| G0-5 “帧标识” | G2 必须另建帧身份，不能复用 | 保留 frame_id，加 session/采集时刻即可作起点 |
| G0-7 “帧身份不足” | G2 必须另建 | 同上，收窄为可扩展现有编号 |
| 文末“材料请求” | 要求用户附 SHA-256 | 用户只需清晰原图及来源；SHA-256 由代理读取后计算 |

#### 材料请求（更正后，仍只一项）

请提供附件 §2 所用的 900 px 原始样张，或等效的清晰正面原图；**只需给出文件/来源，哈希由代理读取后计算**。用途：核对轴心/湿环/蓝色段并建立正式 11 点角度表。本轮无需新增素材即可完成；G1 仍未发布。

### G1a-R1 紧凑审核包（G1a 数学内核）

| 字段 | 内容 |
|---|---|
| 任务/回报编号 | G1a-R1 |
| 本轮结论 | **待审核**（不是审核通过；G1b 未发布、未启动） |
| 版本 | 实际基线 HEAD `2e2374fd974834366249a2034a948a9dbfd6cf48`；A03 记录为 `1b700c67ef70859692b7a0ff105a5fa70e214a4b`。当前 `gauge.py`/`capture_photos.py`/README 哈希与 A03 一致且无本地差异。未提交。新增文件 SHA-256：`dial_math.py` `08c5067a630a61414042b8dedf3b70188ea9eb2493d767e39efdda567f764d9d`；`test_dial_math.py` `c39bca9d2131a97835d03ce8031ea06328f206c57a22637bd335bea474ecbb6a`。本轮开始前本文 SHA-256 `37ce3246d9facd94c8cb86b460c303b802659abf834a4de2a4b1052ddca7f65a`。 |
| 改动 | 1. 新增 [dial_math.py](<src/photo_capture/scripts/dial_math.py#L75>)：`validate_scale`（L75）、`wrap_deg`（L88）、`relative_angle`（L100）、`lookup_value`（L125）。2. 新增 [test_dial_math.py](<src/photo_capture/tests/test_dial_math.py>)：22 项 unittest。3. [G1 状态](<docs/gauge_reading_handoff.md#L143>) 改为“G1a 已回报，待审核”。4. **未改**已有代码/配置/GUI/ROS/导航/相机/依赖；未 commit/push。 |
| 验证 | `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -v` → exit 0，Ran 22 tests, OK；`git diff --check` → exit 0；新文件无尾随空白；无 `dial_math` 的 `.pyc`。基线核对：三个文件哈希同 A03，`waypoint_gui.py` 仍 `024b6948…` 未改。 |
| 数据/标定 | **仅合成夹具**：测试内 11 点非等距 `SYNTHETIC_*` 表，不是 deli No.0011 正式标定，不产生真实 `%RH`；未创建 calib JSON。 |
| 证据 | [dial_math.py](<src/photo_capture/scripts/dial_math.py>)、[test_dial_math.py](<src/photo_capture/tests/test_dial_math.py>)。无图像/叠图；本轮不做图像识别。 |
| 风险 | 仅数学/结构校验：不含环/针/蓝段检测、椭圆、多帧、置信度或图像拒识；合成表通过不代表真实标定或精度；端点 1e-9° 吸附是浮点端点处理，不是测量容差；G1b 仍需清晰原图。 |
| 下一步 | 交审核方复核；不启动 G1b/G2–G4。 |
| 需要裁决 | 无新增；G1b 材料需求沿用 A03/D06，另行发布。 |

**验证命令**

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -v
# exit 0；Ran 22 tests ... OK
git diff --check
# exit 0
! grep -n '[[:blank:]]$' src/photo_capture/scripts/dial_math.py src/photo_capture/tests/test_dial_math.py \
  && echo 'no trailing whitespace'
# exit 0；no trailing whitespace
find src/photo_capture -name '*dial_math*.pyc'
# exit 0；无输出
```

**实现边界与必测对应**

- `validate_scale`：11 点 list/tuple、有限实数、排除 bool/str/嵌套、严格递增、跨度 (0,360)、数值 0…100；非法 `ValueError`。
- `wrap_deg`：[0,360)，负数/多圈；非法 `ValueError`。
- `relative_angle`：严格按 `zero + wrap_deg(needle - roll - zero)`；正/负滚转、跨 0°、整圈抵消；中间溢出 `ValueError`。
- `lookup_value`：不 wrap、不夹紧；闭区间分段线性；仅端点 ≤1e-9° 吸附；量程外 `None`；非法 `ValueError`。
- 测试文件覆盖 6 项必测清单；固定种子测试名称/注释明确“非图像合成/非真实精度”。合成表只存在于测试文件，不创建可加载默认标定。
- 仅用 Python 标准库与 `unittest`；无 numpy/cv2/ROS/Tk/模型导入。导入测试在空临时目录子进程运行：无 `gauge`/`yolo_detect`/`ultralytics`/`rospy`/`tkinter`、无线程、无落盘。
- 未触碰既有改动：`waypoint_gui.py` 与 3 个未跟踪文件（`docs/electric_scooter_video_counting.md`、`src/MLX90640_交付包.zip`、`inspection_waypoints_before_straight.yaml`）状态同开工前。

**过程说明**：开发中初次运行有 3 个失败（端点吸附实现、1 个负滚转夹具 `needle` 算错），修正后 22/22；无遗留失败。未安装 pytest，未运行相机/ROS/图像测试。

### G1a-R2 定向修复审核包（端点吸附）

| 字段 | 内容 |
|---|---|
| 任务/回报编号 | G1a-R2 |
| 本轮结论 | **待审核**（不是审核通过；G1b 未发布、未启动） |
| 版本 | 实际基线 HEAD `99104cc3ed6e8b45bdde264b6388420540f85258`（A04 文档提交）；A04 受审 HEAD 为 `2e2374f…`，当时 `dial_math.py` `08c5067a…`、`test_dial_math.py` `c39bca9d…`。本轮未提交。新文件 SHA-256：`dial_math.py` `05aabd4ab4998041387760270c2966d7776a61c695fcde6d9baa1b2f61cef6d0`；`test_dial_math.py` `ab206cfdbbf50b693ffbc270822342b0db4756ee21c1fb28ba8dc5c6eb6af081`。本轮开始前本文 SHA-256 `49db2863555807722ee56e8f13d408fc9a2c12746a0d1ae69f13dcf0bd82743c`。 |
| 改动 | 1. 仅改 [lookup_value 端点分支](<src/photo_capture/scripts/dial_math.py#L141-L146>)：量程**外**且距端点 ≤1e-9° 才吸附；量程内不再提前返回 0/100，正常走节点/分段插值。2. 更新旧吸附测试为“仅外侧吸收”，并新增“内侧不被吸收”测试。3. 新增 A04 三类合法窄间隔表的 4 项回归。4. **未改**其他接口/`validate_scale`/已有代码/配置/GUI/ROS/导航/相机/依赖；未 commit/push。 |
| 验证 | 原 G1a 命令 → exit 0，**Ran 27 tests, OK**（原 22 项 + 5 项回归/更新）；`git diff --check` → exit 0；新文件无尾随空白、无 `dial_math` 的 `.pyc`。旧 YOLO 三文件哈希仍同 A03；`waypoint_gui.py` 仍 `024b6948…` 未改。 |
| 数据/标定 | **仅合成夹具**；A04 三表均为合法合成表并已加入回归，不是 deli No.0011 正式标定，不产生真实 `%RH`；未创建 calib JSON。 |
| 证据 | [dial_math.py](<src/photo_capture/scripts/dial_math.py#L125>)、[test_dial_math.py](<src/photo_capture/tests/test_dial_math.py>)。A04 三例结果见下表。 |
| 风险 | 仅数学/结构校验；不含环/针/蓝段检测、椭圆、多帧、置信度或图像拒识；合成表通过不代表真实标定或精度；外侧 1e-9° 吸附只是浮点端点处理，不是测量容差；G1b 仍需清晰原图。 |
| 下一步 | 交审核方复核；不启动 G1b/G2–G4。 |
| 需要裁决 | 无新增；G1b 材料需求沿用 A03/D06，另行发布。 |

**A04 三例回归结果**

| 合法合成角度表 | 查询 | 修复前 | 修复后/预期 | 本轮回归 |
|---|---|---|---|---|
| `[0, 5e-10, 30, …, 270]` | 第 1 号节点 `5e-10` | 0 | **10** | `test_first_narrow_interval_keeps_node_and_interpolates` |
| `[0, 30, …, 270, 270+5e-10]` | 第 9 号节点 `270` | 100 | **90** | `test_last_narrow_interval_keeps_node_and_interpolates` |
| `[i*1e-11 for i in range(11)]` | 最后节点 `1e-10` | 0 | **100** | `test_tiny_table_keeps_all_nodes_and_interpolates` |

- 三表均先经 `validate_scale` 接受；未新增最小节点间隔，未拒绝合法表，未放宽节点断言（节点用 `assertEqual`）。
- 窄间隔中点也做了独立预期：`2.5e-10→5.0`、`270+2.5e-10→95.0`、`5.5e-11→55.0`。
- 保留回归：外侧 `±5e-10`/`±1e-9` 吸附、超过阈值返回 `None`、不自动 wrap、宽间隔非线性表、非法输入/表。

**验证命令**

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -v
# exit 0；Ran 27 tests ... OK
git diff --check
# exit 0
! grep -n '[[:blank:]]$' src/photo_capture/scripts/dial_math.py src/photo_capture/tests/test_dial_math.py \
  && echo 'no trailing whitespace'
# exit 0；no trailing whitespace
find src/photo_capture -name '*dial_math*.pyc'
# exit 0；无输出
```

**实现说明**：`lookup_value` 现在只在 `angle < first - 1e-9` 或 `angle > last + 1e-9` 时返回 `None`；`angle <= first`/`angle >= last` 仅吸收外侧或端点本身的输入；严格位于 `(first, last)` 的角度统一进入 `bisect`/节点判断/分段插值，因此窄间隔节点不会被固定端点带吞并。未改模块其他函数。

### G1b0-R1 候选标定审核包

| 字段 | 内容 |
|---|---|
| 任务/回报编号 | G1b0-R1 |
| 本轮结论 | **待审核**（不是正式标定通过；G1b1 未发布、未启动） |
| 版本 | 实际基线 HEAD `6187dd03c2b835258314d0d35e630b6c5f3b9a50`；未提交。参考图 `be71a511…`（1279×1706，205,467 B）；标注 `deli_0011_annotations.json` SHA-256 `90639daf3ec40ceda84a85875c5ceee8569989e7f39f6c12fc462962ad05b331`；工具 `dial_calibration.py` `114dae7f8bf7de63d99f0fb37181321ce3bd91474b75bab0ee0fc2f44055b9c1`；测试 `test_dial_calibration.py` `5828096574c3282c7bd91ca315d1bcc655097e6159a7090b3ccb11ad26f313df`；候选 JSON `897fe4f9ae459387e8b1a2f21fef2e19371e6a629371d5ab4b5e5535237d3864`；叠图 `468472fbc2c7047b7e3b163c2d7204ee2004ae5eee08fdba049c854b01ac7b89`。G1a 两文件哈希保持 `05aabd4a…` / `ab206cfd…` 未变。 |
| 改动 | 1. 新增 [dial_calibration.py](<src/photo_capture/scripts/dial_calibration.py>)：校验哈希/尺寸/点/环，拟合椭球环、构造圆化矩阵、生成候选表、灵敏度与叠图。2. 新增 [test_dial_calibration.py](<src/photo_capture/tests/test_dial_calibration.py>)：16 项合成夹具测试。3. 新增 [deli_0011_annotations.json](<src/photo_capture/config/dial/deli_0011_annotations.json>)：11 刻度、湿度轴、蓝段边界、29 个同边缘环点。4. 本地输出 `candidate_calib.json` 与 `calibration_overlay.png`。5. 未改旧 YOLO/GUI/ROS/数学内核；未装依赖；未 commit/push。 |
| 验证 | CLI 命令 → exit 0；候选表/叠图已生成。`test_dial_calibration` → exit 0，Ran 16 tests, OK；`test_dial_math` → exit 0，Ran 27 tests, OK；`git diff --check` → exit 0；新文件无尾随空白、无 `dial_*` 的 `.pyc`。源图哈希/尺寸与 A06 一致。 |
| 数据/标定 | 参考图：[A06 原图](</home/mowen/.dsh/attachments/v1/objects/be/be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30>)；标注见上。**仅有 candidate，没有正式 `calib.json`，没有真实湿度读数/精度**。 |
| 证据 | [candidate_calib.json](<.gauge_calibration_review/candidate_calib.json>)、[calibration_overlay.png](<.gauge_calibration_review/calibration_overlay.png>)（本地复核目录，不入 Git）。叠图含原图刻度/轴/环/蓝段标注与归一化视图。 |
| 风险 | 标注为人工/半自动，刻度不确定度约 ±1.5–2 px；近圆椭圆长短轴比 1.0138，未用长轴作滚转参考；参考图湿度针在顶部空白区，**未输出有效湿度**；候选表不是实拍精度。 |
| 下一步 | 交审核方逐项复核；**不启动 G1b1/G2–G4**。 |
| 需要裁决 | 无新增；审核方签发前不进入 G1b1。 |

**几何与标注结果**

| 项 | 结果 |
|---|---|
| 湿度轴心 | `[647.6, 1058.0]`（金色轴销质心） |
| 环拟合中心 | `[648.546, 1056.478]` |
| 环拟合长短轴（直径） | `359.165 / 364.138 px`，角度 `117.556°`；外边缘 |
| 半轴定义 | 长半轴 `182.069 px`、短半轴 `179.582 px`；长轴方向 `27.556°` |
| 轴心-环心偏移 | 原始 `[-0.946, 1.522] px`；归一化 `[-0.00526, 0.00848]`，模 `0.00998` |
| 环拟合残差 | RMS `0.816 px`，最大 `2.736 px` |
| 蓝段边界角 | 绿-蓝 `166.105°`；蓝端 `226.480°` |
| 蓝段短弧/中点 | `60.375°`；`roll_ref_deg = 196.292°` |
| 可疑点 | 环点 #23 `[524.0, 927.8]` 残差 `2.736 px`（端点附近） |

**候选 11 点表**（实测点经同一圆化矩阵、按 0→100 展开；`validate_scale` 已通过）

| 值 | 候选角度 | 附件常量 | 差值 |
|---|---|---|---|
| 0 | 314.310 | 314.70 | -0.390 |
| 10 | 350.715 | 351.50 | -0.785 |
| 20 | 384.575 | 383.70 | +0.875 |
| 30 | 415.426 | 414.40 | +1.026 |
| 40 | 444.269 | 444.10 | +0.169 |
| 50 | 472.785 | 473.30 | -0.515 |
| 60 | 499.836 | 501.00 | -1.164 |
| 70 | 526.592 | 527.50 | -0.908 |
| 80 | 551.689 | 552.60 | -0.911 |
| 90 | 573.030 | 572.50 | +0.530 |
| 100 | 585.336 | 587.15 | -1.814 |

- 候选总跨度 `271.026°`，附件 `272.45°`，差 `-1.424°`。
- 附件常量仅作差异对照，**没有参与候选表计算，也没有据其自动校正**。
- 附件环常量 `340.4/345.5 px` 与本图外边缘拟合 `359.2/364.1 px` 的差异主要来自**边缘定义/相机**；本轮按规范选“同一边缘”的外边缘，不混用其他边缘。

**灵敏度与诊断**

| 项 | 结果 |
|---|---|
| 单点 ±1 px 最大角度变化 | `0.390°`（刻度/轴） |
| 蓝段边界 ±1 px 角度变化 | `0.329°` |
| 蓝段中点 ±1 px 变化 | `0.164°` |
| 湿端值 ±1 px 变化 | `0.294 %RH/px`（诊断，不是实测精度） |
| 环拟合残差 | RMS `0.816 px`，最大 `2.736 px` |
| 警告 | 近圆（比 1.0138，不用长轴作滚转参考）；环点 #23 残差 >2 px；参考湿度针在空白区 |

**验证命令**

```bash
PYTHONDONTWRITEBYTECODE=1 python3 src/photo_capture/scripts/dial_calibration.py \
  --image /home/mowen/.dsh/attachments/v1/objects/be/be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30 \
  --annotations src/photo_capture/config/dial/deli_0011_annotations.json \
  --output-dir .gauge_calibration_review
# exit 0；candidate_calib.json + calibration_overlay.png

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_calibration.py' -v
# exit 0；Ran 16 tests ... OK
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -v
# exit 0；Ran 27 tests ... OK
git diff --check
# exit 0
```

**未验证/边界**

- 只验证“这张图能产出几何自洽、可复核的候选”，**未实现运行时单帧检测**；无相机/ROS/模型调用，无实机许可。
- 候选表未做真实图像识别、低分辨率/反光/斜视/多帧测试；不能宣称 `%RH` 精度、延迟或跨表通用。
- 参考图湿度针指向顶部空白区；本轮未计算该针角、未输出读数，也未把其当量程内真值。
- 输出目录 `.gauge_calibration_review/` 是本地复核产物，**未提交**；候选不能作为默认正式 `calib.json`。

### G1b0-R2 定向修复审核包（输出安全/原子写入/零向量）

| 字段 | 内容 |
|---|---|
| 任务/回报编号 | G1b0-R2 |
| 本轮结论 | **待审核**（不是正式标定通过；G1b1 未发布、未启动） |
| 版本 | 实际基线 HEAD `36b550991401f285eccaf4afb97e0b8853f12052`；本文基线 SHA-256 `89cd3e5f76d8520ac4fc5ed67f638d450915198efa9f32c7b613cec64ed28ebc`。工具 `dial_calibration.py` `1dacb5c9bdec9bc642c291badf18c645f91cd7fdaadd5a875c67440ec710663e`；测试 `test_dial_calibration.py` `7f1e59089a15786ae3e3a887b623d8124bf1c9283509b7a2bb9edb7b28f76408`；标注仍为 `90639daf…`；数学内核/测试仍为 `05aabd4a…`/`ab206cfd…`；源图仍 `be71a511…`。R2 候选 `d7dccc3e6d80f24199405dce07a516fd119d4502724efa413983682d2e5df198`；R2 叠图 `468472fb…`；旧 R1 候选/叠图 `897fe4f9…`/`468472fb…` 未变。 |
| 改动 | 1. `run` 输出目标改为“不存在或空目录”，非空/文件/符号链接/输入同路径或位于输出内一律拒绝；**不再删除任何旧候选/叠图/输入**。2. 用临时文件写出叠图/JSON，检查 `cv2.imwrite` 返回值与异常、JSON 写入异常，二者都成功后才发布；失败只清理本次临时/半成品。3. `MIN_VECTOR_NORM_PX=1e-6`：刻度/蓝段相对湿度轴心的零向量在 `validate_annotations` 与 `angle_from_axis` 均抛 `CalibrationError`。4. 测试 16→24 项；旧的“删除旧候选”测试改为“非空输出不变”回归。5. 未改源图/原始标注/数学内核/YOLO/GUI/ROS；未装依赖；未 commit/push。 |
| 验证 | CLI 写入新空目录 `.gauge_calibration_review/r2-final` → exit 0；`test_dial_calibration.py` → exit 0，**Ran 24 tests, OK**；`test_dial_math.py` → exit 0，**Ran 27 tests, OK**；`git diff --check` → exit 0；新文件无尾随空白、无 `dial_*` 的 `.pyc`。旧 R1 候选/叠图哈希运行前后不变。 |
| 数据/标定 | 源图/标注不变；候选 11 点表与 R1 相同（`[314.310, 350.715, 384.575, 415.426, 444.269, 472.785, 499.836, 526.592, 551.689, 573.030, 585.336]`）；仅新增灵敏度统计口径。仍无正式 `calib.json`、无真实湿度读数/精度。 |
| 证据 | [R2 candidate](<.gauge_calibration_review/r2-final/candidate_calib.json>)、[R2 overlay](<.gauge_calibration_review/r2-final/calibration_overlay.png>)。旧 `.gauge_calibration_review/` R1 及修复前的 `.gauge_calibration_review/r2` 均未删；最终链接是 `r2-final`。 |
| 风险 | 仅修工具安全和无效几何；未实现运行时检测、无相机/ROS/模型、无实拍精度；标注仍为人工/半自动，近圆椭圆与参考湿度针空白区限制同 R1；输出目录策略要求每次用新空目录，无 `--force`。 |
| 下一步 | 交审核方复核；**不启动 G1b1/G2–G4**。 |
| 需要裁决 | 无新增；审核方签发前不进入 G1b1。 |

**故障回归覆盖（24 项中新增/改写）**

| 故障 | 结果 |
|---|---|
| 输出目录非空（含旧 `candidate_calib.json`） | 拒绝；旧文件内容/数量不变 |
| 输出不存在 / 已存在空目录 | 允许；成功写出两份最终文件 |
| 标注在输出目录内 / 输出等于图像路径 | 拒绝；输入哈希不变 |
| `cv2.imwrite` 返回 False | `CalibrationError`；无候选/叠图/临时文件 |
| `cv2.imwrite` 抛异常 | `CalibrationError`；同上 |
| JSON 写入失败（`Path.write_text` OSError） | `CalibrationError`；叠图临时文件清理，无半套产物 |
| 刻度点与湿度轴心重合 | `CalibrationError`；不产候选 |
| 蓝段边界与湿度轴心重合 | `CalibrationError`；不产候选 |
| `angle_from_axis` 零向量 | `CalibrationError` |

**灵敏度统计口径（A07 非阻塞备注）**

- 本轮 `sensitivity` 新增计数：`interpolable_tick_perturbations=40`、`out_of_range_tick_perturbations=4`、`interpolable_axis_perturbations=4`、`out_of_range_axis_perturbations=0`。
- `wet_end_value_delta_per_px=0.293887` 仅对 `lookup_value` 非 None 的扰动统计；量程外扰动单独计数，**不当零误差**。该数字仍只是诊断，不是总体精度上界。
- 四方向 ±1px 仍是有限采样，不是完整误差模型；边缘/相机差异仍只是解释假设，未认定为已证实。

**验证命令**

```bash
PYTHONDONTWRITEBYTECODE=1 python3 src/photo_capture/scripts/dial_calibration.py \
  --image /home/mowen/.dsh/attachments/v1/objects/be/be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30 \
  --annotations src/photo_capture/config/dial/deli_0011_annotations.json \
  --output-dir .gauge_calibration_review/r2-final
# exit 0

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_calibration.py' -v
# exit 0；Ran 24 tests ... OK
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -v
# exit 0；Ran 27 tests ... OK
git diff --check
# exit 0
```

### G1b1-R1 指定 ROI 单帧验证审核包

| 字段 | 内容 |
|---|---|
| 任务/回报编号 | G1b1-R1 |
| 本轮结论 | **待审核**（不是 G1 通过；G2–G4 未发布、未启动） |
| 版本 | 实际基线 HEAD `33740bda1e346dc4c522f83ad6ee364184c8df5a`；本文基线 SHA-256 `b278b6fca9caab54fb684c9b7271cb0beaff3009619f202c88d4f18ef2b41912`。工具 [dial_reading.py](<src/photo_capture/scripts/dial_reading.py>) SHA-256 `88735b76822584139200882f600cc89f7f5f3aec8b687c00b4d49a62e4ad59ba`；测试 [test_dial_reading.py](<src/photo_capture/tests/test_dial_reading.py>) `e8e7d103d898ef7913e50d9e68507ede482800820745a8cbd44e4e791eb75637`。A08 候选仍 `d7dccc3e…`，源图仍 `be71a511…`，标注/数学内核未变。 |
| 改动 | 1. 新增 `dial_reading.py`：显式 ROI、候选哈希校验/`--allow-candidate`、环外缘检测、rho=0.94 蓝段、轴心迁移、暗度/覆盖率/峰竞争、单帧诊断与调试叠图。2. 新增 `test_dial_reading.py`：17 项合成/配置/CLI 回归。3. 追加本文/状态。4. 未改旧 YOLO/GUI/ROS/标注/数学内核；未装依赖；未 commit/push。 |
| 验证 | 参考图 CLI → exit 0；结果为 `frame_status=low_conf`、`reason=axis_roll_inconsistent`（0.507° >0.5°），**参考图检测未完成**，未输出 candidate/out_of_range。`test_dial_reading` → exit 0，**17 tests OK**；`test_dial_calibration` → exit 0，**24 tests OK**；`test_dial_math` → exit 0，**27 tests OK**；`git diff --check` → exit 0；新文件无尾随空白、无 `dial_reading` 的 `.pyc`。 |
| 数据/标定 | 输入为 A08 候选（`d7dccc3e…`，`--allow-candidate` + SHA 校验）和源图 `be71a511…`；ROI=`430,870,420,390`。仅有候选开发参考，无正式 `calib.json`、无真实湿度真值。 |
| 证据 | 参考失败叠图：[debug_overlay.png](<.gauge_reading_review/g1b1-reference/debug_overlay.png>)、[frame_result.json](<.gauge_reading_review/g1b1-reference/frame_result.json>)。合成叠图：[成功](<.gauge_reading_review/g1b1-synthetic/debug_success/debug_overlay.png>)、[量程外](<.gauge_reading_review/g1b1-synthetic/debug_out_of_range/debug_overlay.png>)、[蓝段缺失](<.gauge_reading_review/g1b1-synthetic/debug_blue_missing/debug_overlay.png>)、[双针竞争](<.gauge_reading_review/g1b1-synthetic/debug_competition/debug_overlay.png>)。 |
| 风险 | 参考图因轴心迁移 roll 差 0.507° 被拒识，未达到 `out_of_range` 演示；这是质量拒识，不是识别成功。合成测试不等于实拍；无多帧、无自动全图找表、无相机/ROS、无实机精度/延迟。单帧即使 ok 也只给 `candidate_value`，`value=null`。 |
| 下一步 | 交审核方复核；**不启动 G2/G3/G4**。 |
| 需要裁决 | 无新增代码问题；请审核方判断是否接受参考图 `axis_roll_inconsistent` 拒识、以及是否要求调轴心迁移/蓝段测量后再做参考图 `out_of_range` 演示。 |

**参考图实测（失败但可复核）**

| 项 | 结果 |
|---|---|
| ROI / 输入 | `[430, 870, 420, 390]`；1279×1706 原图 |
| 环检测 | 中心 `[648.428, 1056.338]`；长/短半轴 `182.178 / 179.677 px`；残差 RMS `1.428 px`；外缘覆盖 291 bins |
| 蓝段 | 边界 `165.5° / 226.1°`，run `60.65°`，环心 midpoint `195.8°` |
| 轴心迁移 | `roll1=359.508°`，`roll2=0.015°`，差 `0.507° > 0.5°` → 拒识；未运行针检测 |
| 输出 | `frame_status=low_conf`、`status=low_conf`、`reason=axis_roll_inconsistent`、`candidate_value=null`、`value=null` |
| 结论 | **参考图检测未完成**；失败叠图已保留，未把拒识写成识别成功 |

**合成验证范围**

- 12 个角度/值案例：`302→2`、`320→10`、`330→15`、`340→20`、`360→30`、`380→40`、`400→50`、`420→60`、`440→70`、`460→80`、`480→90`、`498→98`；量程内 `candidate_value` 误差断言 ≤2 %RH。
- 滚转 `-6°/0°/+6°` × 整表缩放 `300/450/900 px`；外加 `sigma=1.2` 模糊。
- 反例：无盘、蓝段缺失、蓝段跨 0°、双暗线竞争、多环歧义、ROI 回映、近圆轴交换、配置错误、输出失败。
- 合成表/指针/颜色均显式为 fixture，不是参考图真值；不计算真实 MAE/成功率。

**实现边界**

- CLI：`python3 src/photo_capture/scripts/dial_reading.py <image> --calib <JSON> --allow-candidate --roi x y w h --debug-dir <新的空目录>`；可选 `--calib-sha256` 供合成测试覆盖默认 A08 哈希，默认即 A08。
- `estimate_frame(image_bgr, calibration, roi)` 为纯函数，不读写文件、不起线程；CLI 负责读图/配置/落盘。
- 输出目录必须不存在或为空；写图/JSON 失败非零，失败只清理本次临时/半成品，不覆盖源图或旧成果。
- 候选值只在 `candidate_value`；顶层 `value=null`；单帧 ok 也只 `status=low_conf`、`reason=multi_frame_required`。
- 未实现自动全图找表、多帧、GUI、ROS、相机/底盘接入。

**验证命令**

```bash
PYTHONDONTWRITEBYTECODE=1 python3 src/photo_capture/scripts/dial_reading.py \
  /home/mowen/.dsh/attachments/v1/objects/be/be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30 \
  --calib .gauge_calibration_review/r2-final/candidate_calib.json --allow-candidate \
  --roi 430 870 420 390 --debug-dir .gauge_reading_review/g1b1-reference
# exit 0；frame_status=low_conf, reason=axis_roll_inconsistent

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_reading.py' -v
# exit 0；Ran 17 tests ... OK
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_calibration.py' -q
# exit 0；Ran 24 tests ... OK
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -q
# exit 0；Ran 27 tests ... OK
git diff --check
# exit 0
```

### G1b1-Flash-R2 / R3 本会话派工回报（Lead转录摘要）

| 项 | R2：统一角度基准 | R3：修复色相插值 |
|---|---|---|
| 执行方式 | Workflow，deepseek-official / deepseek-flash | 同左，用户已明确批准Workflow |
| 写入范围 | 单帧模块/测试、新本地复核目录；未写本文 | 同左；未改生成/绘针/擦针/参数矩阵 |
| 单帧测试 | 26项通过 | 28项通过（另保留24标定+27数学） |
| 参考原图 | 正确out_of_range、针272.7°、value=null | 同左，保持不回归 |
| 真实底图合成217例 | 181接受/36拒绝，24个错误候选ok | 187接受/30拒绝，0个错误候选ok |
| 接受案例≤2 %RH | 157/181 | 187/187，最大误差1.914609 %RH |
| 低分辨率2帧 | 均no_dial | 均no_dial，未放大冒充真实分辨率 |

- R2修复`roll1`环心/轴心基准混用，新增非零偏移手算/滚转/ROI回归；原图roll迭代差0.507°→0.001848°，没有放宽0.5°门槛。调试指针线也经inv(A)映射回原图。
- R2真实底图合成第一次暴露系统性错误：在scale1.75+blur1.2条件下，H=179/0附近的红像素被HSV标量插值成假蓝段，导致roll约错170°。Lead独立单例诊断确认，随后直接在本会话派R3，不再要求用户搬运回报。
- R3只把`sample_blue`改为先采BGR条带、再转HSV，其余颜色/覆盖率/峰竞争/roll门槛不变；增加红色跨色相零点负控和真实蓝段跨角度零点正控。原24错误候选消失：18例转正确接受、6个端点转保守拒识；另12个原拒识中间值变正确接受。未靠过滤失败条件改善指标。
- 最终源文件SHA-256：[dial_reading.py](<src/photo_capture/scripts/dial_reading.py>) `9fa50259d2f3ab07093c369d7d78b6acee64d717754b2a703240ce916f994d66`；[test_dial_reading.py](<src/photo_capture/tests/test_dial_reading.py>) `78a43d3f0f1324d121af4d1018fb960ae8b36298a46e445d11a945836b18bf0c`。没有commit/push，没有相机/ROS/旧YOLO改动。
- 完整证据：[R2报告](<.gauge_reading_review/g1b1-flash-r2/real-eval/eval_report.json>)、[R3报告](<.gauge_reading_review/g1b1-flash-r3/real-eval/eval_report.json>)、[原图检出叠图](<.gauge_reading_review/g1b1-flash-r3/real-eval/reference/debug_overlay.png>)、[R3原失败条件修复叠图](<.gauge_reading_review/g1b1-flash-r3/real-eval/synthetic/overlay_fixed_roll-06.0_scale1.75_val010.0_blur1.2.png>)。旧报告保留，不覆盖。
- 限制：216例量程内合成中，0/100端点共36例有29例拒识；另1例原针角本应量程外，总拒识30。10–90及15共180例均在该合成集通过。底图只有一张，擦针重建影响局部文字，上采样不增加真实分辨率，不能当作真实湿度准确率或跨相机验证。两张旧小车图无法检出湿度环；无独立实拍真值，未做真实MAE或现场延迟验收。

## 8. 审核区（决策方追加）

### A00：初始化

- 本轮只建立交接文档，未执行 G0，不把决策方已有调查冒充 DS 的交付。
- 发布 G0；G1–G4 暂未发布。
- 无算法验收结果；无相机/ROS/底盘运行测试；无实机放行。
- 后续审核记录须包含：回报编号、所审版本、复核证据、结论、修改要求、解锁任务及边界。

### A01：G0-R1 审核——需修改，未解锁 G1

**受审版本**：HEAD `78b1984852b362fb86c907aa55c78bb1532fdee6` 上的未提交 G0-R1；审核前本文 SHA-256 `c13828a16a63ca48b8f5c3ad68ce19988baa1a3f6b66e69416a20be94813d280`。G0-R1 原文保留，冲突处以下述复核与 D05 为准。

**独立复核通过**：

- 旧算法/入口/说明三个文件哈希与回报一致；本文基线哈希与提交中版本一致。Git差异显示 G0 回报只修改本文，旧有无关改动仍在；未发现新增识别代码。
- 仓库内再次按名称检索，仍未找到附件列出的四个脚本/标定。此结论限已检查范围，不推断用户所有环境均不存在。
- 读取两张代表原图，确认均为1280×720、场景近似一致且存在上下两个独立指针轴心。读取全部200图，尺寸一致，文件哈希200个不同。
- 复跑回报中的外表盘Hough测量：10个候选，直径中位118.92 px，范围113.76–129.60 px。此临时ROI仅用于复核统计，不批准作为运行时硬编码定位。
- 压缩包大小58,801,104 B及SHA-256与回报一致；实际为**201个条目：200张JPG和1个目录条目**，没有脚本或标定。
- 独立导入并确认Python 3.8.10 / NumPy 1.24.4 / cv2 4.12.0（GUI NONE）；三个OpenCV发行包并存，pytest未安装。未安装/替换任何包。
- 未复跑无关热像单元测试，也未开启相机/ROS；其26项通过不能代替湿度识别测试。本轮所有核查命令exit 0；无湿度精度/性能验收结果。

**必须更正的发现**：

1. **P1：温湿度目标混淆。** G0-3将“外圈蓝红环、指针约11点”写作本任务可用蓝段/指针证据；这主要描述温度长针与温度环，没有分别核查下方湿度环与湿度针。119 px又是整表外径，不能等同湿度环尺度。必须分别说明两轴/两环、目标湿度指针的可辨认性及不确定性，不强猜当前湿度值。未澄清前不允许开始找针实现。
2. **P2：哈希与帧身份推论过强。** “200个SHA不同”只说明无字节完全重复文件；不是独立/新鲜帧证明。“旧frame_id必须另建、不能复用”的结论没有依据，按D05改为可扩展现有编号。重读已有帧处理函数即可，不需要开发查重系统。
3. **P3：材料清单与来源措辞。** ZIP实际包含一个目录条目，纠正“非jpg条目为0”；“文件名符合旧保存规则”只支持来源推测，不能单凭命名证明相机原帧及确切编码质量。未有额外证据就降级措辞。

**定向返工 G0-R2（本轮唯一发布项）**：

- 只在DS回报区追加一份简短更正，逐项对应上述3条；保留G0-R1原文，注明哪些旧表述被替代。完成后将G0状态改为“待审核（G0-R2）”。
- 使用现有代表原图，分别记录整表/温度部分/湿度小环的目标含义。湿环尺寸如无法可靠量取，允许写“明显小于整表、未定量”；不得为填数字编造测量，不要求训练模型或写检测程序。
- 给出未来最小帧身份建议：会话/采集组 + 现有frame_id + 采集时刻；解释哈希能证明什么、不能证明什么，不冻结G2接口。
- 修正ZIP和素材来源的证据等级；现有缺源码/缺正式标定结论可沿用，不再全home检索，不再运行无关测试。
- **验收条件**：3条逐项回应，目标湿度与温度不再混用，没有新算法/配置/相机/环境变更；无需用户新增素材即可完成。回报后停下等审，不重复长日志。

**下一阶段裁决**：G1仍未发布。G0-R2通过后，可优先考虑“G1a数学/标定校验与合成拒识”，避免等待素材期间空转；正式单帧湿度读数仍需清晰标定图。用户只需提供附件原始样张或等效清晰正面原图，不要求自行计算哈希。现有119 px整表图暂用于低分辨率拒识材料，不作真实精度真值。无任何实机许可。

### A02：G0-R2 审核通过；仅发布 G1a

- **受审版本**：实际HEAD `e2a9a8d60f784db1a3c052261de773ef9438ed4a` + 未提交R2；审核前本文SHA-256 `e49f343cc66d73296a043f0205355eb05678070a4a2149bcef025a533e4c6f62`。
- **结论：本阶段通过。** P1温湿度目标与尺度已分清，湿度针只按“可见、稳定测角未验证”处理；P2哈希证据与扩展frame_id已收窄；P3 ZIP与来源措辞已纠正。3项定向更正满足A01，无须再做G0-R3。
- **复核**：Git差异只有本文R2回报/状态变化及原有无关修改；旧算法与入口哈希、两张代表原图哈希同前次审核。独立确认两张图均1280×720、EXIF均0条，ZIP为201条目（200 JPG+1目录）。未重新运行整轮图像测量、相机/ROS或无关测试；核查命令均exit 0。
- **版本勘误（不阻塞）**：R2写的HEAD `78b1984…`是旧记录，实际本文基线哈希`be4afab5…`与当前提交`e2a9a8d…`完全一致。本审核按实际HEAD及文件哈希绑定，保留原回报，不为此再开返工；以后每轮从git重新读取HEAD。
- **新决策 D06**：正式标定图仍缺，但不必等待才能做纯数学工作；将G1拆分，仅发布第5.4节G1a。保持“无正式标定，不提供真实湿度读数”的边界。G1b须另行发布，G2–G4维持待发布。
- **下一步**：用户把本文首页更新后的启动指令交给DS，DS完成G1a-R1后停在待审核。尚未确认DS开始执行，不虚构进度。
- **材料**：后续G1b仍需清晰正面原图及来源，代理负责计算哈希；本轮G1a无需用户补材料。没有正式标定、图像识别精度或实机许可。

**审核收尾并发告警（优先于本节前述发布结论）**：本轮最初检查时旧仪表代码与R1指纹一致；最终校验期间却新增了旧仪表算法、GUI入口、README的大幅修改，随后GUI测试也变更，并出现未跟踪的`.gauge_test/`。这些不是本审核方写入，且超出G0-R2/G1a范围；来源及是否另有授权尚未确认。最终“原代码指纹不变”断言失败，不能把这些新补丁包含在本次通过结论中，亦不能声称最终代码检查通过。第二次Git差异检查通过仅证明无空白错误，不代表实现验收。**立即暂挂G1a并暂停新派工，等待用户确认并发任务关系；不覆盖、不回退、不提交这些外部改动。** G0-R2事实更正通过结论仅绑定上列受审版本，所有新代码均未审核；若用户要改审现有实现，应先等其写入完成并固定补丁版本。

### A03：独立YOLO任务结束，解除暂挂并恢复G1a

- **用户确认**：另一任务是独立的旧YOLO方案测试，与R2代理无关；先前要求等待，现明确通知已经完成、允许本路线继续。用户反馈“效果一般”，仅记录定性反馈，不推断误差数值、根因或训练质量，也不据此宣判所有YOLO方法无效。
- **新基线**：HEAD `1b700c67ef70859692b7a0ff105a5fa70e214a4b`，该提交已包含旧仪表方案的四个相关文件；这四个文件当前相对HEAD无本地差异。已读取[算法说明](<src/photo_capture/scripts/gauge.py#L3-L17>)及[使用说明](<src/photo_capture/README.md#L268-L289>)：当前为gauge.pt七关键点、六静态点单应、Radon/针尖估角、五刻度分段插值；不再是G0调查时的三点仿射版本。本次只确认版本边界，未进行该实现的完整代码/效果验收。
- **代码指纹**：当前[gauge.py](<src/photo_capture/scripts/gauge.py>) SHA-256 `a35f46f2ff872917a8fbd0da05e5116d811f8bf94a42b0c2d4b2e49e7d560e76`；[capture_photos.py](<src/photo_capture/scripts/capture_photos.py>) `c12931420aebf6a02e2bc7e134c98ad79bd01036e6824df3338da3ebc5ad1a19`；[README](<src/photo_capture/README.md>) `7c5217937e6dda1d4c2e503a143a38625763db8b9d781083964906c084b2fd90`。G0/A01/A02旧哈希保留为历史证据，不要求恢复旧文件。
- **新决策D07**：解除A02的并发暂挂，仅恢复第5.4节G1a。保留现有YOLO方案作独立对照，不在本任务里重训、调参、替换或回退它；新路线仍先验证非线性湿度数学内核，再另行发布湿环/短针图像验证，不能靠追加关键点和更换网络来替代证据。
- **执行边界**：DS仅新增G1a规定的两个文件并追加回报，开始前重新记录实际HEAD及现有无关改动。当前尚未发现这两个新增文件，不把任务发布视作DS已经开始。G1a本身不会改善实际图像效果，不能宣称读数算法已完成。
- **材料与后续**：G1a不等原图，可以开始；G1b仍需清晰正面原图与来源，用于确认湿度轴心/彩环及建立正式角度表。现有约119 px整表图继续作为低分辨率挑战材料，是否低于可用下限须测试，不承诺训练或传统方法能补回缺失图像信息。G2–G4未发布，无实机许可。
- **校验范围**：本轮只修改交接文档，不启动识别、相机或ROS。新基线指纹/差异检查不代表旧YOLO精度通过；后续G1a回报必须绑定自己的补丁版本。

### A04：G1a-R1 审核——需修改，仅一项代码缺陷

- **受审版本**：HEAD `2e2374fd974834366249a2034a948a9dbfd6cf48`；未提交的[dial_math.py](<src/photo_capture/scripts/dial_math.py>) SHA-256 `08c5067a630a61414042b8dedf3b70188ea9eb2493d767e39efdda567f764d9d`，[test_dial_math.py](<src/photo_capture/tests/test_dial_math.py>) `c39bca9d2131a97835d03ce8031ea06328f206c57a22637bd335bea474ecbb6a`；审核前本文SHA-256 `52bb4404e4c487689944db57f4459cd007b455f4735eced19c217bb9c651a581`。
- **已通过部分**：读取全部实现和测试；独立按规定命令复跑22项unittest，0.048秒、exit 0。函数分层、非法输入/溢出处理、纯标准库与无图像/模型依赖符合本阶段边界；代码/回报哈希吻合，旧YOLO三文件指纹仍与A03一致。未开展真实图像、相机/ROS或精度验收。
- **P2，须修复**：[lookup_value的端点分支](<src/photo_capture/scripts/dial_math.py#L141-L146>)不仅吸附量程外舍入误差，也把量程内邻近点直接改为0或100。合法表未规定最小节点间隔，因此会吞掉合法刻度，违反“所有节点返回对应值”的契约。现有测试只使用一个宽间隔合成表，并把端点内侧强制吸附当作正确预期，未覆盖这个冲突。它是数学边界缺陷，不是已测得的真实湿度误差。

**独立复现**（三表均通过validate_scale，`table_val=list(range(0,101,10))`）：

| 合法合成角度表 | 查询 | R1实际 | 应为 |
|---|---|---|---|
| `[0,5e-10,30,60,90,120,150,180,210,240,270]` | 第1号节点 `5e-10` | 0 | 10 |
| `[0,30,60,90,120,150,180,210,240,270,270+5e-10]` | 第9号节点 `270` | 100 | 90 |
| `[i*1e-11 for i in range(11)]` | 最后节点 | 0 | 100 |

复现脚本正常结束exit 0，打印了上述不一致；不能把这个exit 0解释为断言通过。

**D08 / G1a-R2定向修复（唯一执行项）**：

1. 只改G1a两个文件并追加简短回报。`lookup_value`在闭量程内保留节点值和正常插值；吸附只用于区间外距对应端点≤1e-9°的输入。仍返回None表示真正越界；非法表继续ValueError。
2. 不新增最小节点间隔，不拒绝上述原本合法的表，不引入实际测量容差。澄清第5.4节端点约定，其他接口不变。
3. 更新旧测试中“内侧也必须吸附”的预期；增加上述三类表的节点/区间内插值回归，同时保留外侧±1e-9°以内吸附、超阈值拒绝、不自动wrap、正常非线性表及非法输入回归。期望值须独立计算；可用二进制可精确表达的小间隔避免测试本身的消减误差，但不放宽断言掩盖错节点。
4. 运行原G1a验收命令，报告实际总数、退出码和两个文件新哈希，状态改为“待审核（G1a-R2）”后停下。不重做G0调查，不扩展图像/模型/GUI，也无需用户提供新素材即可修完。

**放行结论**：G1a-R1需修改，G1b仍待发布。本次只编辑交接文档，保留实现补丁给DS修复；没有代改算法，也没有实机许可。G1b所需清晰原图仍未取得，但不妨碍本轮修复。

### A05：G1a-R2 审核通过，转入等待G1b原图

- **受审版本**：HEAD `99104cc3ed6e8b45bdde264b6388420540f85258` + 两个未提交文件。[dial_math.py](<src/photo_capture/scripts/dial_math.py>) SHA-256 `05aabd4ab4998041387760270c2966d7776a61c695fcde6d9baa1b2f61cef6d0`；[test_dial_math.py](<src/photo_capture/tests/test_dial_math.py>) `ab206cfdbbf50b693ffbc270822342b0db4756ee21c1fb28ba8dc5c6eb6af081`；审核前本文SHA-256 `bb7a267a3ef5b529a877dad241aff237cb62667e84f85cbee627183217aef63d`。
- **结论：G1a通过。** 端点分支只处理外侧舍入误差，区间内保留节点及插值；未新增最小节点间隔，未改其他接口或放宽节点断言。A04唯一缺陷关闭，不再要求G1a-R3。
- **独立验证**：按规定命令复跑27项unittest，0.048秒、exit 0；另外用独立内联脚本对A04三张合法表的全部33个节点逐一assert，全部通过，三例结果分别为10、90、100；外侧容差、不自动wrap、非法输入等由完整回归保留。文件指纹吻合，旧YOLO三文件仍同A03。本轮没有相机/ROS/真实图像测试。
- **边界**：通过的是数学内核及结构校验，不是湿度识别系统。仍无正式标定图/角度表，不宣称真实精度、延迟或低分辨率可用；没有图像识别或实机许可。
- **下一步**：G1a停止开发；G1b先等清晰正面整表原图及来源，尤其要求下方湿度小环刻度可辨。优先附件900 px原始样张；等效新原图可用，不要求用户计算SHA-256。不要上传截图/插值放大图替代原图。审核方收到后核对表型、双轴、湿环、蓝段与11个主刻度的可标性，再冻结G1b实现和标定方案。现有119 px整表图暂不作精度真值。
- **提交范围建议**：本阶段如获用户上传授权，可只提交上述两个新增文件及本文；不包含原有导航改动或其他未跟踪素材。当前未自动commit/push。本次审核仅编辑文档，没有代改实现和测试。

### A06：接纳用户提供的跨相机原图，发布G1b0

- **用户决定**：允许上传已通过的数学内核、测试和交接文档三个文件；同时提供照片并指定作为清晰参考。此次上传不包含参考原图、未来G1b0代码或其他本地改动。
- **参考数据**：用户附件名`e34c4cc30ba869d2f2b863825b3e97bb.jpg`；只读文件：[参考原图](</home/mowen/.dsh/attachments/v1/objects/be/be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30>)。已独立核对JPEG、1279×1706、205,467 B，SHA-256 `be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30`。同环境的DS可按该路径读取；另一环境需用户传原文件，不用截图替代。
- **目视接纳范围**：deli得力No.0011型号文字、上下独立轴心、下方湿度0/10/…/100主刻度及蓝绿红小环可辨，具备离线标注候选的条件。玻璃仍有反光，不能仅凭清楚就认定几何无偏。此处没有自动测量刻度坐标，也没有直接签发正式角度表。
- **D09，跨相机原则**：可以用手机/其他相机拍摄标定参考；提取的是同表型的归一化刻度几何，不是把手机像素坐标搬给小车。标定与运行时必须采用相同轴心/圆化/角度定义；两台相机的透视、镜头畸变、白平衡、分辨率差异须在小车真实图像上验证，颜色阈值不保证可直接迁移。手机照片不能替代小车相机内参标定或现场精度验收，不保证不同实物完全通用。
- **特殊样本**：当前湿度短针指向顶部无刻度区，适合作为量程外拒绝案例，不是量程内湿度真值；不将其夹紧成0或100。标定读取静态刻度，不依赖当时指针恰好在量程内。
- **派工**：仅发布第5.5节G1b0；DS开始前记录实际HEAD，读取源文件并核对哈希后再标注。G1b1及G2–G4待发布；不得接车。此次依然由DS实现，本审核方只登记材料、冻结方法与验收标准。

### A07：G1b0-R1 审核——几何可复算，工具需修复

- **受审版本**：HEAD `6187dd03c2b835258314d0d35e630b6c5f3b9a50` + 未提交的工具/测试/标注；三者SHA-256分别为`114dae7f8bf7de63d99f0fb37181321ce3bd91474b75bab0ee0fc2f44055b9c1`、`5828096574c3282c7bd91ca315d1bcc655097e6159a7090b3ccb11ad26f313df`、`90639daf3ec40ceda84a85875c5ceee8569989e7f39f6c12fc462962ad05b331`。审核前本文SHA-256 `c4d6874cd584b397320242bbd4f7643f06b7468eaf3941579d49b64ab9e096eb`。候选/叠图指纹与R1回报一致，未覆盖旧产物。
- **通过部分**：已读取完整工具、测试、标注并查看叠图；温湿度目标未再混用。只读加载源图、标注并重算11点表，与候选完全一致；环残差RMS复算0.81633 px。SPD圆化、轴心偏移记录、候选标记和本轮范围基本符合任务。独立复跑16项标定测试（0.487秒）及27项数学测试（0.048秒），均exit 0。该结果不构成正式标定/实拍精度通过。

**必须修复的发现（已在临时合成数据上复现，不动原图/真实复核产物）**：

1. **P1，输入可能被误删**：[run](<src/photo_capture/scripts/dial_calibration.py#L541-L550>)在读输入前直接删除输出目录中两个固定文件。将合法标注命名为`candidate_calib.json`并位于同一输出目录，运行后标注被删除，再报无法读取。其他路径别名也未防护；违反输入只读原则。即使没有别名，坏哈希重跑也会先丢掉上次复核成果。
2. **P1，缺少叠图也误报成功**：[_draw_overlay](<src/photo_capture/scripts/dial_calibration.py#L531-L534>)不检查`cv2.imwrite`返回值。故障注入令其返回False，run仍返回`status=candidate`并写候选JSON，但叠图不存在；现有CLI会据此返回0。JSON写入失败时也可能只剩半套文件。
3. **P2，无方向的点被当成有效角度**：[angle_from_axis](<src/photo_capture/scripts/dial_calibration.py#L312-L317>)未拒绝零向量，`atan2(0,0)`被接受为0°。合成表的60刻度点设为轴心`[100,100]`，标注校验和建表仍成功，伪造出360°刻度。刻度点、蓝段边界点相对轴心的零向量都必须拒绝，而不是靠最终表顺序偶然兜底。

**D10 / G1b0-R2定向任务**：

- 最小安全策略：输出目录必须不存在或为空；若已有任何内容，明确拒绝且不改动它，要求本轮改用新空目录（例如`.gauge_calibration_review/r2`）。先解析/核查输入与输出目标，不能删除输入、旧叠图、旧候选，也不能靠清空目录处理冲突；同路径/别名场景安全拒绝。不引入`--force`覆盖模式。
- 检查`cv2.imwrite`布尔结果，处理编码/写文件异常；只有候选JSON和叠图都成功才报告成功。失败返回非零且不得留下可被误当本次成功的半套最终产物；可先在本次新目录内用临时文件完成写入再发布，失败只清理本次创建的临时/半成品，绝不碰已有文件。
- 拒绝刻度/蓝段相对轴心的零向量（使用明确数值阈值，不另加未经验证的图像尺度门槛）；在角度辅助函数和正常标注→建表通路补回归，错误用CalibrationError，不产候选。
- 增加故障回归：非空输出不变、标注/图像与输出冲突时输入哈希不变、imwrite返回False/抛异常、JSON写入失败、零向量刻度和蓝段；重写原先要求“删除旧候选”的测试，不能继续将破坏性清理当正确行为。
- 保留原16项有效回归和数学27项，在新空目录生成R2实图产物，报告命令/退出码/全部新哈希与本地链接。仅修改本轮工具、测试和本文；源图/原始标注/数学内核/YOLO不变，不扩展完整检测器，不commit/push。

**非阻塞诊断备注**：现有±1px灵敏度仅采四个方向，且量程外扰动返回None时未统计为失败；0.294 %RH/px不是总体精度上界。R2回报应列明量程外扰动次数及“仅对可插值扰动”的统计口径，不必扩写复杂误差模型。边缘/相机导致附件常量差异目前也只是解释假设，不认定已证实。

**结论**：G1b0-R1需修改，几何表不要求无证据重标，G1b1仍未发布。此次审核只更新本文；无正式标定签发、无实机许可。

### A08：G1b0-R2审核通过，发布指定ROI单帧验证

- **受审版本**：HEAD `36b550991401f285eccaf4afb97e0b8853f12052` + 未提交R2；工具SHA-256 `1dacb5c9bdec9bc642c291badf18c645f91cd7fdaadd5a875c67440ec710663e`，测试`7f1e59089a15786ae3e3a887b623d8124bf1c9283509b7a2bb9edb7b28f76408`；审核前本文SHA-256 `8868fb10e30796e21a5b35d89987de3cb5d0ed5552ff8f8d7de2120447a77b88`。
- **结论：G1b0通过。** A07三项缺陷均关闭：非空/冲突输出安全拒绝，输入与旧产物保留；检查写图结果、临时文件发布并在失败时回滚；刻度/蓝段零向量以1e-6 px数值阈值拒绝。原标注、11点几何及R1产物未变，无需重标。
- **独立验证**：24项标定回归0.584秒、27项数学回归0.047秒，均exit 0；另外注入“第二个最终文件os.replace失败”，确认首个已发布文件与临时文件回滚、无半套结果，输入仍在。R1/R2候选的table/ring/axis_offset/blue/ticks逐项完全一致。没有为了验证覆盖或删除真实复核文件。
- **证据绑定**：[R2候选](<.gauge_calibration_review/r2-final/candidate_calib.json>) SHA-256 `d7dccc3e6d80f24199405dce07a516fd119d4502724efa413983682d2e5df198`；[R2叠图](<.gauge_calibration_review/r2-final/calibration_overlay.png>) `468472fbc2c7047b7e3b163c2d7204ee2004ae5eee08fdba049c854b01ac7b89`。标注仍为`90639daf…`，源图仍为`be71a511…`。
- **诊断口径已补齐**：刻度扰动40次可插值、4次越界；轴心扰动4次可插值、0次越界。0.293887 %RH/px只作四方向局部诊断，不认定为总体精度上界。两文件发布有异常回滚，不将其夸大为进程崩溃/并发下的整组事务保证；本轮只支持单写者离线工具。
- **D11**：接受该候选作为显式指定、哈希绑定的**离线开发参考**，不是生产正式标定，也不作为默认配置。发布第5.6节G1b1：先指定湿度ROI验证像素到角度/候选值的链路，暂不解决自动全图找表，最终可信数值留到多帧与实拍验收。不得在算法里写死参考图轴心或绕过拒识。
- **下一步**：DS重读首页，只实现G1b1并提交R1后停下。本审核方仍只改本文，不代写识别代码；G2–G4未发布，没有实机许可。若用户授权上传，本轮可提交工具、测试、原始标注和本文4个文件；源图及整个本地复核目录均不入库，其他改动不混入。

### A09：本会话接管G1b1-R1，显式派Flash限定修复

- **用户授权与派工方式**：用户要求B1后由本会话直接派DeepSeek V4.1 Flash；实测内置团队能传消息，但spawn_teammate不支持模型选项，创建/执行回执显示gpt-6.1-sol。已中止该队友，仅做过一句通信测试，未授权其代码工作。用户随后明确选择“改用Workflow”，本次固定`provider=deepseek-official, model=deepseek-flash`，不修改插件/全局模型配置。命名团队的未执行任务已撤销，避免两个实现者重叠。
- **受审版本**：HEAD `33740bda1e346dc4c522f83ad6ee364184c8df5a`；R1新增代码`88735b76822584139200882f600cc89f7f5f3aec8b687c00b4d49a62e4ad59ba`、测试`e8e7d103d898ef7913e50d9e68507ede482800820745a8cbd44e4e791eb75637`，均未提交。Lead独立复跑17项单帧、24项标定、27项数学测试，均exit 0；测试通过不消除参考原图未检出的事实。
- **P1根因**：[migrate_axis](<src/photo_capture/scripts/dial_reading.py>)把运行时环心蓝段中点减去了参考轴心中点，未按第5.6节使用参考环心中点。候选的两参考中点分别195.78467879°/196.292032°，自带约0.507°差，导致原图被0.5°门槛错误拒绝。Lead在内存中统一基准后，roll1/roll2为0.01532121°/0.01347276°，差0.00184845°，可继续检测272.7°指针、覆盖率1.0；未改文件或放宽阈值。
- **R2已派范围**：仅写单帧模块/测试；校正初始roll基准、增加非零轴心偏移的手算与滚转回归；修正调试线从归一化方向到原图的反变换；完成参考图空白区out_of_range演示。已通过的标定/数学/旧YOLO及所有输入数据保持原样，本文由Lead独占。
- **评测缺口**：R1只画干净色环，未满足真实底图去针再绘制；且300/450/900实际来自画布尺寸，并非已测整表直径。Flash须交真实底图复制/去针掩膜/已知角度合成的完整条件清单，明确湿环尺寸与画布尺寸，不冒称实拍真值。所有拒识/错峰均报告，不能只统计成功例；低分辨率原帧只作挑战，不报真实MAE。
- **执行记录**：Workflow `gauge-flash-g1b1-review-fix`，job `workflow-481b882ce71a4bcd`；完成后由Lead收取结果、独立跑测试并读叠图，再追加最终结论。无自动上传、无G2发布、无实机许可。

### A10：Flash-R3独立复审——指定ROI离线开发验证限定通过

- **受审版本**：HEAD `33740bda1e346dc4c522f83ad6ee364184c8df5a` + 上述两份未提交新文件，SHA按Flash-R3回报绑定。Lead独立确认旧YOLO、标定工具/标注、数学内核指纹不变，本轮只由Flash写单帧代码/测试、Lead写本文；没有用错误模型继续实现。
- **独立复跑**：28项单帧、24项标定、27项数学回归，共79项全部通过（2.253/0.582/0.048秒，exit 0）。Lead在全新目录单独重跑完整217例，不只读取Flash报告；结果同为187接受/30拒绝/错误候选0，接受例最大误差1.91460859 %RH、最大roll误差0.33652724°。原图仍out_of_range，2张低分辨率帧仍no_dial。
- **生成规则核验**：对R2/R3的擦针、仿射场景、绘针、完整评测与低分辨率挑战函数逐一比较AST哈希，完全相同；候选/源图不变，case_id 217个条件保持一致。改善来自采样顺序修复，不是换数据、减案例或只筛成功帧。Lead已查看原图叠线与原错误条件修复图，湿度小轴/短针/蓝段位置对应合理。
- **复核产物**：[独立总报告](<.gauge_reading_review/lead-g1b1-r3-verify/eval_report.json>)、[独立逐案例JSON](<.gauge_reading_review/lead-g1b1-r3-verify/synthetic/synthetic_eval.json>)、[独立原图叠线](<.gauge_reading_review/lead-g1b1-r3-verify/reference/debug_overlay.png>)。精确命令：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_reading.py' -q
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_calibration.py' -q
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/photo_capture/tests -p 'test_dial_math.py' -q
PYTHONDONTWRITEBYTECODE=1 python3 src/photo_capture/tests/test_dial_reading.py --real-eval --output-dir .gauge_reading_review/lead-g1b1-r3-verify
# 全部exit 0；复核目录已存在，重跑必须另用新空目录，不能覆盖。
```

- **验收结论**：本轮两个明确代码缺陷已关闭，G1b1的显式ROI/单帧诊断及给定离线测试限定通过；不是全量程稳健、自动找表或真实湿度精度通过。所有输出的顶层value仍null，不把单帧候选发给巡航。
- **D12，下一步按信息瓶颈而非代码量排序**：0/100端点仍会拒识，旧小车约119px整表图仍no_dial，增加多帧不能凭空补回看不见的环/针。先补较近距离/较高真实分辨率的小车相机原帧，记录整表及湿度环像素、光照/倾斜、可核读参考，再决定G2范围。此处是材料请求，不是AI开相机或实机运行许可；未发布G2–G4，不能绕过审核跑车。端点拒识不得靠夹紧修掉。
- **交付与代理状态**：两次Flash Workflow及Lead复核命令均已结束、输出已收集；命名团队队友保持inactive，不存在另一个实现者正在写文件。本轮可review补丁仅涉及单帧模块、测试、本文3文件，需用户明确同意后才提交上传；所有原图/合成图/复核目录留本地，导航等无关改动不混入。

### A11：固定距离新实拍验证、型号勘误与隔离实验

- **输入与复核**：[19:40十帧报告](<.gauge_reading_review/new-frames-20261115-1940/summary.json>)来自1280×720原始JPEG，整表约147–159 px、湿度小环外边框约57–65 px；现有默认路径10/10 no_dial，Lead独立逐帧复跑一致，顶层及候选值均null。停在颜色环定位，未进入指针/查表，不用此结果计算RH误差。
- **手机对照**：[19:40近照](</home/mowen/.dsh/attachments/v1/objects/fe/fe554e70eab8087ff60d8d449539db38af90c7c79debed5b97d2603b42504eb4>)湿环约275×296 px，旧路径诊断72.04；[19:56模拟视角](</home/mowen/.dsh/attachments/v1/objects/d3/d3ab7bc1c93be2d47cb8ad996ad9b90e1d6547bc0fdd5699060810e9cf5facb6>)湿环约88×98 px，诊断76.39。分别下采样至湿环长轴60 px仍能检出，诊断71.56/77.25；这些是控制实验和不同时刻的候选，不是精确真值或同刻误差。
- **D13，型号勘误**：用户明确两张参考与实物为同一块No.9011，旧No.0011是本方识读/元数据误记。以用户确认作为实物身份依据，撤回“换了型号所以标定不匹配”的判断。保留历史记录但不再据其另建表型；现有候选标签/文件名暂保留历史兼容，几何与哈希不因本轮改动。将来清理标签需连同新候选哈希及回归一起审查，不在实验中悄悄替换。
- **D14，固定硬件/视距**：用户确认靠近有压线风险、此前相机设置已研究。仓库[硬件表](<README.md#L13-L16>)记UVC `/dev/video0`拍照和Orbbec Dabai DCW2可选深度设备；[画质说明](<src/photo_capture/README.md#L519-L536>)记1280×720 MJPG、锐度3等既有取舍。此次不推断可选Orbbec必是当前UVC型号，不打开设备、不改设置/导航，也不再要求用户靠近。
- **瓶颈更正**：相机ROI在S≥90/V≥60仅41–72色像素；降低S到60仍碎成小块，降到30会把背景混入，示例椭圆残差>6 px。Lead仅内存扫90/75/60/45/30/20，默认文件未改，10帧都没因此检出。结合手机同尺度成功，撤回“仅像素不足”的结论；但手机60 px可检出也不证明小车能达到±2。
- **E1任务**：显式Workflow(deepseek-official/deepseek-flash)隔离新增低饱和实验模块及测试，边缘几何为主、软角向颜色签名估roll，不要求彩色连通域闭合，不改已通过读数器及其阈值。输入固定10帧，2/8仅内部切分；负例门控看过全burst，不宣称盲留出。原型实现约2355行，后续只定向修复，不继续膨胀框架。
- **Flash-E1-R1结果**：环/roll/暗度针10/10可恢复，8评估诊断均值74.984、std0.322；22负例全部拒绝。全部真实相机帧被`axis_angle_uncertain`拒绝，顶层value=null、trusted=false。产物：[报告](<.gauge_reading_review/fixed-view-low-saturation/report.md>)、[代表叠图](<.gauge_reading_review/fixed-view-low-saturation/overlays/overlay_0003.png>)。22项新测试+79项既有回归由Lead独立复跑，全部exit 0。
- **R1复核发现，不能直接把“轴心不足”定性为物理极限**：`_dark_hub_candidates`注释声称用DT峰中心，代码却仍返回连通域质心。Lead在第3帧复算质心[31.3492,34.2540]、DT峰平台中心[34.0,35.125]，差2.7902 px。另将轴心与环心的正常几何偏移直接当不确定度；两中心本来可不同，这不等于估计方差。稳定诊断不是正确性证明，过保守的错误拒识也不是可接受的物理结论。
- **E1-R2定向派工**：仅改两份隔离实验文件，真正采用DT峰证据并以平台/阈值稳定性形成轴心不确定度，保留几何冲突和信息不足拒识；不提高2°实验稳定预算、不围绕75或手机读数调参。增加轴帽连细针、合法非零偏移、等粗/双帽歧义、跨0°区间回归，重跑同数据/负例/手机控制和79项保护。job `workflow-1718845d51714499`，由Lead收取后独立复审。没有生产切换、G2发布或实机许可。

### A12：E1-R2独立复核与针身方向后续实验

- **R2受审版本**：隔离模块SHA `4c5eb10f67fd52cf93f445f1a9afe55b7752af50e8a72ae52136c3e4fb77b22e`、实验测试SHA `d52cb948e7933340e785fc93238560893d36c0b66d0d092f4a949e499212388e`；已修DT真实取点、环心偏移与轴不确定度混淆，跨0°区间与无hub fallback有独立回归。不是放宽2°实验预算。
- **独立验证**：Lead复跑32实验+28单帧+24标定+27数学，共111测试全部exit 0；全新目录独立重跑原10帧/控制集，8评估仍0接受、8诊断。产物：[独立报告](<.gauge_reading_review/lead-fixed-view-r2-verify/report.md>)、[独立JSON](<.gauge_reading_review/lead-fixed-view-r2-verify/summary.json>)；Flash原报告保留于[R2报告](<.gauge_reading_review/fixed-view-low-saturation-r2/report.md>)。
- **失败含义**：不是异常退出或测试失败。10帧均找回环/roll/针，但轴帽平台及阈值候选给出的针角敏感跨度5.6–9.1°，超过实验2°稳定门，全部low_conf；8帧诊断均值76.254、std0.493，不作为真值或优化目标。手机原图独立轴帽证据跨度0.6°可通过，而手机60px无独立轴证据仍拒识，说明旧等同门已移除。
- **边界说明**：平台范围/阈值扰动是该估计器的敏感性包络，不是经校准的统计置信区间，更不是图像物理不可读的证明。22个控制里含低饱和/模糊有表压力场景，不能把其全部拒识直接宣称22个严格真负例成功；后续须按有效正例、错误/不匹配场景与真正非表负例拆分。
- **D15，改特征而不是放宽门槛**：用户明确要求继续。针身直线方向经SPD变换即可取角，数学上无需精确知道轴点沿线位置；轴帽可只用于宽松定位和针尖/尾部消歧。但Lead一次粗略二值HUBER直线内存诊断仅6–15支持像素、跨帧std约3.51°，因此不能直接用简单质心/PCA替换就宣称成功。
- **E1-R3已派Flash**：在同两份隔离文件上做有界针身条带/成对边缘方向估计，保留旧射线结果为对照；方向需独立证据和主次峰/稳定预算，歧义仍拒识。必要时只研究同burst共享几何的联合图像证据，不做数值投票、不当独立精度。纠正合成夹具cos符号/整轴长度问题后补多方向与合法偏移回归，将正控/压力控/负例分开。job `workflow-a433f0c7c7084688`，已明确一次原型后停下交Lead，不无限调参。
- 原相机/视距/导航与所有通过代码、候选哈希保持不变；最终value始终null，未解锁G2–G4或实机，也未提交上传本轮隔离实验。

### A13：E1-R3中心线复核——模型与区间定义须纠正，已派R4

- **R3交付版本**：实验模块SHA `ce612006762a1a6426e31f9a651929d663097311b85e3f15660a7625b4a1b7c2`；测试SHA `6d6a7d1c45a2d018ebf9aabe0e236b81579c4a43e928551b345c07df6ef61129`。Flash自测47实验+79旧回归通过，10帧仍拒识；[R3报告](<.gauge_reading_review/fixed-view-line-r3/report.md>)与[R3逐帧数据](<.gauge_reading_review/fixed-view-line-r3/summary.json>)保留，未据此验收精度。
- **有价值的恢复**：针身双边中点拟合对hub/环扰动跨度0.24–1.14°（均值约0.67°），比R2轴点射线敏感度低；但实际报告方向仍取平坦条带峰，8帧std2.65°，并把“边缘必须平行”作为2°硬门。联合角向剖面85%峰宽15°，没有证明额外改善，不能用留一结果1.5°替代总不确定性。
- **D16，不能把锥度当方向噪声**：直锥形针的两边可以不平行，而对应位置中点仍严格共线；把开口角本身算作中心线不确定度缺乏物理依据。现有实现又计算了中点中心线，却只把它作为条带峰的偏差项，不能据这种模型冲突宣布图像物理不可读。后续明确选择针身中点中心线模型，条带峰与开口角保留诊断，真正弯曲/缺证据仍须拒识。
- **数学问题**：bootstrap角跨度返回`2*atan(max_slope-min_slope)`，并非实际角范围，存在额外约2倍放大；应比较各方向角的环形跨度。其`fitted[index]+residual[index]`等于原`t[index]`，是块成对重采样，不是重新分配残差；只能报告有限样本敏感性包络，不叫校准置信区间。另需补扰动失败不得忽略为零变化的安全门。
- **E1-R4定向派工**：只修现有实验里的主中心线、相应的区间/缺证据门和独立锥形/弯针/双针回归；保留2°方向预算，不去做新增PSF前向大框架，不围绕期望RH调结果。复用原评测数据/18真负例与4压力正控的分类，报告任何新失败，不偷换接受率。job `workflow-1ff5a9272fad49a2`。
- 依旧无可信读数、无独立精度、无正式代码/硬件/导航变更；所有value=null、trusted=false，Lead复审后再记录本轮可行性结论。

### A14：E1-R4独立复核——中心线有进展，角度坐标门须统一

- **受审版本**：实验模块SHA `1e91629853814fe39698518ee1eacab612ea2e64530ec5c46a29b308918fe995`、测试SHA `c22b357aa0a4790f53edc6951d9b723a7e0a606a4f13b05797abe8572fe9ffba`。中心线替代条带峰、直锥不要求双边平行、bootstrap实际角范围、Huber零残差与扰动缺证据门已修。
- **Lead独立重跑**：60项实验+79项原回归，共139项全部exit 0；全新目录重跑10帧及控制集，复现3/8评估帧通过当时内部稳定门，其余拒识。产物：[独立报告](<.gauge_reading_review/lead-fixed-view-r4-verify/report.md>)、[Flash报告](<.gauge_reading_review/fixed-view-centerline-r4/report.md>)、[第4帧叠图](<.gauge_reading_review/fixed-view-centerline-r4/overlays/overlay_0004.png>)。归一化线方向8帧圆std0.65°，诊断73.685±0.406只是内部一致性，不是准确率；18真负例无误接受、4压力正控仍失败，手机控制未全部解决。
- **尚不能按3/8验收**：主方向已经SPD映射到归一化平面，而split/bootstrap/fit/hub/ring跨度仍是像素平面角，直接与2°比较不一致。独立手算反例A=diag(.5,1)：像素0–1.9°约映射为0–3.8°，原像素门可假通过。此问题会影响边界接受数，必须纠正，不能悄悄把2°重新解释成像素门。
- **D17，小改而非再建框架**：保留像素跨度，使用正定A的角映射导数上界κ₂(A)得到保守归一化跨度上界min(360,κ×pixel_span)，门仍为归一化2°。不是精确区间/统计置信区间；无效矩阵或缺证据fail closed。保守修正后通过数可能减少，不能为保3帧而调门。
- **E1-R5已定向派Flash**：仅同两份隔离实验文件，统一报告/CSV/叠图坐标单位，补独立SPD/各向同性/跨0/无效矩阵回归，不改针定位/阈值/已通过默认路径。job `workflow-b703d30faee94ed3`，完成后独立复核并在本轮停到可review补丁，不再扩大模型或硬件范围。

### A15：E1-R5独立复核——有部分稳定候选，自动读取仍未验收

- **交付版本**：[隔离实验模块](<src/photo_capture/scripts/dial_low_saturation_experiment.py>) SHA `7d773d04282dca8b4b8b5502a9e31b477b69623d52cea0d40d00fb1e45889435`；[实验测试](<src/photo_capture/tests/test_dial_low_saturation_experiment.py>) SHA `a4c2338d96c5c7069bda444face1d315b712abb22fd6c0494310cc717502777b`。实验显式入口、不被原读数器/GUI引用；主方向不因单位修复而变化，像素跨度与归一化保守上界分开报告。
- **独立验证**：Lead复跑65实验+28单帧+24标定+27数学，共144项全部exit 0（29.590/2.248/0.583/0.047秒）。在全新目录独立重跑原始10帧/控制集，复现8 eval仅1稳定、7拒识，dev 0/2；所有顶层value=null、trusted=false。产物：[独立报告](<.gauge_reading_review/lead-fixed-view-r5-verify/report.md>)、[独立JSON](<.gauge_reading_review/lead-fixed-view-r5-verify/summary.json>)；[Flash R5报告](<.gauge_reading_review/fixed-view-coordinate-r5/report.md>)保留。
- **最终数值口径**：第6帧像素跨度1.434°、κ=1.06793、归一化上界1.531°，满足2°内部稳定门，诊断候选约73.60 %RH；不是经独立真值验证的实测结果。第4/9帧原像素门假通过，映射上界2.092/2.065°后正确拒绝。R4的3/8不能再作最终接受数。
- **失败与进展各是什么**：默认算法HSV强色门从未检出10帧；隔离边缘路线已找回环/roll/针，轴点依赖和错误的平行细棒模型已纠正。当前仍有7/8评估帧的近/远段中心线或块重采样超过预算，整体自动读取尚未通过；这是当前估计器/证据限制，不是已证明该相机物理不可读。65测试通过说明实现与拒识契约受保护，不代表精度通过。
- **控制集**：18真负例全部拒绝；4有效低饱和/噪声压力正控仍未通过，不能算真负例成绩。手机原图支持不足拒识、手机60px缺粗hub拒识，跨尺度鲁棒性尚未解决。当前2/8只是同burst内部一致性，R1已看全数据；没有人工精确真值，没有MAE或±2 RH验收。几何上界是保守敏感性包络，不是统计置信区间。
- **D18，停到可review检查点**：当前方向方案有局部可行证据，但不能部署/切默认路径；不再为提高本批通过率继续放宽门或堆模型。若继续，下一项应冻结“按静态盘面配准后的多帧中心线几何联合估计”，以每帧位置偏差作干扰项并检查共同方向/分段一致性；不是平均旧条带峰或RH读数。仍需独立状态/姿态与人工参考才能谈精度，不先许诺成功。
- **边界与状态**：本轮所有Flash任务及Lead复核已收取结束；用户无需因本轮再近拍/调设备。仅新增实验模块/测试、更新本文3文件；原算法/标定/数学、原始照片及导航均未改，原图/复核图留本地。本轮未commit/push，是否上传由用户决定；G2–G4和算法实机运行仍未放行。

## 9. 实机记录区（用户提供事实后追加）

尚未放行算法实机运行，尚未测试。

后续每次记录：许可编号及版本、设备/表型、场景与真实直径、光照/倾斜、采集序列链接、人工参考、输出与失败、实际延迟、结论和下一步。不能只写“上车正常”。

### 用户材料与固定约束（A11取代先前近距采集建议）

- 已收到“仪表验证”10帧原图、19:40手机近照与19:56模拟小车视角手机图。用户已完成本轮材料提供，目前不要求再次采集或手工标注。
- 用户确认：再靠近会有压线风险，相机画质/变焦已研究、现有设置基本最优。因此保持当前距离、相机设置、导航参数不变，不再以靠近/变焦/重复调相机作为默认下一步。
- No.9011是同一块实际表，过去No.0011是误记；不再要求用户为“换型号”重拍或重标。
- 后续若确需材料，只由Lead提出一个明确缺件与目的；不擅自启动相机或巡航。当前优先使用已有原图做隔离低饱和碎环实验。
