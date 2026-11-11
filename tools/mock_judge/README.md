# 智慧社区模拟裁判台

**协议联调工具，非官方裁判，不计算分数，也不验证识别真伪。** 接收附件约定的八类任务、文字与单/多图，提供中文网页、锁队、手动计时、历史记录和导出。不运行 ROS、不启相机、不控制机器人，不修改现有导航或视觉代码。

## 1. 启动

从工作区根目录执行（Python 3.8+）：

```bash
cd ~/bit_ws
PYTHONDONTWRITEBYTECODE=1 python3 -m tools.mock_judge
```

浏览器打开 <http://127.0.0.1:8080>。默认数据在 `~/.local/share/bit_ws/mock_judge/`，启动日志会给出路径。网页管理区输入该目录 `admin_token` 文件里的令牌；令牌只在网页内存保留，刷新即清除，不要发给其他队伍或放进 Git。

当前开发机已有依赖；其他机器建议隔离安装，不升级 ROS 的全局环境：

```bash
python3 -m venv ~/.venvs/mock-judge
~/.venvs/mock-judge/bin/pip install -r tools/mock_judge/requirements.txt
PYTHONDONTWRITEBYTECODE=1 ~/.venvs/mock-judge/bin/python -m tools.mock_judge
```

端口占用不会自动换端口。可使用 `--port 8081`，并同时修改客户端地址。停止时在启动终端按 Ctrl+C。

### 小车从局域网接入

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m tools.mock_judge --host 0.0.0.0 --port 8080
```

浏览器和小车使用 `http://裁判电脑的局域网IP:8080`，**不是** `0.0.0.0` 或小车自己的 `127.0.0.1`。路由器须关闭 AP 隔离；必要时由管理员明确配置防火墙。本工具不自动改网络设置。

普通上报和读取接口无认证，以兼容附件，局域网其他设备可能查看记录或发送模拟结果。管理令牌也经明文 HTTP 传输，故仅在可信隔离局域网使用，**不要暴露公网**。Flask 内置服务器用于联调，不是生产部署。此页面独立于 DeepSeek Harness GUI。

## 2. 无硬件演示

另开终端，保持裁判服务运行：

```bash
# 附件原测试：三次车牌单图 + 人群三图，共4条记录6张图片
PYTHONDONTWRITEBYTECODE=1 python3 tools/mock_judge/examples/attachment_demo.py \
  --url http://127.0.0.1:8080 --team 示例战队

# 八任务示例，含纯文字/单图/多图，共8条记录9张图片
PYTHONDONTWRITEBYTECODE=1 python3 tools/mock_judge/examples/demo.py \
  --url http://127.0.0.1:8080 --team 模拟联调队
```

示例图片是标准库生成的色块，不是识别成果。附件原有 `上传参考代码.py` / `识别上传范例.py` 中 `JUDGE_HOST`、`TEAM` 改为这里的地址和队名即可上传；本工具不把这些示例服务接进车辆流程。

## 3. 网页操作与模拟约定

1. hello 自检或上报后，队伍会出现在列表；可直接上报，不强制先 hello。
2. 输入管理令牌，选择/输入队伍并锁定。锁定后其他队伍的 hello/report 返回403；解锁后可接收多队伍，原记录不删除。
3. 准备阶段 `ready` 允许联调上传。**hello 和 report 均不启动计时**。
4. 手动「开始计时」进入 `running`，按服务器时间连续计时；没有暂停或自动超时。重启的时间也计入。
5. 「结束场次」仅可在 running 时操作，进入 `finished` 并冻结计时；新上报返回409，hello仍可测试连通。
6. 「新建场次」允许 ready/finished，running 必须先结束。旧场次保留，锁定和计时不沿用。联调和正式模拟建议分别新建场次，避免混在一起。
7. 选择场次、队伍、任务和精确点位查看。最新视图按“场次/队伍/任务/点位”分组，历史保留每次上传。重复请求不会去重，网络超时后重试可能出现两条记录。
8. 缩略图可打开大图，支持前后翻页。管理操作始终作用于**当前场次**，导出作用于所选场次；旧场次不能重新开始或追加。

以上是模拟约定，附件没有规定完整比赛计时/官方界面/评分细则，不能据此声称复刻官方判分。

## 4. HTTP 接口

所有文本 UTF-8；时间是 UTC ISO8601，计时值为秒。错误返回 `{"ok":false,"error":"说明"}`。

### 原上传协议

- `POST /api/hello`：JSON `{"team":"队名"}`。
  - 200：`{"ok":true,"message":"通信正常","team":"队名"}`。
- `POST /api/report`：`application/x-www-form-urlencoded`（纯文字）或 `multipart/form-data`。
  - 必填：`team`、`task`、`result`；选填：`sub_id`、`extra`，缺省为空字符串。
  - 图片：旧字段 `image`、重复字段 `images` 都支持；混用也按上传顺序保留。
  - 200：`{"ok":true,"report_id":"UUID","image_count":3}`。
  - task：`crowd`人群、`sign`指示牌、`trash`垃圾桶、`fire`火灾、`plate`车牌、`temperature`温度、`meter`仪表、`ebike`电动车。红绿灯和泊车不上传。
  - 不解析/改写 result 或 extra，不要求固定播报句式。

### 只读接口

- `GET /api/state[?session_id=...]`：`ok/schema_version/server_time/session/sessions/teams/tasks`。不带参数是当前场次，带参数是所选旧场次。
- session：`id/created_at/status/locked_team/started_at/finished_at/elapsed_seconds`。
- teams：每队 `team/last_hello_at/last_report_at/report_count`，仅为选定场次的数据；tasks 是 `id/label` 列表。
- `GET /api/reports`：可选 `session_id/team/task/sub_id/limit/offset`。默认当前场次、新到旧、limit=100；limit范围1..200，offset非负。省略字段表示不筛选，显式 `sub_id=` 匹配空点位。
- 返回 `ok/reports/total/limit/offset`。每条报告含 `id/session_id/received_at/team/task/sub_id/result/extra/image_count/images`；每张图含 `id/url/original_name/media_type/width/height/position`（从0编号）。
- `GET /api/images/<id>`：实际 JPEG/PNG 内容。客户端文件名只存元数据，绝不作为路径。

### 管理接口

均需请求头 `Authorization: Bearer <admin_token>`，禁止异域 Origin；不支持跨域 CORS。

- `POST /api/admin/team-lock`：JSON `{"team":"队名"}`；解锁用 `{"team":null}`。
- `POST /api/admin/session/start`、`finish`、`new`：JSON `{}`。
- 上述操作返回 `{"ok":true,"session":{...}}`。
- `GET /api/admin/export[?session_id=...]`：ZIP，内含 `manifest.json`、UTF-8 BOM 的 `reports.csv` 和 `images/`；JSON有schema_version、场次、报告、图片archive_path、自检和管理事件。CSV对危险公式前缀加单引号，原文仍保留于JSON。

### 错误与限制

| HTTP | 含义 |
| --- | --- |
| 400 | 缺字段、空白必填、重复文本字段、超长或未知任务、非法分页 |
| 401 | 管理令牌缺失或错误 |
| 403 | 队伍锁定不符，或跨域管理 |
| 404 | 场次/图片/接口不存在 |
| 409 | 已结束场次上报或非法状态转换 |
| 413 | 请求、单图、图片数量或像素超限 |
| 415 | 非表单上报、非JPEG/PNG、损坏图片 |
| 500 | 存储失败/内部错误，检查服务器日志和磁盘 |

默认32 MiB/请求、16图/报告、8 MiB/图、2000万像素/图；JPEG/PNG按实际内容检查，不信任后缀或MIME。team/sub_id≤128字符，result/extra≤10000字符；空白必填不接受。调整参数：

```bash
python3 -m tools.mock_judge --help
# --max-request-mib 32 --max-images 16 --max-image-mib 8 --max-image-pixels 20000000
```

原客户端示例中的后台线程不检查所有HTTP错误；联调时请同时查看裁判台和日志，不把“线程启动了”当作上报成功。

## 5. 保存、备份与恢复

数据目录结构：`judge.sqlite3`、`images/`、`staging/`、`admin_token`。默认不在仓库，**不要提交个人图片、数据库和令牌**。可用 `--data-dir /绝对路径/专用目录` 隔离不同练习。

正常重启保留场次、锁队、hello、全部上报及计时。写入使用 SQLite 事务和临时图片；校验或正常I/O失败不留下半条报告。进程被强杀/突然断电时文件系统和SQLite不能构成跨介质原子事务，可能留下未关联文件；不要手动删除数据库所引用图片，必要时停止服务后备份整个数据目录再排查。

备份应先停止服务再复制**整个数据目录**；ZIP导出用于查看/对账，不提供导入恢复接口。恢复时指向备份目录。时钟依赖系统时间，应在开始前校时，赛中调时可能影响计时；计时值不会小于0。

端口/目录失败会明确报错，不静默改用其他端口/内存存储。数据库 `user_version=1`；将来升级必须显式迁移，当前遇到未知版本拒绝启动。令牌丢失时先停止服务、备份，将旧令牌文件移至安全位置，重启会生成新令牌；不要同时运行两个实例操作相同数据目录。

## 6. 测试与交接

```bash
cd ~/bit_ws
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/mock_judge/tests -v
node --check tools/mock_judge/static/app.js
node tools/mock_judge/tests/test_dashboard.js
```

Python测试使用临时目录和可控时钟，不开ROS/相机/模型；Node测试使用内存模拟DOM/fetch，验证点位切换、分页、轮询串行化及管理数据流，不启动服务器。Node只用于可选回归，不是运行服务的依赖。接口/模拟DOM测试不等于浏览器交互测试或真实车辆识别验证。

实施计划见 [mock_judge_plan.md](../../docs/mock_judge_plan.md)，阶段结果、已验证/未验证项及服务信息见 [mock_judge_progress.md](../../docs/mock_judge_progress.md)。接手agent先读进度，再运行以上测试；保留工作区用户原有修改，不自动提交/推送。
