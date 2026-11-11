# 模拟裁判系统进度与交接

更新时间：2026-11-11（Asia/Shanghai）。计划已获用户批准；用户已明确同意仅提交并推送裁判系统。

## 状态
- [x] P0：读取附件、工程调查、范围确认、保存计划与用户改动基线。
- [x] P1：协议/图片/持久化闭环通过，最终完整回归已完成。
- [x] P2：会话、锁队、计时、导出和恢复通过，最终33项Python回归无警告。
- [x] P3：网页裁判台及可复现模拟DOM回归；真实浏览器视觉/交互待人工验收。
- [x] P4：样例、说明、真实HTTP验收和最终检查完成；人工浏览器交互验收另列未验证项。

## 已完成与验证
- 附件4个文件已只读检查，协议与模拟约定见[计划](mock_judge_plan.md)。
- Python 3.8.10；依赖版本 Flask 3.0.3 / Werkzeug 3.0.6 / Pillow 10.4.0 / requests 2.32.4；8080检查时空闲。
- 用户选择协议兼容裁判台，不做人工/自动评分。
- P1新增 tools/mock_judge/{__init__,store,app,__main__}.py 和 requirements.txt。实现SQLite事务、图片校验、hello/report及读取接口；管理/导出代码随后在P2/P4验证完成。
- P1 smoke（2026-11-10）：用临时目录和Flask test_client验证中文hello、纯文字上报、带路径穿越文件名单图上传、图片读回、两条记录重启持久化，全部通过；hello/report后仍为ready且未启动计时。命令为 `PYTHONDONTWRITEBYTECODE=1 python3` 临时内联脚本，job `bash-c97380efb08c4604` 已收集，exit 0。
- 测试/示例agent完成 tests/、examples/；网页agent完成 templates/、static/及单独授权的tests/test_dashboard.js；主agent负责后端、文档、整合与真实HTTP验收。所有agent均已完成，无常驻服务。

## 执行约定
每完成可验证小项更新本文件。测试用临时目录，`PYTHONDONTWRITEBYTECODE=1`，不触碰ROS/相机。任何服务记录job/PID/精确URL/数据目录。提交和推送必须取得用户明确同意。

## P2验证与后续实现
- 测试agent执行 `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/mock_judge/tests -v`，首轮26 tests / 1.641s / OK：覆盖八类任务、中文边界、混合image/images保序、分页、锁队/鉴权、跨域拒绝、手动计时、重启恢复、18并发上报、导出及CSV防注入、第二张图片移动失败后的SQL/文件回滚。
- 已修复首轮测试未关闭图片响应的ResourceWarning。测试agent最终执行 `PYTHONDONTWRITEBYTECODE=1 python3 -W error::ResourceWarning -m unittest discover -s tools/mock_judge/tests -v`：33 tests / 1.864s / OK / exit0，无warnings。补充验证了token文件600/重启持久化/不泄露、非法状态转换、未知数据库版本拒绝、截断JPEG、数据库连接失败返回500后可恢复，以及示例mock测试。
- 新增 tools/mock_judge/README.md，记录启动/局域网/原协议与模拟约定、状态码、限制、备份恢复、安全边界和交接入口。
- examples/attachment_demo.py 已从附件原测试复制并仅增加来源/CLI配置；examples/demo.py 提供八任务8条9图演示。
- 真实HTTP（127.0.0.1:8080）执行两份示例均成功：附件验收队4条记录6图，三个车牌点位可独立读回；八任务验收队8条9图且八任务齐全；上传后计时仍ready。job `bash-bac2c70c6e184cfe` 已收集，exit0。
- 直接从zip在内存执行原始测试代码，仅替换URL/队名，也成功4条6图；随后真实HTTP验证锁队403、手动开始/结束、新建running拒绝409、结束后上报409、旧场次导出16条报告/21图全部可读取、新场次为空且旧场次保留3队。
- 主agent独立复跑33项单元测试：1.883s / OK / exit0，无ResourceWarning（job `bash-b98b39c5fead4031`已收集）。CLI --help、占用8080明确exit1不换端口、非法65536端口exit2均验证通过（job `bash-bb80f53930d646e2`已收集）。
- 进程级重启验证通过：停止PID9016后以同目录启动PID9637，2个场次、旧场次16条21图、锁队和finished计时均保留；全部21图仍可HTTP读取。
- 首页HTML、JS/CSS资源真实HTTP均200，DOM id唯一、CSP与nosniff头正确；这不等于浏览器渲染测试。
- 全部9份Python文件AST解析通过，无pycache生成。

## P3已完成与验证
- 新增templates/index.html、static/app.js、static/style.css及tests/test_dashboard.js。八任务卡、每卡点位按钮切换、队伍/任务/旧场次筛选、历史分页、缩略图和画廊、管理令牌、计时、导出及断连提示均实现；CSS改为可读多行，hidden强制隐藏。
- 主agent独立执行 `node --check tools/mock_judge/static/app.js && node tools/mock_judge/tests/test_dashboard.js`：exit0。覆盖250组跨页完整最新记录、同时间追加顺序、增量缓存、稳定DOM/画廊、三个车牌点位本地切换不请求、历史分页/历史队伍、当前场次管理vs所选场次导出、令牌隔离清除、安全URL/纯文本、中文文件名、finished计时冻结、中文错误、断连保留、最大in-flight=1。
- Node运行环境有UNDICI-EHPA代理实验提示，是宿主环境提示非测试失败；Python严格ResourceWarning回归无警告。
- 真实浏览器环境缺失（无Chromium/Playwright/Selenium），尚未实际验证响应式布局、native dialog焦点、浏览器图片显示/下载和移动交互；模拟DOM测试不能代替这些。

## P4交付与最终检查
- 本次仅新增17个文件（tools/mock_judge下15个，docs下计划/进度2个），未改现有tracked文件。
- 对初始9个tracked修改及2个未跟踪用户测试文件，共11个文件逐个比较SHA256，全部与实施前一致。git diff --stat仍为原先9文件/787插入/265删除；git diff --check通过。
- 全部17个新增文件末尾换行/无尾空白/Python AST检查通过，无运行数据库、图片、令牌或pycache进入仓库。
- 33项Python回归、Node语法/模拟DOM回归、原始附件与演示真实HTTP、管理与导出、进程重启及CLI错误路径均通过。2026-11-10交付时未提交、未推送；2026-11-11用户明确授权仅提交本任务17个文件，不混入其他改动。

## 2026-11-11 提交预检与范围
- 用户请求“裁判系统先提交吧，之后有问题再说”，授权提交并推送origin/main。
- 当前HEAD为0b3a4c7，原有导航/视觉的其他改动已由后续工作另行入库，不能再以11月10日的工作区状态推断当前提交范围。
- 重新执行严格ResourceWarning的33项Python测试：1.236s / OK / exit0；Node语法和模拟DOM回归也通过，浏览器交互仍未实测。
- 本次仅包含tools/mock_judge/的15个源代码/网页/示例/测试/说明文件和docs/mock_judge_plan.md、docs/mock_judge_progress.md，共17个文件。无巡航GUI地址/自检/自动上传接入改动。
- 明确排除当前其他本地改动：src/nav_demo/scripts/waypoint_gui.py、src/nav_demo/waypoints/inspection_waypoints_before_straight.yaml、src/MLX90640_交付包.zip。
- 首次暂存后的git diff --cached --check发现附件示例沿用了ZIP源文件的CRLF换行，以及CSS末尾多余空行；此前未跟踪文件未被普通git diff --check覆盖。此次仅统一示例为LF并删除CSS末尾空行，不改程序行为，重新暂存检查与复测后再提交。
- 换行清理后再次严格复测：33项Python测试1.120s / OK，Node语法与模拟DOM回归通过，git diff --cached --check通过；暂存范围确认恰好17个批准文件。
- 本节随裁判系统首次提交保存；具体提交ID和远端推送结果以Git历史及此次最终回复为准。

## 接手 / 用户下一步
1. 从工作区根目录运行 `PYTHONDONTWRITEBYTECODE=1 python3 -m tools.mock_judge`，打开http://127.0.0.1:8080；小车接入显式加 `--host 0.0.0.0` 并使用裁判电脑局域网IP。详细说明见[README](../tools/mock_judge/README.md)。默认是全新持久数据目录，不会自动加载/tmp验收数据。
2. 如需查看本轮验收记录，显式加 `--data-dir /tmp/bit-ws-mock-judge-acceptance.1KOqGH`。旧场次16条21图，当前场次为空；/tmp可能被系统清理，不作长期存储。
3. 具备真实浏览器后核对响应式布局、三车位标签切换、dialog焦点/前后图、图片解码、导出下载、移动交互及停止服务后的断连提示。这些尚未实测，不标完成。
4. 回归命令：`PYTHONDONTWRITEBYTECODE=1 python3 -W error::ResourceWarning -m unittest discover -s tools/mock_judge/tests -v`；`node --check tools/mock_judge/static/app.js`；`node tools/mock_judge/tests/test_dashboard.js`。
5. 不添加官方评分、不自动接车；若以后要做，先获取评分细则或单独确认视觉/巡航非阻塞上传设计。

## 活跃服务 / 后台任务
无活跃服务。两轮验收服务均已停止并收集输出，未修改DSH GUI或启动替代服务。
- 第一轮job `bash-14f859e230f04fe2`（PID9016）已job_kill，预期SIGTERM。
- 重启验收job `bash-ae1375934c624424`（PID9637）也已job_kill并收集，预期SIGTERM。原URL http://127.0.0.1:8080，data-dir `/tmp/bit-ws-mock-judge-acceptance.1KOqGH` 保留/tmp不入库；旧场次ID `c4fb4f784c874751adff338347db00b0` 有16条21图，当前新场次ready/空记录。
- 其他测试job均已收集，无遗漏后台工作。
