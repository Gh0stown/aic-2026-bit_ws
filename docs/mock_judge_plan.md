# 模拟裁判系统实施计划

状态：2026-11-10 用户已批准；协议兼容裁判台，不实现无依据的官方评分。

## 目标及验收
根据附件上传协议接收 crowd/sign/trash/fire/plate/temperature/meter/ebike 八类任务，支持文字、image 单图、重复 images 多图；提供网页、队伍锁定、手动计时、持久历史和 ZIP 导出。原附件客户端只改地址/队名即可运行，三次车牌单图和一次人群三图产生四条记录六张图片。hello 不启动计时，数据重启不丢失。不接管机器人，不更改现有导航/视觉接口。

附件只读路径：`/home/mowen/.dsh/attachments/v1/files/e2/e2d440e700dfa47e547b9fc45da8604f1273442cf53334a04a8c5f0bf285e033/上传案例.zip`
SHA256：`e2d440e700dfa47e547b9fc45da8604f1273442cf53334a04a8c5f0bf285e033`。
附件没有服务端/官方评分或完整计时规则；管理行为是模拟约定。

## 目录与依赖
独立 `tools/mock_judge/`，Python 3.8 + Flask + SQLite + Pillow，原生 HTML/CSS/JS 轮询，不加 ROS/catkin/Node。当前环境 Flask 3.0.3、Werkzeug 3.0.6、Pillow 10.4.0、requests 2.32.4。模块按 HTTP、存储/会话、网页、示例和测试拆分。默认数据 `~/.local/share/bit_ws/mock_judge/`，可 --data-dir 修改。默认监听 127.0.0.1:8080，局域网显式 --host 0.0.0.0。不修改现有 DSH GUI，不开放防火墙。

## 协议和状态
- POST /api/hello：JSON team，返回 ok/message=通信正常/team，不计时、不强制注册。
- POST /api/report：URL 编码或 multipart，必填 team/task/result，选填 sub_id/extra；image/images 支持混用并保序。成功返回 ok/report_id/image_count。
- 错误 JSON：400 参数错误、403 锁队不符、409 已结束、413 超限、415 媒体不支持。
- GET /api/state、GET /api/reports 分页过滤、GET /api/images/<id>。
- 管理令牌保护 POST /api/admin/team-lock、POST /api/admin/session/{start,finish,new}、GET /api/admin/export；禁止跨域管理。普通上传不加令牌以兼容附件。
- ready→running→finished。ready 允许测试上报；手动开始按服务器连续时间计时，重启也计入；finish 冻结并拒收；new 保留旧记录。不设自动倒计时/扣分。
- 未锁队允许多队，锁队后 hello/report 都拒绝其他队。历史按 session/team/task/sub_id 分组，每次上传追加，默认最新；无请求 ID 不承诺幂等。
- SQLite 保存 schema_version、场次、锁队、自检、上传、图片、管理事件；UUID 文件名，先校验暂存再事务落盘，失败清理。管理与上报写入串行化。
- ZIP 导出版本化 JSON、CSV、图片，CSV 防公式注入。

## 安全和失败处理
32 MiB 请求，最多16图，8 MiB/图，2000万像素/图（可配置）；JPEG/PNG 检查内容而非信任 MIME/后缀。必填拒绝空白，team/sub_id 最多128字符，result/extra最多10000字符。原文保存，网页纯文本渲染。路径不取客户端文件名。存储失败回滚，启动目录/端口失败明确报错，debug关闭，仅适合可信局域网联调。

## 阶段及交接要求
- P0：本计划和进度基线。
- P1：hello/report、图像、SQLite、基础测试，优先取得无网页闭环。
- P2：锁队、会话计时、恢复和导出，测试并发/状态。
- P3：中文网页、八任务卡片、点位、历史、画廊、管理和断连状态。
- P4：附件示例/八任务演示、中文使用与接口说明、真实 HTTP 验收和最终差异检查。

每完成可验证小项立即更新 docs/mock_judge_progress.md，记录文件、测试命令和结果、决策、失败原因、下一步；未验证不标完成。启动服务记录 job/PID/URL/数据目录/停止命令。阶段结束及中断前写恢复步骤。测试用 unittest/Flask client/临时目录/注入时钟；不启相机ROS、不写仓库运行数据。具备浏览器才报告页面实测，否则标记人工验收待做。

## 保护现状
初始已修改9个tracked文件：README.md、docs/stable_baseline.md、docs/waypoint_route_plan.md、src/nav_demo/config/inspection_routes.yaml、src/nav_demo/scripts/cruise_debug.py、src/nav_demo/scripts/waypoint_gui.py、src/nav_demo/tests/test_cruise_timing.py、src/nav_demo/waypoints/inspection_waypoints.yaml、src/photo_capture/scripts/capture_photos.py；另有未跟踪src/nav_demo/tests/test_waypoint_gui.py和src/photo_capture/tests/。全部保留，不混入本次提交。

实施后 git status/diff 检查，汇总本次新增内容和未验证项，再询问是否上传；不自动 commit/push。
