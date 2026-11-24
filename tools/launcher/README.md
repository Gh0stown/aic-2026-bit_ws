# bit_ws 一键启动面板

把原来要开三个终端手动敲的流程，做成一个 tkinter 面板：按 ①→④ 依次启动
导航 nav777（含 rviz）、底盘 newt.py、控制台 GUI、视觉识别终端，带状态灯、
单步停止、全部停止和比赛一键启动。停止某一步会同时关掉它的终端窗口，全部停止
也会清理残留窗口，反复测试不会堆一屏终端。底部「提示 / 报错处理」里有终端展示
建议和报错重开的办法（rviz 雷达报错重开 ① 基本就好）。

## 运行

```bash
python3 ~/bit_ws/tools/launcher/bit_ws_start.py
python3 ~/bit_ws/tools/launcher/bit_ws_start.py --check      # 检查路径 / 路线 / 终端
python3 ~/bit_ws/tools/launcher/bit_ws_start.py --self-test  # 只构造界面，不启动进程
```

放到桌面（推荐，双击启动）：

```bash
install -m 755 ~/bit_ws/tools/launcher/bit_ws_start.py ~/桌面/bit_ws_start.py
```

再在桌面新建 `bit_ws.desktop`（把 `/home/mowen` 换成实际用户目录）：

```ini
[Desktop Entry]
Type=Application
Name=bit_ws 一键启动
Exec=/usr/bin/python3 /home/mowen/桌面/bit_ws_start.py
Path=/home/mowen/bit_ws
Terminal=false
Icon=utilities-terminal
```

首次双击若提示不受信任，右键选「允许启动」。

## 说明

- 默认工作区 `~/bit_ws`，可用环境变量 `BIT_WS` 覆盖。
- 面板本身不依赖 rospy，只负责在新终端里执行 `roslaunch` / `rosrun`，
  停止时发 SIGINT（等同 Ctrl+C，底盘会先发零速），再按窗口标题关掉对应终端。
- 控制台路线默认 `match`（比赛专用：0~4 共同，4 号按指示牌 A/B 自动接
  `left` / `straight`），可在面板的下拉框里换。
- 命令和顺序对应 `~/桌面/readme.md` 的导航 + 视觉流程；控制台里的
  「启动视觉」和 ④ 等价，不要重复开。
