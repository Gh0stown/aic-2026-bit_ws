#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bit_ws 一键启动面板（桌面版）。

把原来要开三个终端手动敲的流程，变成按 ①→②→③→④ 顺序点按钮：

  ① 导航 nav777      雷达 / 定位 / move_base / rviz（rviz 会自己开）
  ② 底盘 newt.py     把 /cmd_vel 写给底盘，不开车不会动
  ③ 控制台 GUI       巡航 / 识别 / 裁判上报（测试控制台 / 比赛控制台）
  ④ 视觉识别终端     start_vision.sh，统一视觉服务 /inspection_vision

比赛时只需要看 rviz（① 会开）和 ④ 的视觉终端；控制台里也有「启动视觉」按钮，
和 ④ 等价，不要重复开。关闭本面板不会停止已启动的进程，请用「全部停止」或
各步骤右侧的「停止」（发 SIGINT，等同 Ctrl+C，底盘会先发零速再退出）。

「停止」会顺便关掉这一步的终端窗口，「全部停止」也会清理残留窗口，反复测试
不会堆一屏终端。控制台路线默认 match（比赛按 4 号指示牌自动接 left / straight）。

面板底部「提示 / 报错处理」里写了：需要展示哪些终端、报错先停止再重开
（比如 rviz 雷达报错，重开 ① 基本就好）、以及不要重复开进程等注意事项。

用法：
    python3 ~/bit_ws/tools/launcher/bit_ws_start.py
    桌面上也有一份：python3 ~/桌面/bit_ws_start.py
    （桌面配了 bit_ws.desktop，双击图标即可；首次双击若不受信任，
      右键 → 允许启动）

维护：改完本文件后同步到桌面：
    install -m 755 tools/launcher/bit_ws_start.py ~/桌面/bit_ws_start.py
路径默认 ~/bit_ws，可用环境变量 BIT_WS 覆盖。
"""

import os
import queue
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from urllib.parse import urlparse

import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox, ttk

APP_TITLE = 'bit_ws 一键启动'

HOME = os.path.expanduser('~')
WS = os.environ.get('BIT_WS') or os.path.join(HOME, 'bit_ws')
ROS_SETUP = '/opt/ros/noetic/setup.bash'
WS_SETUP = os.path.join(WS, 'devel', 'setup.bash')
NAV777_LAUNCH = os.path.join(WS, 'src', 'nav777.launch')
VISION_SCRIPT = os.path.join(WS, 'src', 'nav_demo', 'scripts', 'start_vision.sh')
ROUTES_YAML = os.path.join(WS, 'src', 'nav_demo', 'config', 'inspection_routes.yaml')

DEFAULT_ROUTES = ['match', 'left', 'straight', 'left_noopt', 'straight_noopt',
                  'test_789', 'test_strafe', 'test_15']

STEPS = {
    1: {
        'name': '导航 nav777',
        'desc': '雷达 / 定位 / move_base / rviz（启动后等 5~10 秒）',
        'pattern': r'roslaunch .*nav777\.launch',
    },
    2: {
        'name': '底盘驱动 newt.py',
        'desc': '把 /cmd_vel 写给底盘；不启动车不会动',
        'pattern': r'car_bringup/scripts/newt\.py',
    },
    3: {
        'name': '控制台 GUI',
        'desc': '巡航 / 识别 / 裁判上报（比赛用精简版，match 路线）',
        'pattern': r'nav_demo/scripts/(console|cruise_debug|match_console)\.py',
    },
    4: {
        'name': '视觉识别终端',
        'desc': 'start_vision.sh：统一视觉服务 /inspection_vision',
        'pattern': r'start_vision\.sh|inspection_vision\.py',
    },
}
RV_PATTERN = r'rviz .*single_nav\.rviz'

# 每个步骤打开的终端窗口标题前缀；停止时按标题前缀把对应窗口一起关掉。
WINDOW_PREFIX = {
    1: 'bit_ws ①',
    2: 'bit_ws ②',
    3: 'bit_ws ③',
    4: 'bit_ws ④',
}

# 配色
BG = '#eef2f7'
HEADER_BG = '#1f2d3d'
CARD_BG = '#ffffff'
BORDER = '#ccd5e0'
ACCENT = '#0b63ce'
ACCENT_DARK = '#084a9c'
OK = '#15803d'
WARN = '#b45309'
MUTED = '#94a3b8'
TEXT = '#1f2937'

# 「提示 / 报错处理」窗口里的内容：小标题 + 条目
HELP_SECTIONS = [
    ('需要展示的终端', [
        '比赛：保留 rviz（① 自动打开）和 ④ 视觉终端；底盘、控制台终端最小化即可。',
        '调试：nav777 终端留着看雷达 / 定位 / move_base 报错；底盘终端看串口；'
        '控制台终端看巡航日志。',
    ]),
    ('报错怎么处理（先停止，再重开）', [
        'rviz 里雷达报错 / 没有点云 / 定位漂：在 ① 点「停止」，等状态灯变灰，'
        '再点「启动」重开 nav777，基本就好。',
        '底盘不动 / 串口报错：重开 ②；还不行检查 /dev/carserial 和 PS2 模式'
        '（python3 src/mbot_bringup/scripts_can_york/ps2_serial.py）。',
        '视觉终端报错、/inspection_vision 服务没起来：重开 ④；确认相机没被 '
        'Orbbec 节点占用。',
        '控制台连不上 ROS 或服务报错：重开 ③。',
        '改过地图 / 点位 / 路线：先「全部停止」，再按 ①→④ 重新来。',
    ]),
    ('注意', [
        '重开前一定先停止，不要同时开两个 roslaunch 或两个 /inspection_vision。',
        '「停止」会同时关掉这一步的终端窗口；「全部停止」会把残留的终端窗口一起清掉。',
        '控制台里的「启动视觉」和 ④ 等价，别重复点。',
        '关闭面板不会停进程；收工请用「全部停止」。',
        '默认控制台路线是 match，按比赛自动接 left / straight。',
        '自检：python3 ~/桌面/bit_ws_start.py --check',
    ]),
]


# --------------------------------------------------------------------------
# 系统小工具
# --------------------------------------------------------------------------
def ros_master_up(timeout=0.25):
    """探测 ROS master 端口是否在监听（默认 127.0.0.1:11311，可用 ROS_MASTER_URI 覆盖）。"""
    host, port = '127.0.0.1', 11311
    uri = os.environ.get('ROS_MASTER_URI')
    if uri:
        try:
            parsed = urlparse(uri)
            host = parsed.hostname or host
            port = parsed.port or port
        except ValueError:
            pass
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        return sock.connect_ex((host, port)) == 0
    except OSError:
        return False
    finally:
        sock.close()


def pid_alive(pid):
    try:
        with open('/proc/%d/stat' % pid, 'r') as handle:
            data = handle.read()
        return data.rsplit(')', 1)[1].split()[0] != 'Z'
    except (OSError, IndexError):
        return False


def find_pids(pattern):
    """按命令行正则查进程（pgrep -f），排除启动器自己。"""
    try:
        result = subprocess.run(['pgrep', '-f', pattern],
                                capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.SubprocessError):
        return []
    pids = []
    own = os.getpid()
    for token in result.stdout.split():
        try:
            pid = int(token)
        except ValueError:
            continue
        if pid != own and pid_alive(pid):
            pids.append(pid)
    return pids


def _wait_gone(pids, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not any(pid_alive(pid) for pid in pids):
            return True
        time.sleep(0.2)
    return not any(pid_alive(pid) for pid in pids)


def terminate(pids, log, grace=6.0):
    """先 SIGINT（等同 Ctrl+C），不退再 SIGTERM，最后 SIGKILL。"""
    pids = [pid for pid in pids if pid_alive(pid)]
    if not pids:
        return
    for pid in pids:
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            pass
    log('已发送 SIGINT（等同 Ctrl+C）给 %d 个进程，等待退出 …' % len(pids))
    if _wait_gone(pids, grace):
        log('已停止。')
        return
    left = [pid for pid in pids if pid_alive(pid)]
    for pid in left:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    if _wait_gone(left, 3.0):
        log('SIGINT 没退，已用 SIGTERM 停止。')
        return
    left = [pid for pid in pids if pid_alive(pid)]
    for pid in left:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    log('已强制结束（SIGKILL）。')


def find_step_windows(prefixes):
    """用 xwininfo 找标题以 prefixes 开头的终端窗口，返回窗口 id 列表。"""
    try:
        result = subprocess.run(['xwininfo', '-root', '-tree'],
                                capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.SubprocessError):
        return []
    windows = []
    for line in result.stdout.splitlines():
        match = re.match(r'\s+(0x[0-9a-fA-F]+)\s+"([^"]*)"', line)
        if not match:
            continue
        name = match.group(2)
        if any(name.startswith(prefix) for prefix in prefixes):
            windows.append(int(match.group(1), 16))
    return windows


def _send_wm_delete(window_id):
    """给窗口发 WM_DELETE_WINDOW，效果等同点终端窗口的关闭按钮。"""
    display = None
    try:
        import ctypes
        x11 = ctypes.CDLL('libX11.so.6')
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XInternAtom.restype = ctypes.c_ulong
        x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        x11.XSendEvent.restype = ctypes.c_int
        x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                                   ctypes.c_long, ctypes.c_void_p]
        x11.XFlush.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        display = x11.XOpenDisplay(None)
        if not display:
            return False
        protocols = x11.XInternAtom(display, b'WM_PROTOCOLS', 0)
        delete = x11.XInternAtom(display, b'WM_DELETE_WINDOW', 0)
        event = ctypes.create_string_buffer(192)
        ctypes.c_int.from_buffer(event, 0).value = 33
        ctypes.c_ulong.from_buffer(event, 8).value = 0
        ctypes.c_int.from_buffer(event, 16).value = 0
        ctypes.c_void_p.from_buffer(event, 24).value = display
        ctypes.c_ulong.from_buffer(event, 32).value = window_id
        ctypes.c_ulong.from_buffer(event, 40).value = protocols
        ctypes.c_int.from_buffer(event, 48).value = 32
        ctypes.c_ulong.from_buffer(event, 56).value = delete
        ctypes.c_ulong.from_buffer(event, 64).value = 0
        ok = bool(x11.XSendEvent(display, window_id, 0, 0, event))
        x11.XFlush(display)
        return ok
    except Exception:
        return False
    finally:
        if display is not None:
            try:
                x11.XCloseDisplay(display)
            except Exception:
                pass


def close_step_windows(sid, log, wait=2.0):
    """关掉某一步的终端窗口；返回找到的窗口数（0 表示本来就没开）。"""
    prefixes = [WINDOW_PREFIX[sid]]
    windows = find_step_windows(prefixes)
    if not windows:
        return 0
    for window in windows:
        _send_wm_delete(window)
    deadline = time.time() + wait
    while time.time() < deadline:
        if not find_step_windows(prefixes):
            break
        time.sleep(0.1)
    left = find_step_windows(prefixes)
    if left:
        log('已结束进程，但还有 %d 个终端窗口没关掉，可以手动关。' % len(left))
    else:
        log('已关闭对应终端窗口。')
    return len(windows)


def build_terminal_argv(title, command):
    """生成「新开终端并执行命令」的 argv；找不到终端返回 None。"""
    inner = (
        'source {ros} 2>/dev/null; '
        'source {ws} 2>/dev/null; '
        'cd {cwd} 2>/dev/null; '
        '{cmd}; '
        'echo; echo "[bit_ws] 命令已结束，可以关闭这个终端。"; '
        'exec bash'
    ).format(ros=shlex.quote(ROS_SETUP),
             ws=shlex.quote(WS_SETUP),
             cwd=shlex.quote(WS),
             cmd=command)
    if shutil.which('gnome-terminal'):
        return ['gnome-terminal', '--title', title, '--', 'bash', '-lc', inner]
    if shutil.which('x-terminal-emulator'):
        return ['x-terminal-emulator', '-T', title, '-e', 'bash', '-lc', inner]
    if shutil.which('xterm'):
        return ['xterm', '-T', title, '-e', 'bash', '-lc', inner]
    return None


def spawn_terminal(argv):
    """在新终端里执行命令（单独成一个函数，方便测试替换）。"""
    return subprocess.Popen(argv, start_new_session=True,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def load_routes():
    try:
        import yaml
        with open(ROUTES_YAML, 'r') as handle:
            data = yaml.safe_load(handle) or {}
        names = list((data.get('routes') or {}).keys())
        if names:
            return names
    except Exception:  # noqa: BLE001 - 读不到就用内置列表
        pass
    return list(DEFAULT_ROUTES)


def pick_font(root, candidates, fallback='TkDefaultFont'):
    try:
        families = set(tkfont.families(root))
    except tk.TclError:
        return fallback
    for name in candidates:
        if name in families:
            return name
    return fallback


def console_script(kind):
    return 'match_console.py' if kind == 'match' else 'cruise_debug.py'


# --------------------------------------------------------------------------
# 主界面
# --------------------------------------------------------------------------
class LauncherApp(object):

    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.routes = load_routes()
        self.console_kind = tk.StringVar(value='test')
        self.route = tk.StringVar(
            value='match' if 'match' in self.routes else self.routes[0])
        self.launching_until = {}
        self.stopping = set()
        self.match_queue = []
        self.match_deadline = 0.0
        self.match_wait_until = 0.0
        self.status_dots = {}
        self.state_labels = {}
        self.start_buttons = {}
        self.stop_buttons = {}
        self.log_count = 0

        self.family = pick_font(root, [
            'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'Source Han Sans SC',
            'Droid Sans Fallback', 'DejaVu Sans'])
        self.mono = pick_font(root, [
            'Noto Sans Mono CJK SC', 'DejaVu Sans Mono', 'Monospace'])

        self._build_ui()
        self._bind_keys()
        self.root.report_callback_exception = self._on_tk_error
        self.log('面板已就绪：按 1 → 2 → 3 → 4 顺序启动；比赛也可直接点「比赛一键启动」。')
        self._refresh()
        self.root.after(1000, self._poll)

    # ---------------- 界面 ----------------
    def _build_ui(self):
        self.root.title(APP_TITLE)
        width = min(880, max(760, self.root.winfo_screenwidth() - 120))
        height = min(820, max(680, self.root.winfo_screenheight() - 120))
        self.root.geometry('%dx%d' % (width, height))
        self.root.minsize(760, 680)
        self.root.configure(bg=BG)

        header = tk.Frame(self.root, bg=HEADER_BG)
        header.pack(fill='x')
        tk.Label(header, text=APP_TITLE, bg=HEADER_BG, fg='white',
                 font=(self.family, 19, 'bold')).pack(
                     anchor='w', padx=18, pady=(12, 0))
        tk.Label(header,
                 text='按顺序启动：① 导航 → ② 底盘 → ③ 控制台 → ④ 视觉　'
                      '比赛时保留 rviz 和视觉终端即可',
                 bg=HEADER_BG, fg='#b9c4d0',
                 font=(self.family, 9)).pack(anchor='w', padx=18, pady=(2, 12))

        bar = tk.Frame(self.root, bg=BG)
        bar.pack(fill='x', padx=16, pady=(10, 0))
        status_items = [('ros', 'ROS master'), ('rviz', 'rviz'),
                        ('1', '导航'), ('2', '底盘'), ('3', '控制台'),
                        ('4', '视觉')]
        for index, (key, text) in enumerate(status_items):
            cell = tk.Frame(bar, bg=BG)
            cell.grid(row=0, column=index, sticky='w', padx=(0, 16))
            dot = tk.Label(cell, text='●', bg=BG, fg=MUTED,
                           font=(self.family, 11))
            dot.pack(side='left')
            tk.Label(cell, text=text, bg=BG, fg='#4b5563',
                     font=(self.family, 9)).pack(side='left', padx=(3, 0))
            self.status_dots[key] = dot

        body = tk.Frame(self.root, bg=BG)
        body.pack(fill='x', padx=16, pady=(8, 0))
        for sid in (1, 2, 3, 4):
            self._build_card(body, sid)

        self.hint = tk.Label(self.root, text='', bg=BG, fg='#374151',
                             anchor='w', justify='left', wraplength=840,
                             font=(self.family, 10))
        self.hint.pack(fill='x', padx=18, pady=(8, 0))

        self.tips = tk.Label(
            self.root,
            text='展示：比赛保留 rviz + ④ 视觉终端，其余终端可最小化；调试时各终端都留着看日志。'
                 '报错先「停止」再「启动」对应步骤，rviz 雷达报错重开 ① 基本就好'
                 '（详见「提示 / 报错处理」）。',
            bg=BG, fg='#64748b', anchor='w', justify='left', wraplength=840,
            font=(self.family, 9))
        self.tips.pack(fill='x', padx=18, pady=(4, 0))
        self.root.bind('<Configure>', self._on_resize)

        actions = tk.Frame(self.root, bg=BG)
        actions.pack(fill='x', padx=16, pady=(10, 0))
        tk.Button(actions, text='比赛一键启动（①→④）', command=self.start_match,
                  bg=OK, fg='white', activebackground='#116232',
                  activeforeground='white', relief='flat', bd=0,
                  cursor='hand2', font=(self.family, 10, 'bold'),
                  padx=16, pady=7).pack(side='left')
        tk.Button(actions, text='全部停止', command=self.stop_all,
                  bg='#b91c1c', fg='white', activebackground='#991b1b',
                  activeforeground='white', relief='flat', bd=0,
                  cursor='hand2', font=(self.family, 10, 'bold'),
                  padx=16, pady=7).pack(side='left', padx=(10, 0))
        tk.Button(actions, text='提示 / 报错处理', command=self.show_help,
                  bg='#e2e8f0', fg='#334155', activebackground='#cbd5e1',
                  relief='flat', bd=0, cursor='hand2',
                  font=(self.family, 10), padx=16, pady=7).pack(
                      side='left', padx=(10, 0))
        tk.Button(actions, text='关闭面板', command=self.on_close,
                  bg='#e2e8f0', fg='#334155', activebackground='#cbd5e1',
                  relief='flat', bd=0, cursor='hand2',
                  font=(self.family, 10), padx=16, pady=7).pack(side='right')
        tk.Label(actions, text='按键 1/2/3/4 启动，0 全部停止', bg=BG,
                 fg='#64748b', font=(self.family, 9)).pack(
                     side='right', padx=(0, 14))

        log_frame = tk.Frame(self.root, bg=BG)
        log_frame.pack(fill='both', expand=True, padx=16, pady=(10, 12))
        self.log_text = tk.Text(log_frame, height=7, bg='#111827',
                                fg='#d1d5db', insertbackground='white',
                                relief='flat', bd=0, wrap='word',
                                font=(self.mono, 9), state='disabled')
        scroll = tk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.log_text.pack(side='left', fill='both', expand=True)

    def _build_card(self, parent, sid):
        card = tk.Frame(parent, bg=CARD_BG, highlightbackground=BORDER,
                        highlightthickness=1, bd=0)
        card.pack(fill='x', pady=(0, 7))
        inner = tk.Frame(card, bg=CARD_BG)
        inner.pack(fill='x', padx=14, pady=9)
        inner.grid_columnconfigure(1, weight=1)

        tk.Label(inner, text='①②③④'[sid - 1], bg=CARD_BG, fg=ACCENT,
                 font=(self.family, 20, 'bold')).grid(
                     row=0, column=0, rowspan=2, padx=(0, 12))
        tk.Label(inner, text=STEPS[sid]['name'], bg=CARD_BG, fg=TEXT,
                 font=(self.family, 12, 'bold')).grid(
                     row=0, column=1, sticky='w')
        tk.Label(inner, text=STEPS[sid]['desc'], bg=CARD_BG, fg='#64748b',
                 font=(self.family, 9)).grid(
                     row=1, column=1, sticky='w', pady=(2, 0))

        state = tk.Label(inner, text='未启动', bg=CARD_BG, fg=MUTED,
                         font=(self.family, 10))
        state.grid(row=0, column=2, rowspan=2, padx=(10, 12))
        self.state_labels[sid] = state

        start = tk.Button(inner, text='启动',
                          command=lambda s=sid: self.start_step(s),
                          bg=ACCENT, fg='white', activebackground=ACCENT_DARK,
                          activeforeground='white', relief='flat', bd=0,
                          cursor='hand2', font=(self.family, 10, 'bold'),
                          width=8, pady=4)
        start.grid(row=0, column=3, rowspan=2, padx=(0, 6))
        self.start_buttons[sid] = start

        stop = tk.Button(inner, text='停止',
                         command=lambda s=sid: self.stop_steps([s]),
                         bg='#e2e8f0', fg='#334155',
                         activebackground='#cbd5e1', relief='flat', bd=0,
                         cursor='hand2', font=(self.family, 10),
                         width=6, pady=4, state='disabled')
        stop.grid(row=0, column=4, rowspan=2)
        self.stop_buttons[sid] = stop

        if sid == 3:
            ctrl = tk.Frame(card, bg=CARD_BG)
            ctrl.pack(fill='x', padx=14, pady=(0, 10))
            tk.Label(ctrl, text='控制台类型', bg=CARD_BG, fg='#475569',
                     font=(self.family, 9)).pack(side='left')
            tk.Radiobutton(ctrl, text='测试（完整）', variable=self.console_kind,
                           value='test', bg=CARD_BG, activebackground=CARD_BG,
                           selectcolor='white', font=(self.family, 9),
                           fg=TEXT).pack(side='left', padx=(6, 2))
            tk.Radiobutton(ctrl, text='比赛（精简）', variable=self.console_kind,
                           value='match', bg=CARD_BG, activebackground=CARD_BG,
                           selectcolor='white', font=(self.family, 9),
                           fg=TEXT).pack(side='left', padx=(2, 12))
            tk.Label(ctrl, text='路线', bg=CARD_BG, fg='#475569',
                     font=(self.family, 9)).pack(side='left')
            combo = ttk.Combobox(ctrl, textvariable=self.route,
                                 values=self.routes, state='readonly',
                                 width=12, font=(self.family, 9))
            combo.pack(side='left', padx=(6, 0))
            tk.Label(ctrl, text='（比赛用「比赛一键启动」会自动选 match 路线）',
                     bg=CARD_BG, fg='#94a3b8',
                     font=(self.family, 8)).pack(side='left', padx=(10, 0))

    def _bind_keys(self):
        for sid in (1, 2, 3, 4):
            self.root.bind(str(sid),
                           lambda event, s=sid: self._key_start(s))
        self.root.bind('0', self._key_stop_all)
        self.root.bind('<Control-q>', lambda event: self.on_close())
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    def _on_resize(self, event):
        if event.widget is self.root:
            wrap = max(600, event.width - 60)
            self.hint.configure(wraplength=wrap)
            self.tips.configure(wraplength=wrap)

    def _on_tk_error(self, exc, value, tb):
        self.log('界面回调异常：%s' % value)
        traceback.print_exception(exc, value, tb)

    # ---------------- 日志 ----------------
    def log(self, message):
        timestamp = time.strftime('%H:%M:%S')
        self.log_text.configure(state='normal')
        self.log_text.insert('end', '[%s] %s\n' % (timestamp, message))
        self.log_count += 1
        if self.log_count > 300:
            self.log_text.delete('1.0', '2.0')
            self.log_count -= 1
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    # ---------------- 状态刷新 ----------------
    def _refresh(self):
        self._drain_events()
        master = ros_master_up()
        running = {sid: bool(find_pids(STEPS[sid]['pattern']))
                   for sid in STEPS}
        rviz = bool(find_pids(RV_PATTERN))
        now = time.time()
        self._update_status(master, running, rviz, now)
        self._update_buttons(master, running, now)
        self._update_hint(master, running)
        self._advance_match(master, running, now)

    def _poll(self):
        try:
            self._refresh()
        except Exception:  # noqa: BLE001 - 界面不能因为一次查询失败就死掉
            self.log('状态刷新失败：%s' % sys.exc_info()[1])
        self.root.after(1000, self._poll)

    def _drain_events(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                return
            if kind == 'log':
                self.log(payload)
            elif kind == 'stop_done':
                self.stopping.discard(payload)
                self.launching_until.pop(payload, None)
            elif kind == 'fatal':
                self.log(payload)

    def _update_status(self, master, running, rviz, now):
        self.status_dots['ros'].configure(fg=OK if master else MUTED)
        self.status_dots['rviz'].configure(fg=OK if rviz else MUTED)
        for sid in (1, 2, 3, 4):
            dot = self.status_dots[str(sid)]
            if sid in self.stopping:
                dot.configure(fg=WARN)
                self.state_labels[sid].configure(text='停止中…', fg=WARN)
            elif running[sid]:
                dot.configure(fg=OK)
                self.state_labels[sid].configure(text='运行中', fg=OK)
            elif now < self.launching_until.get(sid, 0):
                dot.configure(fg=WARN)
                self.state_labels[sid].configure(text='启动中…', fg=WARN)
            else:
                dot.configure(fg=MUTED)
                self.state_labels[sid].configure(text='未启动', fg=MUTED)

    def _update_buttons(self, master, running, now):
        for sid in (1, 2, 3, 4):
            start = self.start_buttons[sid]
            if running[sid]:
                start.configure(text='运行中', state='disabled')
            elif sid in self.stopping:
                start.configure(text='停止中…', state='disabled')
            elif now < self.launching_until.get(sid, 0):
                start.configure(text='启动中…', state='disabled')
            elif sid != 1 and not master:
                start.configure(text='等 ROS', state='disabled')
            else:
                start.configure(text='启动', state='normal')
            stop = self.stop_buttons[sid]
            active = (running[sid] or sid in self.stopping
                      or now < self.launching_until.get(sid, 0))
            stop.configure(state='normal' if active and sid not in self.stopping
                           else 'disabled')

    def _update_hint(self, master, running):
        if self.match_queue:
            text = '比赛一键启动进行中：① nav777 → ② 底盘 → ③ 比赛控制台 → ④ 视觉终端 …'
        elif not master:
            text = ('第一步：按 1 启动 nav777（雷达 / 定位 / 导航 / rviz）。'
                    '等 rviz 窗口出来、状态灯变绿后再按 2。')
        elif not running[1]:
            text = 'ROS 已就绪，但没检测到 nav777；按 1 启动导航。'
        elif not running[2]:
            text = '第二步：按 2 启动底盘 newt.py，不然车不会动（现在 ROS 已就绪）。'
        elif not running[3]:
            text = ('第三步：按 3 打开控制台 GUI；比赛建议用下方「比赛一键启动」'
                    '（自动选比赛控制台 + match 路线）。')
        elif not running[4]:
            text = ('第四步：按 4 打开视觉识别终端。比赛时保留 rviz 和视觉终端即可；'
                    '控制台里的「启动视觉」和 ④ 等价，别重复开。')
        else:
            text = ('全部就绪。比赛展示：rviz + ④ 视觉终端，其他终端可最小化；'
                    '报错先停止再重开（详见「提示 / 报错处理」）。关闭面板不会停进程。')
        self.hint.configure(text=text)

    # ---------------- 提示 / 报错处理 ----------------
    def show_help(self):
        win = tk.Toplevel(self.root)
        win.title('使用提示 / 报错处理')
        win.configure(bg=BG)
        win.transient(self.root)
        width = 700
        height = min(620, max(420, self.root.winfo_screenheight() - 200))
        x = self.root.winfo_rootx() + 40
        y = self.root.winfo_rooty() + 20
        win.geometry('%dx%d+%d+%d' % (width, height, x, y))
        win.minsize(560, 400)

        tk.Label(win, text='使用提示 / 报错处理', bg=HEADER_BG, fg='white',
                 anchor='w', padx=14, pady=8,
                 font=(self.family, 13, 'bold')).pack(fill='x')
        frame = tk.Frame(win, bg=BG)
        frame.pack(fill='both', expand=True, padx=12, pady=10)
        text = tk.Text(frame, bg='white', fg=TEXT, relief='flat', bd=0,
                       wrap='word', font=(self.family, 10), padx=12, pady=8,
                       spacing1=2, spacing3=2)
        scroll = tk.Scrollbar(frame, command=text.yview)
        text.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        text.pack(side='left', fill='both', expand=True)
        text.tag_configure('h', font=(self.family, 11, 'bold'),
                           foreground=ACCENT, spacing1=10, spacing3=4)
        text.tag_configure('line', font=(self.family, 10),
                           foreground=TEXT, lmargin1=14, lmargin2=28)
        for title, lines in HELP_SECTIONS:
            text.insert('end', title + '\n', 'h')
            for line in lines:
                text.insert('end', '• ' + line + '\n', 'line')
            text.insert('end', '\n')
        text.configure(state='disabled')
        tk.Button(win, text='知道了', command=win.destroy,
                  bg=ACCENT, fg='white', activebackground=ACCENT_DARK,
                  activeforeground='white', relief='flat', bd=0,
                  cursor='hand2', font=(self.family, 10, 'bold'),
                  padx=22, pady=6).pack(pady=(0, 10))

    # ---------------- 启动 ----------------
    def _typing(self):
        widget = self.root.focus_get()
        return isinstance(widget, (tk.Entry, ttk.Entry, ttk.Combobox))

    def _key_start(self, sid):
        if self._typing():
            return
        self.start_step(sid)

    def _key_stop_all(self, event=None):
        if self._typing():
            return
        self.stop_all()

    def start_step(self, sid, console_kind=None, route=None):
        if sid in self.stopping:
            self.log('正在停止，稍等再启动。')
            return False
        if find_pids(STEPS[sid]['pattern']):
            self.log('%s 已在运行，不重复启动。' % STEPS[sid]['name'])
            return False
        if sid in (2, 3, 4) and not ros_master_up():
            self.log('还连不上 ROS master：请先按 1 启动 nav777，等 rviz 出来再按 %d。' % sid)
            return False

        title = 'bit_ws %s %s' % ('①②③④'[sid - 1], STEPS[sid]['name'])
        if sid == 1:
            if not os.path.isfile(NAV777_LAUNCH):
                self.log('找不到 %s，检查工作区路径（当前 %s）。' % (NAV777_LAUNCH, WS))
                return False
            command = 'roslaunch %s' % shlex.quote(NAV777_LAUNCH)
        elif sid == 2:
            command = 'rosrun car_bringup newt.py'
        elif sid == 3:
            kind = console_kind or self.console_kind.get()
            route = route or self.route.get()
            if route not in self.routes:
                self.log('未知路线 %r，可选：%s' % (route, ', '.join(self.routes)))
                return False
            command = 'rosrun nav_demo %s --route %s' % (
                console_script(kind), shlex.quote(route))
        else:
            if not os.path.isfile(VISION_SCRIPT):
                self.log('找不到 %s。' % VISION_SCRIPT)
                return False
            if not os.access(VISION_SCRIPT, os.X_OK):
                self.log('%s 没有执行权限，执行：chmod +x %s'
                         % (VISION_SCRIPT, VISION_SCRIPT))
                return False
            command = shlex.quote(VISION_SCRIPT)

        argv = build_terminal_argv(title, command)
        if argv is None:
            self.log('找不到可用的终端（gnome-terminal / x-terminal-emulator / xterm）。')
            return False
        try:
            spawn_terminal(argv)
        except OSError as error:
            self.log('打开终端失败：%s' % error)
            return False
        self.launching_until[sid] = time.time() + 15
        self.log('%s：%s' % (STEPS[sid]['name'], command))
        return True

    # ---------------- 比赛一键启动 ----------------
    def start_match(self):
        if self.match_queue:
            self.log('比赛一键启动已经在进行了。')
            return
        if not messagebox.askyesno(
                '比赛一键启动',
                '将按顺序启动：\n\n'
                '① 导航 nav777（含 rviz）\n'
                '② 底盘 newt.py\n'
                '③ 比赛控制台（精简版，match 路线）\n'
                '④ 视觉识别终端\n\n'
                '已经在运行的步骤会自动跳过。继续？',
                parent=self.root):
            return
        self.match_queue = [1, 2, 3, 4]
        self.match_deadline = time.time() + 120
        self.match_wait_until = 0.0
        self.log('比赛一键启动：① nav777 → ② 底盘 → ③ 比赛控制台（match）→ ④ 视觉终端')

    def _advance_match(self, master, running, now):
        if not self.match_queue:
            return
        if time.time() > self.match_deadline:
            self.match_queue = []
            self.log('比赛一键启动超时（120 秒）。检查 nav777 终端里的报错，'
                     '必要时手动点各步骤的「启动」。')
            return
        if now < self.match_wait_until:
            return
        sid = self.match_queue[0]
        if sid == 1:
            self.match_queue.pop(0)
            if running[1]:
                self.log('① 导航已在运行，跳过。')
            else:
                self.start_step(1)
            self.match_wait_until = now + 1.0
        elif sid == 2:
            if not master:
                return
            self.match_queue.pop(0)
            if running[2]:
                self.log('② 底盘已在运行，跳过。')
            else:
                self.start_step(2)
                self.match_wait_until = now + 2.5
        elif sid == 3:
            if not master:
                return
            self.match_queue.pop(0)
            if running[3]:
                self.log('③ 控制台已在运行，跳过（当前若是测试控制台，'
                         '比赛请手动换成比赛控制台）。')
            else:
                self.start_step(3, console_kind='match', route='match')
                self.match_wait_until = now + 2.5
        elif sid == 4:
            if not master:
                return
            self.match_queue.pop(0)
            if running[4]:
                self.log('④ 视觉终端已在运行，跳过。')
            else:
                self.start_step(4)
                self.match_wait_until = now + 1.0

    # ---------------- 停止 ----------------
    def stop_steps(self, sids):
        todo = [sid for sid in sids if sid not in self.stopping]
        if not todo:
            return
        for sid in todo:
            self.stopping.add(sid)
        threading.Thread(target=self._stop_worker, args=(todo,),
                         daemon=True).start()

    def _stop_worker(self, sids):
        for sid in sids:
            name = STEPS[sid]['name']
            if self.launching_until.get(sid, 0) > time.time():
                time.sleep(1.2)
            pids = find_pids(STEPS[sid]['pattern'])
            if pids:
                terminate(pids,
                          lambda message, n=name: self.events.put(
                              ('log', '%s：%s' % (n, message))),
                          grace=8.0 if sid == 1 else 5.0)
            windows = close_step_windows(
                sid,
                lambda message, n=name: self.events.put(
                    ('log', '%s：%s' % (n, message))))
            if not pids and not windows:
                self.events.put(('log', '%s 没有发现运行中的进程。' % name))
            self.events.put(('stop_done', sid))

    def stop_all(self):
        active = [sid for sid in (1, 2, 3, 4)
                  if find_pids(STEPS[sid]['pattern'])
                  or time.time() < self.launching_until.get(sid, 0)
                  or sid in self.stopping]
        windows = find_step_windows(list(WINDOW_PREFIX.values()))
        if not active and not windows:
            self.log('没有发现运行中的步骤，也没有残留的终端窗口。')
            return
        if active:
            message = ('将停止导航、底盘、控制台、视觉（包括不是本面板启动的），'
                       '并关闭对应终端窗口。\n\n底盘会先收到 Ctrl+C，发零速后再退出。继续？')
        else:
            message = ('没有运行中的进程，将关闭面板打开的残留终端窗口。继续？')
        if not messagebox.askyesno('全部停止', message, parent=self.root):
            return
        self.log('全部停止：视觉 → 控制台 → 底盘 → 导航（含关闭终端窗口）')
        self.stop_steps([4, 3, 2, 1])

    def on_close(self):
        active = [sid for sid in (1, 2, 3, 4)
                  if find_pids(STEPS[sid]['pattern'])
                  or time.time() < self.launching_until.get(sid, 0)]
        windows = find_step_windows(list(WINDOW_PREFIX.values()))
        if active or windows:
            answer = messagebox.askyesnocancel(
                '关闭面板',
                '还有步骤在运行或终端窗口没关。\n\n'
                '是：停止所有进程并关闭终端窗口再关闭\n'
                '否：只关闭面板，进程和终端窗口继续保留\n'
                '取消：回到面板',
                parent=self.root)
            if answer is None:
                return
            if answer:
                self.root.withdraw()
                all_pids = []
                for sid in (4, 3, 2, 1):
                    all_pids.extend(find_pids(STEPS[sid]['pattern']))
                if all_pids:
                    terminate(all_pids, lambda message: None, grace=6.0)
                for sid in (4, 3, 2, 1):
                    close_step_windows(sid, lambda message: None)
        self.root.destroy()


# --------------------------------------------------------------------------
# 入口
# --------------------------------------------------------------------------
def run_check():
    print('bit_ws 一键启动 · 环境检查')
    print('  工作区        :', WS, '' if os.path.isdir(WS) else '（不存在）')
    print('  nav777.launch :', NAV777_LAUNCH,
          'OK' if os.path.isfile(NAV777_LAUNCH) else '缺失')
    print('  start_vision  :', VISION_SCRIPT,
          'OK' if os.path.isfile(VISION_SCRIPT) else '缺失')
    print('  路线          :', ', '.join(load_routes()))
    terminal = (shutil.which('gnome-terminal') or shutil.which('x-terminal-emulator')
                or shutil.which('xterm') or '未找到')
    print('  终端          :', terminal)
    print('  ROS master    :', '运行中' if ros_master_up() else '未启动')
    ok = os.path.isfile(NAV777_LAUNCH) and os.path.isfile(VISION_SCRIPT)
    return 0 if ok else 1


def main():
    if '--check' in sys.argv:
        return run_check()

    root = tk.Tk()
    try:
        app = LauncherApp(root)
    except Exception:  # noqa: BLE001 - 界面构造失败时把原因弹出来
        traceback.print_exc()
        messagebox.showerror(APP_TITLE, traceback.format_exc())
        return 1

    if '--self-test' in sys.argv:
        root.withdraw()
        app.show_help()
        root.update_idletasks()
        root.update()
        windows = find_step_windows(list(WINDOW_PREFIX.values()))
        print('self-test ok: 主界面、提示窗口、状态刷新、命令行生成均未报错；'
              '默认路线 %s，当前匹配终端窗口 %d 个'
              % (app.route.get(), len(windows)))
        root.destroy()
        return 0

    root.mainloop()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception:  # noqa: BLE001 - 双击运行时终端不可见，尽量弹窗报错
        traceback.print_exc()
        try:
            messagebox.showerror(APP_TITLE, traceback.format_exc())
        except Exception:  # noqa: BLE001
            pass
        sys.exit(1)
