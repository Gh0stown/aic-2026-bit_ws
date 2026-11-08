#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡航图形界面：选路线、看地图、开始/暂停/继续/终止/记录现场。

界面：
    顶部  路线选择、停留时间、平移速度、载入计划
    左侧  场地地图 + 点位 + 当前目标高亮 + 实时位姿箭头
    右侧  点位执行列表（序号/名称/到点方式/识别任务）+ 日志
    底部  开始 / 暂停 / 继续 / 终止 / 记录现场

暂停：取消当前 move_base 目标、停止平移、持续发零速度；
继续：从当前步骤重新开始，不跳点；
记录现场：把时间、当前点位、位姿、备注写进日志文件
    nav_demo/waypoints/cruise_debug_YYYYmmdd_HHMMSS.log

用法：
    rosrun nav_demo cruise_debug.py
    rosrun nav_demo cruise_debug.py --route fast
"""

import argparse
import math
import os
import queue
import sys
import threading
import time
from datetime import datetime

import yaml

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from cruise_waypoints import (  # noqa: E402
    DEFAULT_POINTS, DEFAULT_ROUTES, DEFAULT_STRAFE_IDS,
    angle_diff, build_plan, lateral_delta, load_yaml,
)

PKG_DIR = os.path.dirname(THIS_DIR)
DEFAULT_MAP = os.path.join(PKG_DIR, 'map', 'mymap_20261106_1916_clean.yaml')

MAP_W, MAP_H = 578, 484


def load_map(yaml_path):
    meta = load_yaml(yaml_path)
    image = meta.get('image')
    if not image:
        raise IOError('地图 yaml 没有 image：%s' % yaml_path)
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    from PIL import Image
    return Image.open(image).convert('RGB'), meta


class PoseTracker(object):
    """后台用 TF 读 map -> base_footprint。"""

    def __init__(self, map_frame, base_frame):
        import rospy
        import tf
        import tf.transformations as tf_trans
        self._tf = tf
        self._rospy = rospy
        self._tf_trans = tf_trans
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.listener = tf.TransformListener()
        self.pose = None
        self._lock = threading.Lock()
        rospy.Timer(rospy.Duration(0.2), self._tick)

    def _tick(self, _event):
        try:
            trans, rot = self.listener.lookupTransform(
                self.map_frame, self.base_frame, self._rospy.Time(0))
        except Exception:  # noqa: BLE001
            return
        yaw = self._tf_trans.euler_from_quaternion(rot)[2]
        with self._lock:
            self.pose = (trans[0], trans[1], yaw, time.time())

    def get(self):
        with self._lock:
            return self.pose


class CruiseWorker(threading.Thread):
    def __init__(self, plan, dwell, strafe_ids, strafe_speed, map_frame,
                 log_fn, on_status):
        super(CruiseWorker, self).__init__(daemon=True)
        self.plan = plan
        self.dwell = dwell
        self.strafe_ids = strafe_ids
        self.strafe_speed = max(0.02, min(0.10, float(strafe_speed)))
        self.map_frame = map_frame
        self.log_fn = log_fn
        self.on_status = on_status

        self._resume = threading.Event()
        self._resume.set()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._odom = None
        self._odom_lock = threading.Lock()
        self._client = None
        self._pub = None
        self._status = '待开始'
        self._step = ''
        self._current_id = None
        self._current_index = -1

    # ---------- 外部控制 ----------
    def pause(self):
        self._resume.clear()
        self._set_status('已暂停')
        self.log('== 暂停 ==')

    def resume(self):
        self._resume.set()
        self._set_status('运行中')
        self.log('== 继续 ==')

    def stop(self):
        self._stop.set()
        self._resume.set()
        self._cancel_all()
        self._stop_motion()
        self._set_status('已终止')

    def status(self):
        with self._lock:
            return self._step, self._status, self._current_id, self._current_index

    def _set_status(self, text):
        with self._lock:
            self._status = text
        self.on_status()

    def _set_step(self, text, pid=None, index=-1):
        with self._lock:
            self._step = text
            self._current_id = pid
            self._current_index = index
        self.on_status()

    def log(self, text):
        self.log_fn(text)

    # ---------- 运动 ----------
    def _wait_if_paused(self):
        while not self._stop.is_set() and not self._resume.wait(0.05):
            pass
        return not self._stop.is_set()

    def _cancel_all(self):
        if self._client is not None:
            try:
                self._client.cancel_all_goals()
            except Exception:  # noqa: BLE001
                pass

    def _stop_motion(self):
        if self._pub is None:
            return
        try:
            from geometry_msgs.msg import Twist
            for _ in range(3):
                self._pub.publish(Twist())
                time.sleep(0.05)
        except Exception:  # noqa: BLE001
            pass

    def _send_goal(self, x, y, yaw, label):
        import rospy
        from actionlib_msgs.msg import GoalStatus
        from move_base_msgs.msg import MoveBaseGoal
        from tf.transformations import quaternion_from_euler

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        while not self._stop.is_set():
            if not self._resume.is_set():
                self.log('    （暂停中，等待继续）')
                if not self._wait_if_paused():
                    return False
                continue
            self.log('  -> %s (%.3f, %.3f, %.0f°)'
                     % (label, x, y, math.degrees(yaw)))
            self._client.send_goal(goal)
            while not self._stop.is_set():
                if not self._resume.is_set():
                    self.log('    暂停：取消当前目标，停车')
                    self._client.cancel_goal()
                    self._stop_motion()
                    if not self._wait_if_paused():
                        return False
                    break
                if self._client.wait_for_result(rospy.Duration(0.1)):
                    if self._client.get_state() == GoalStatus.SUCCEEDED:
                        return True
                    self.log('    未成功（state=%s）' % self._client.get_state())
                    return False
            else:
                return False
        return False

    @staticmethod
    def _delta_y(start, current):
        dx = current[0] - start[0]
        dy = current[1] - start[1]
        return -math.sin(start[2]) * dx + math.cos(start[2]) * dy

    def _fresh_odom(self, max_age=0.5):
        with self._odom_lock:
            pose = self._odom
        if pose is None or time.time() - pose[3] > max_age:
            return None
        return pose

    def _strafe(self, distance, direction):
        import rospy
        from geometry_msgs.msg import Twist

        distance = abs(float(distance))
        sign = 1.0 if direction == 'left' else -1.0
        start = self._fresh_odom()
        if start is None:
            self.log('    /odom 无数据，无法平移')
            return False

        timeout = distance / self.strafe_speed * 2.0 + 3.0
        started = time.monotonic()
        completed = 0.0
        current = 0.0
        rate = rospy.Rate(20)
        try:
            while not self._stop.is_set():
                if not self._resume.is_set():
                    self._stop_motion()
                    cur = self._fresh_odom()
                    if cur is not None:
                        completed += self._delta_y(start, cur) * sign
                    self.log('    暂停：平移停车（已完成 %.2fm）' % completed)
                    if not self._wait_if_paused():
                        return False
                    start = self._fresh_odom()
                    if start is None:
                        self.log('    /odom 中断')
                        return False
                    continue
                cur = self._fresh_odom()
                if cur is None:
                    self.log('    平移中 /odom 中断')
                    return False
                moved = completed + self._delta_y(start, cur) * sign
                if moved >= distance - 0.005:
                    return True
                if time.monotonic() - started > timeout:
                    self.log('    平移超时')
                    return False
                current = min(self.strafe_speed, current + 0.02)
                twist = Twist()
                twist.linear.y = sign * current
                self._pub.publish(twist)
                rate.sleep()
        finally:
            self._stop_motion()
        return False

    def _dwell(self, seconds):
        end = time.monotonic() + seconds
        while not self._stop.is_set():
            if not self._resume.is_set():
                paused_at = time.monotonic()
                if not self._wait_if_paused():
                    return
                end += time.monotonic() - paused_at
                continue
            if time.monotonic() >= end:
                return
            time.sleep(0.05)

    def run(self):
        import actionlib
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from move_base_msgs.msg import MoveBaseAction

        self._client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)

        self._set_status('等待 move_base')
        self.log('等待 move_base action server ...')
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            self._set_status('move_base 未启动')
            self.log('连不上 move_base，先启动 nav777')
            return

        self._set_status('运行中')
        prev = None
        try:
            for idx, step in enumerate(self.plan):
                if self._stop.is_set():
                    break
                via_strafe = (prev is not None and prev['id'] in self.strafe_ids
                              and step['id'] in self.strafe_ids)
                self._set_step('[%d/%d] %s %s' % (idx + 1, len(self.plan),
                                                  step['id'], step['name']),
                               step['id'], idx)
                self.log('')
                self.log('[%d/%d] %s %s' % (idx + 1, len(self.plan),
                                            step['id'], step['name']))

                if via_strafe:
                    lateral = lateral_delta(prev, step)
                    direction = 'left' if lateral > 0 else 'right'
                    self.log('  平移 %.2f m %s（保持车头 %.0f°）'
                             % (abs(lateral), direction, math.degrees(step['yaw'])))
                    arrived = abs(lateral) < 0.01 or self._strafe(lateral, direction)
                else:
                    arrived = self._send_goal(step['x'], step['y'], step['yaw'],
                                              '点位 %s' % step['id'])

                if not arrived:
                    if self._stop.is_set():
                        break
                    self.log('  到达 %s 失败' % step['id'])
                    prev = step
                    continue

                last_yaw = step['yaw']
                for name, yaw in step['tasks']:
                    if self._stop.is_set():
                        break
                    self.log('  识别：%s，转向 %.0f°' % (name, math.degrees(yaw)))
                    if self._send_goal(step['x'], step['y'], yaw, name):
                        last_yaw = yaw
                        self._dwell(self.dwell)
                    elif not self._stop.is_set():
                        self.log('  转向 %s 失败' % name)

                next_step = self.plan[idx + 1] if idx + 1 < len(self.plan) else None
                next_is_strafe = (next_step is not None
                                  and step['id'] in self.strafe_ids
                                  and next_step['id'] in self.strafe_ids)
                if next_is_strafe and angle_diff(last_yaw, step['yaw']) > 0.05:
                    self.log('  转回行驶朝向 %.0f° 再继续平移'
                             % math.degrees(step['yaw']))
                    self._send_goal(step['x'], step['y'], step['yaw'], '恢复朝向')

                prev = step
            if self._stop.is_set():
                self._cancel_all()
                self._stop_motion()
                self._set_status('已终止')
            else:
                self._set_status('结束')
                self.log('')
                self.log('巡航结束')
        finally:
            self._cancel_all()
            self._stop_motion()

    def _odom_cb(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z +
                                p.orientation.x * p.orientation.y),
                         1.0 - 2.0 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        with self._odom_lock:
            self._odom = (p.position.x, p.position.y, yaw, time.time())


class App(object):
    def __init__(self, root, map_img, map_meta, points, routes, args):
        self.root = root
        self.map_img = map_img
        self.map_meta = map_meta
        self.res = float(map_meta.get('resolution', 0.05))
        self.ox = float(map_meta['origin'][0])
        self.oy = float(map_meta['origin'][1])
        self.points = points
        self.routes = routes
        self.args = args
        self.plan = []
        self.worker = None
        self.log_queue = queue.Queue()
        self.pose_tracker = None
        self.current_id = None

        self.log_path = os.path.join(
            os.path.dirname(DEFAULT_POINTS),
            'cruise_debug_%s.log' % datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.log_file = open(self.log_path, 'a')

        self._setup_fonts()
        self._build_ui()
        self._select_route(args.route)
        self.pose_tracker = PoseTracker(args.map_frame, args.base_frame)
        self._reload_plan()

        self._drain_log()
        self._poll()

    # ---------- 界面 ----------
    def _setup_fonts(self):
        import tkinter.font as tkfont
        families = set(tkfont.families())
        for name in ('Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'Noto Sans CJK JP'):
            if name in families:
                for key in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont', 'TkHeadingFont'):
                    tkfont.nametofont(key).configure(family=name, size=10)
                break

    def _build_ui(self):
        import tkinter as tk
        from tkinter import scrolledtext, ttk

        self.root.title('巡航调试')
        self.root.geometry('1120x720')

        top = ttk.Frame(self.root, padding=6)
        top.pack(fill='x')
        ttk.Label(top, text='路线').pack(side='left')
        self.route_var = tk.StringVar()
        self.route_combo = ttk.Combobox(top, textvariable=self.route_var,
                                        values=list(self.routes.keys()), width=10,
                                        state='readonly')
        self.route_combo.pack(side='left', padx=(4, 12))
        self.route_combo.bind('<<ComboboxSelected>>', lambda e: self._refresh_map())

        ttk.Label(top, text='顺序（可空）').pack(side='left')
        self.order_var = tk.StringVar()
        ttk.Entry(top, textvariable=self.order_var, width=30).pack(side='left', padx=(4, 12))

        ttk.Label(top, text='停留 s').pack(side='left')
        self.dwell_var = tk.StringVar(value=str(self.args.dwell))
        ttk.Spinbox(top, from_=0.5, to=10.0, increment=0.5, width=5,
                    textvariable=self.dwell_var).pack(side='left', padx=(4, 12))

        ttk.Label(top, text='平移 m/s').pack(side='left')
        self.speed_var = tk.StringVar(value=str(self.args.strafe_speed))
        ttk.Spinbox(top, from_=0.02, to=0.10, increment=0.01, width=5,
                    textvariable=self.speed_var).pack(side='left', padx=(4, 12))

        ttk.Button(top, text='载入计划', command=self._reload_plan).pack(side='left')

        body = ttk.Frame(self.root, padding=6)
        body.pack(fill='both', expand=True)

        left = ttk.Frame(body)
        left.pack(side='left', fill='both')
        self.canvas = tk.Canvas(left, width=MAP_W, height=MAP_H,
                                highlightthickness=1, highlightbackground='#666')
        self.canvas.pack()
        self.pose_var = tk.StringVar(value='位姿：等待 map -> base_footprint')
        ttk.Label(left, textvariable=self.pose_var).pack(anchor='w', pady=(4, 0))

        right = ttk.Frame(body)
        right.pack(side='left', fill='both', expand=True, padx=(8, 0))

        cols = ('id', 'name', 'mode', 'tasks')
        self.tree = ttk.Treeview(right, columns=cols, show='headings', height=9)
        for col, text, width, anchor in (
                ('id', 'ID', 40, 'center'),
                ('name', '点位', 180, 'w'),
                ('mode', '到点方式', 70, 'center'),
                ('tasks', '识别朝向', 220, 'w')):
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor=anchor)
        self.tree.pack(fill='x')

        self.log_box = scrolledtext.ScrolledText(right, height=14, state='disabled')
        self.log_box.pack(fill='both', expand=True, pady=(6, 0))

        bottom = ttk.Frame(self.root, padding=6)
        bottom.pack(fill='x')
        self.start_btn = ttk.Button(bottom, text='开始', command=self.start)
        self.pause_btn = ttk.Button(bottom, text='暂停', command=self.pause, state='disabled')
        self.resume_btn = ttk.Button(bottom, text='继续', command=self.resume, state='disabled')
        self.stop_btn = ttk.Button(bottom, text='终止', command=self.stop, state='disabled')
        snap_btn = ttk.Button(bottom, text='记录现场', command=self.snapshot)
        for i, b in enumerate((self.start_btn, self.pause_btn, self.resume_btn,
                               self.stop_btn, snap_btn)):
            b.grid(row=0, column=i, padx=4, sticky='ew')
            bottom.columnconfigure(i, weight=1)

        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    # ---------- 地图 ----------
    def _select_route(self, route):
        if route and route in self.routes:
            self.route_var.set(route)
        elif self.routes:
            self.route_var.set(sorted(self.routes.keys())[0])

    def _parse_order(self):
        text = self.order_var.get().strip()
        if text:
            try:
                return [int(x) for x in text.replace(' ', '').split(',') if x != '']
            except ValueError:
                self._log('顺序格式不对：%s' % text)
                return []
        route = self.route_var.get()
        if route in self.routes:
            return [int(x) for x in self.routes[route]]
        return []

    def _reload_plan(self):
        if self.worker is not None and self.worker.is_alive():
            self._log('巡航进行中，不能重新载入')
            return
        order = self._parse_order()
        if not order:
            self._log('没有可用的点位顺序')
            return
        self.plan = build_plan(self.points, order)
        self.tree.delete(*self.tree.get_children())
        prev = None
        for idx, step in enumerate(self.plan):
            mode = '平移' if (prev is not None and prev['id'] in self._strafe_ids()
                              and step['id'] in self._strafe_ids()) else 'move_base'
            tasks = ' → '.join('%s %.0f°' % (n, math.degrees(y)) for n, y in step['tasks']) \
                or '—'
            self.tree.insert('', 'end', iid=str(idx),
                             values=(step['id'], step['name'], mode, tasks))
            prev = step
        self.current_id = None
        self._refresh_map()
        self._log('已载入 %d 个点位' % len(self.plan))

    def _strafe_ids(self):
        if self.args.no_strafe:
            return set()
        return {int(x) for x in self.args.strafe_ids.replace(' ', '').split(',') if x != ''}

    def _refresh_map(self):
        from PIL import Image, ImageTk
        # 保持比例
        scale = min(MAP_W / self.map_img.width, MAP_H / self.map_img.height)
        disp = self.map_img.resize((int(self.map_img.width * scale),
                                    int(self.map_img.height * scale)), Image.NEAREST)
        self.scale = scale
        self.canvas.delete('all')
        self.photo = ImageTk.PhotoImage(disp)
        self.canvas.create_image(0, 0, anchor='nw', image=self.photo)

        # 路线
        for a, b in zip(self.plan, self.plan[1:]):
            self.canvas.create_line(*self._w2c(a['x'], a['y']),
                                    *self._w2c(b['x'], b['y']),
                                    fill='#9cc4ea', width=2, tags='route')
        # 点位
        for step in self.plan:
            cx, cy = self._w2c(step['x'], step['y'])
            self.canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                                    fill='#1a9e1a', outline='white', width=2, tags='pt')
            self.canvas.create_text(cx + 7, cy - 9, text=str(step['id']),
                                    font=('TkDefaultFont', 9, 'bold'), tags='pt')
        self._draw_highlight()
        self._draw_pose()

    def _w2c(self, x, y):
        return ((x - self.ox) / self.res) * self.scale, \
               (self.map_img.height - (y - self.oy) / self.res) * self.scale

    def _draw_highlight(self):
        self.canvas.delete('highlight')
        if self.current_id is None:
            return
        step = next((s for s in self.plan if s['id'] == self.current_id), None)
        if step is None:
            return
        cx, cy = self._w2c(step['x'], step['y'])
        self.canvas.create_oval(cx - 13, cy - 13, cx + 13, cy + 13,
                                outline='#e07000', width=3, tags='highlight')

    def _draw_pose(self):
        self.canvas.delete('pose')
        pose = self.pose_tracker.get() if self.pose_tracker else None
        if pose is None:
            return
        cx, cy = self._w2c(pose[0], pose[1])
        a = -pose[2]
        tip = (cx + 16 * math.cos(a), cy + 16 * math.sin(a))
        b1 = (cx - 6 * math.cos(a) - 7 * math.sin(a), cy - 6 * math.sin(a) + 7 * math.cos(a))
        b2 = (cx - 6 * math.cos(a) + 7 * math.sin(a), cy - 6 * math.sin(a) - 7 * math.cos(a))
        self.canvas.create_polygon([tip[0], tip[1], b1[0], b1[1], b2[0], b2[1]],
                                   fill='#e02020', outline='white', width=1, tags='pose')

    # ---------- 日志/状态 ----------
    def _log(self, text):
        self.log_queue.put(text)

    def _drain_log(self):
        while True:
            try:
                text = self.log_queue.get_nowait()
            except queue.Empty:
                break
            line = '[%s] %s' % (datetime.now().strftime('%H:%M:%S'), text)
            self.log_file.write(line + '\n')
            self.log_file.flush()
            self.log_box.configure(state='normal')
            self.log_box.insert('end', line + '\n')
            self.log_box.see('end')
            self.log_box.configure(state='disabled')
        self.root.after(100, self._drain_log)

    def _poll(self):
        pose = self.pose_tracker.get() if self.pose_tracker else None
        if pose is not None:
            self.pose_var.set('位姿：x=%+.3f  y=%+.3f  yaw=%+.1f°'
                              % (pose[0], pose[1], math.degrees(pose[2])))
        if self.worker is not None and self.worker.is_alive():
            step, status, pid, idx = self.worker.status()
            if pid != self.current_id:
                self.current_id = pid
                self._draw_highlight()
            if idx is not None and idx >= 0 and self.tree.exists(str(idx)):
                self.tree.selection_set(str(idx))
                self.tree.see(str(idx))
        self._draw_pose()
        self.root.after(200, self._poll)

    # ---------- 按钮 ----------
    def start(self):
        if self.worker is not None and self.worker.is_alive():
            return
        if not self.plan:
            self._log('先载入计划')
            return
        dwell = float(self.dwell_var.get())
        speed = float(self.speed_var.get())
        self.log_file.write('\n===== 开始巡航 %s =====\n'
                            % datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        self.worker = CruiseWorker(self.plan, dwell, self._strafe_ids(), speed,
                                   self.args.map_frame, self._log, lambda: None)
        self.worker.start()
        self.start_btn.configure(state='disabled')
        self.pause_btn.configure(state='normal')
        self.stop_btn.configure(state='normal')
        self.resume_btn.configure(state='disabled')
        self._log('开始巡航：%d 个点，%.1fs 停留，%.2f m/s 平移'
                  % (len(self.plan), dwell, speed))

    def pause(self):
        if self.worker is not None:
            self.worker.pause()
            self.pause_btn.configure(state='disabled')
            self.resume_btn.configure(state='normal')

    def resume(self):
        if self.worker is not None:
            self.worker.resume()
            self.pause_btn.configure(state='normal')
            self.resume_btn.configure(state='disabled')

    def stop(self):
        if self.worker is not None:
            self.worker.stop()
            self.pause_btn.configure(state='disabled')
            self.resume_btn.configure(state='disabled')
            self.stop_btn.configure(state='disabled')

    def snapshot(self):
        from tkinter import simpledialog
        step, status, pid, _idx = (self.worker.status() if self.worker is not None
                                   else ('', '待开始', None, -1))
        pose = self.pose_tracker.get() if self.pose_tracker else None
        note = simpledialog.askstring('记录现场', '备注（可留空）：', parent=self.root) or ''
        if pose is None:
            pose_text = '无位姿'
        else:
            pose_text = 'x=%.3f y=%.3f yaw=%.1f°' % (pose[0], pose[1], math.degrees(pose[2]))
        self._log('==== 现场记录 ==== 步骤=%s 状态=%s 位姿=%s 备注=%s'
                  % (step or pid, status, pose_text, note))

    def on_close(self):
        if self.worker is not None:
            self.worker.stop()
        try:
            import rospy
            rospy.signal_shutdown('gui closed')
        except Exception:  # noqa: BLE001
            pass
        self.log_file.close()
        self.root.destroy()


def main():
    import rospy
    import tkinter as tk

    ap = argparse.ArgumentParser(description='巡航图形界面（可暂停/继续/记录现场）')
    ap.add_argument('--points', default=DEFAULT_POINTS)
    ap.add_argument('--routes', default=DEFAULT_ROUTES)
    ap.add_argument('--route', default=None)
    ap.add_argument('--dwell', type=float, default=2.0)
    ap.add_argument('--strafe-ids', default=DEFAULT_STRAFE_IDS)
    ap.add_argument('--strafe-speed', type=float, default=0.06)
    ap.add_argument('--no-strafe', action='store_true')
    ap.add_argument('--map', default=DEFAULT_MAP)
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--withdraw', action='store_true', help=argparse.SUPPRESS)
    args, _ = ap.parse_known_args()

    rospy.init_node('cruise_debug', disable_signals=True)
    threading.Thread(target=rospy.spin, daemon=True).start()

    data = load_yaml(args.points)
    points = {int(p['id']): p for p in data.get('points', [])}
    routes = load_yaml(args.routes).get('routes', {}) if os.path.isfile(args.routes) else {}
    map_img, map_meta = load_map(args.map)

    root = tk.Tk()
    app = App(root, map_img, map_meta, points, routes, args)
    if args.withdraw:
        root.withdraw()
    root.mainloop()


if __name__ == '__main__':
    main()
