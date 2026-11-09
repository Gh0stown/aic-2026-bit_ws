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
    rosrun nav_demo cruise_debug.py --route left
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
    angle_diff, build_plan, is_strafe_leg, lateral_delta, load_yaml,
)

PKG_DIR = os.path.dirname(THIS_DIR)
DEFAULT_MAP = os.path.join(PKG_DIR, 'map', 'mymap_20261106_1916_clean.yaml')

MAP_W, MAP_H = 578, 484
DRIVE_SPEED_LONG = 0.60   # 长直线（>2m）
DRIVE_SPEED_MID = 0.60    # 中段（1~2m）
DRIVE_SPEED_SHORT = 0.60  # 短段（<1m）


class CruiseTimer(object):
    """统计一轮的墙钟经过时间和人工暂停时间，不参与运动超时控制。"""

    def __init__(self, clock=None):
        self._clock = clock or time.monotonic
        self._lock = threading.Lock()
        self._started = self._clock()
        self._ended = None
        self._pause_started = None
        self._paused = 0.0

    def pause(self):
        with self._lock:
            if self._ended is None and self._pause_started is None:
                self._pause_started = self._clock()

    def resume(self):
        with self._lock:
            if self._ended is None and self._pause_started is not None:
                self._paused += self._clock() - self._pause_started
                self._pause_started = None

    def _snapshot(self):
        now = self._ended if self._ended is not None else self._clock()
        total = max(0.0, now - self._started)
        paused = self._paused
        if self._pause_started is not None:
            paused += now - self._pause_started
        return {'total': total, 'paused': paused,
                'active': max(0.0, total - paused),
                'finished': self._ended is not None}

    def snapshot(self):
        with self._lock:
            return self._snapshot()

    def finish(self):
        with self._lock:
            if self._ended is None:
                self._ended = self._clock()
            return self._snapshot()


def format_duration(seconds):
    # 先按 0.1 秒取整，再分解，避免出现“0 分 60.0 秒”。
    tenths = int(round(max(0.0, seconds) * 10))
    return '%d 分 %04.1f 秒' % (tenths // 600, (tenths % 600) / 10.0)


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


class ScanBuffer(object):
    """缓存最新一帧 /scan。"""

    def __init__(self):
        import rospy
        from sensor_msgs.msg import LaserScan
        self._scan = None
        self._stamp = 0.0
        rospy.Subscriber('/scan', LaserScan, self._cb, queue_size=1)

    def _cb(self, msg):
        self._scan = msg
        self._stamp = time.time()

    def get(self, max_age=1.5):
        if self._scan is None or time.time() - self._stamp > max_age:
            return None
        return self._scan


def _score_grid(xs, ys, yaw, bx, by, dist, res, ox, oy, sigma):
    import numpy as np
    height, width = dist.shape
    c, s = math.cos(yaw), math.sin(yaw)
    dx = bx * c - by * s
    dy = bx * s + by * c
    score = np.zeros((len(ys), len(xs)), dtype=np.float32)
    for k in range(len(dx)):
        px = (xs[None, :] + dx[k] - ox) / res
        py = height - (ys[:, None] + dy[k] - oy) / res
        ix = np.clip(np.round(px).astype(np.int32), 0, width - 1)
        iy = np.clip(np.round(py).astype(np.int32), 0, height - 1)
        score += np.exp(-((dist[iy, ix] / sigma) ** 2))
    return score


def estimate_pose(base_points, dist, res, ox, oy, sigma=0.08):
    """用一帧激光点（base 系）在地图距离场上做粗到细搜索，返回 (x, y, yaw, score)。"""
    import numpy as np
    bx = base_points[:, 0]
    by = base_points[:, 1]
    height, width = dist.shape

    xs = np.arange(ox + 0.3, ox + width * res - 0.3, 0.1)
    ys = np.arange(oy + 0.3, oy + height * res - 0.3, 0.1)
    gx = np.clip(np.round((xs - ox) / res).astype(int), 0, width - 1)
    gy = np.clip(np.round(height - (ys - oy) / res).astype(int), 0, height - 1)
    free = dist[gy[:, None], gx[None, :]] > 0.15

    best = None
    for yaw in np.deg2rad(np.arange(0.0, 360.0, 10.0)):
        score = _score_grid(xs, ys, yaw, bx, by, dist, res, ox, oy, sigma)
        score[~free] = -1.0
        idx = np.unravel_index(np.argmax(score), score.shape)
        val = float(score[idx])
        if best is None or val > best[3]:
            best = (float(xs[idx[1]]), float(ys[idx[0]]), float(yaw), val)

    x0, y0, yaw0, _ = best
    xs = np.arange(x0 - 0.1, x0 + 0.101, 0.02)
    ys = np.arange(y0 - 0.1, y0 + 0.101, 0.02)
    best2 = best
    yaw_center = math.degrees(yaw0)
    for yaw in np.deg2rad(np.arange(yaw_center - 6.0, yaw_center + 6.1, 2.0)):
        score = _score_grid(xs, ys, yaw, bx, by, dist, res, ox, oy, sigma)
        idx = np.unravel_index(np.argmax(score), score.shape)
        val = float(score[idx])
        if val > best2[3]:
            best2 = (float(xs[idx[1]]), float(ys[idx[0]]), float(yaw), val)
    return best2


class CruiseWorker(threading.Thread):
    def __init__(self, plan, dwell, strafe_ids, strafe_speed, map_frame,
                 log_fn, on_status, pose_get=None, base_frame='base_footprint',
                 run_timer=None, drive_long=DRIVE_SPEED_LONG,
                 drive_mid=DRIVE_SPEED_MID, drive_short=DRIVE_SPEED_SHORT,
                 rot_speed=1.00, reverse_speed=0.30):
        super(CruiseWorker, self).__init__(daemon=True)
        self.plan = plan
        self.dwell = dwell
        self.strafe_ids = strafe_ids
        self.strafe_speed = max(0.02, min(0.15, float(strafe_speed)))
        self.drive_long = max(0.10, min(0.60, float(drive_long)))
        self.drive_mid = max(0.10, min(0.60, float(drive_mid)))
        self.drive_short = max(0.10, min(0.60, float(drive_short)))
        self.rot_speed = max(0.20, min(1.00, float(rot_speed)))
        self.reverse_speed = max(0.05, min(0.30, float(reverse_speed)))
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.log_fn = log_fn
        self.on_status = on_status
        self._pose_get = pose_get
        self.run_timer = run_timer if run_timer is not None else CruiseTimer()
        self._timing_lock = threading.Lock()
        self._timing_reported = False

        self._resume = threading.Event()
        self._resume.set()
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._odom = None
        self._odom_lock = threading.Lock()
        self._client = None
        self._active_client = None
        self._pub = None
        self._tf_listener = None
        self._status = '待开始'
        self._step = ''
        self._current_id = None
        self._current_index = -1

    # ---------- 外部控制 ----------
    def pause(self):
        self.run_timer.pause()
        self._resume.clear()
        self._set_status('已暂停')
        self.log('== 暂停 ==')

    def resume(self):
        self.run_timer.resume()
        self._resume.set()
        self._set_status('运行中')
        self.log('== 继续 ==')

    def stop(self):
        self._stop_evt.set()
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
        while not self._stop_evt.is_set() and not self._resume.wait(0.05):
            pass
        return not self._stop_evt.is_set()

    def _cancel_all(self):
        for client in (self._client, self._active_client):
            if client is not None:
                try:
                    client.cancel_all_goals()
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

    def _pose_far(self, x, y, tol=0.6):
        """move_base 报成功时，用 TF 位姿再核对一次到点距离。"""
        if self._pose_get is None:
            return False
        pose = self._pose_get()
        if pose is None:
            return False
        return math.hypot(pose[0] - x, pose[1] - y) > tol

    def _send_goal(self, x, y, yaw, label):
        import actionlib
        import rospy
        from actionlib_msgs.msg import GoalStatus
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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

        far_count = 0
        while not self._stop_evt.is_set():
            if not self._resume.is_set():
                self.log('    （暂停中，等待继续）')
                if not self._wait_if_paused():
                    return False
                continue
            # 每个目标用新的 action client，避免暂停/取消后拿到上一次目标的残留状态
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            if not client.wait_for_server(rospy.Duration(2.0)):
                self.log('    等待 move_base 超时')
                return False
            self.log('  -> %s (%.3f, %.3f, %.0f°)'
                     % (label, x, y, math.degrees(yaw)))
            client.send_goal(goal)
            self._active_client = client

            action = None
            try:
                while not self._stop_evt.is_set() and action is None:
                    if not self._resume.is_set():
                        client.cancel_goal()
                        self._stop_motion()
                        t0 = time.monotonic()
                        while (client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE)
                               and time.monotonic() - t0 < 2.0):
                            time.sleep(0.05)
                        self.log('    暂停：目标已取消，停车')
                        if not self._wait_if_paused():
                            return False
                        action = 'pause'
                    elif client.wait_for_result(rospy.Duration(0.1)):
                        state = client.get_state()
                        if state != GoalStatus.SUCCEEDED:
                            self.log('    未成功（state=%s）' % state)
                            return False
                        if self._pose_far(x, y):
                            far_count += 1
                            self.log('    目标报成功但离点位较远（第 %d 次），重发' % far_count)
                            if far_count >= 2:
                                return False
                            action = 'retry'
                        else:
                            return True
            finally:
                self._active_client = None
            if self._stop_evt.is_set():
                client.cancel_goal()
                self._stop_motion()
                return False
            # 'pause' 或 'retry' 都回到外层重新发
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
        paused = 0.0
        completed = 0.0
        current = 0.0
        rate = rospy.Rate(20)
        try:
            while not self._stop_evt.is_set():
                if not self._resume.is_set():
                    t_pause = time.monotonic()
                    self._stop_motion()
                    cur = self._fresh_odom()
                    if cur is not None:
                        completed += self._delta_y(start, cur) * sign
                    self.log('    暂停：平移停车（已完成 %.2fm）' % completed)
                    if not self._wait_if_paused():
                        return False
                    paused += time.monotonic() - t_pause
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
                if time.monotonic() - started - paused > timeout:
                    self.log('    平移超时')
                    return False
                current = min(self.strafe_speed, current + 0.02)
                twist = Twist()
                twist.linear.y = sign * current
                err_yaw = self._norm_angle(start[2] - cur[2])
                twist.angular.z = max(-0.15, min(0.15, 1.2 * err_yaw))
                self._pub.publish(twist)
                rate.sleep()
        finally:
            self._stop_motion()
        return False

    @staticmethod
    def _norm_angle(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _map_pose(self):
        import rospy
        import tf.transformations as tft
        if self._tf_listener is None:
            return None
        try:
            trans, rot = self._tf_listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0))
        except Exception:  # noqa: BLE001
            return None
        return (trans[0], trans[1], tft.euler_from_quaternion(rot)[2])

    def _bearing_to(self, x, y):
        pose = self._map_pose()
        if pose is None:
            return None
        return math.atan2(y - pose[1], x - pose[0])

    def _rotate_to(self, target, label):
        """只用 /cmd_vel 的 angular.z 闭环转到目标朝向（map 系）。"""
        import rospy
        from geometry_msgs.msg import Twist

        target = self._norm_angle(target)
        pose = self._map_pose()
        if pose is None:
            self.log('    没有 map->%s 位姿，无法转向 %s' % (self.base_frame, label))
            return False
        timeout = 8.0 + abs(self._norm_angle(target - pose[2])) / 0.2
        started = time.monotonic()
        paused = 0.0
        best_err = abs(self._norm_angle(target - pose[2]))
        last_progress = time.monotonic()
        rate = rospy.Rate(20)
        try:
            while not self._stop_evt.is_set():
                if not self._resume.is_set():
                    t_pause = time.monotonic()
                    self._stop_motion()
                    if not self._wait_if_paused():
                        return False
                    paused += time.monotonic() - t_pause
                    last_progress = time.monotonic()
                    continue
                pose = self._map_pose()
                if pose is None:
                    self.log('    转向中位姿丢失')
                    return False
                err = self._norm_angle(target - pose[2])
                if abs(err) <= 0.05:
                    return True
                if abs(err) < best_err - 0.02:
                    best_err = abs(err)
                    last_progress = time.monotonic()
                elif abs(err) > 0.15 and time.monotonic() - last_progress > 2.0:
                    self.log('    旋转受阻（误差 %.1f°，2 秒没有改善）'
                             % math.degrees(abs(err)))
                    return False
                if time.monotonic() - started - paused > timeout:
                    self.log('    转向超时（目标 %.0f°）' % math.degrees(target))
                    return False
                # 比例控制，接近目标自动减速，避免一侧轮子突然发力
                cmd = min(self.rot_speed, max(0.20, 1.5 * abs(err)))
                twist = Twist()
                twist.angular.z = (1.0 if err > 0 else -1.0) * cmd
                self._pub.publish(twist)
                rate.sleep()
            return False
        finally:
            self._stop_motion()

    def _align_to(self, x, y, tol=0.02, speed=0.06, timeout=6.0, hold_yaw=None):
        """用麦轮 linear.x/y 闭环走到目标，可同时保持车头（倒车入库用）。"""
        import rospy
        from geometry_msgs.msg import Twist

        started = time.monotonic()
        rate = rospy.Rate(20)
        try:
            while not self._stop_evt.is_set():
                if not self._resume.is_set():
                    self._stop_motion()
                    if not self._wait_if_paused():
                        return False
                    continue
                pose = self._map_pose()
                if pose is None:
                    return False
                ex = x - pose[0]
                ey = y - pose[1]
                dist = math.hypot(ex, ey)
                if dist <= tol:
                    return True
                if time.monotonic() - started > timeout:
                    self.log('    最终对准超时（剩余 %.2f m）' % dist)
                    return False
                yaw = pose[2]
                vx = math.cos(yaw) * ex + math.sin(yaw) * ey
                vy = -math.sin(yaw) * ex + math.cos(yaw) * ey
                twist = Twist()
                twist.linear.x = max(-speed, min(speed, 1.5 * vx))
                twist.linear.y = max(-speed, min(speed, 1.5 * vy))
                if hold_yaw is not None:
                    err_yaw = self._norm_angle(hold_yaw - pose[2])
                    twist.angular.z = max(-0.2, min(0.2, 1.0 * err_yaw))
                self._pub.publish(twist)
                rate.sleep()
            return False
        finally:
            self._stop_motion()

    def _dwell(self, seconds):
        end = time.monotonic() + seconds
        while not self._stop_evt.is_set():
            if not self._resume.is_set():
                paused_at = time.monotonic()
                if not self._wait_if_paused():
                    return
                end += time.monotonic() - paused_at
                continue
            if time.monotonic() >= end:
                return
            time.sleep(0.05)

    def finish_timing(self):
        """每轮只记录一次；窗口关闭时也可调用，确保计时落盘。"""
        with self._timing_lock:
            if self._timing_reported:
                return
            timing = self.run_timer.finish()
            self.log('计时汇总：总耗时 %s（含暂停），暂停 %s，执行耗时 %s（不含暂停）'
                     % (format_duration(timing['total']),
                        format_duration(timing['paused']),
                        format_duration(timing['active'])))
            self._timing_reported = True

    def run(self):
        try:
            self._run_plan()
        finally:
            # 正常结束、终止、启动失败或异常退出都停表；不改变运动处理。
            self.finish_timing()

    def _run_plan(self):
        import actionlib
        import rospy
        import tf
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from move_base_msgs.msg import MoveBaseAction

        self._client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)
        self._tf_listener = tf.TransformListener()

        self._set_status('等待 move_base')
        self.log('等待 move_base action server ...')
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            self._set_status('move_base 未启动')
            self.log('连不上 move_base，先启动 nav777')
            return

        if self._map_pose() is None:
            self.log('等待 map -> %s 的 TF ...' % self.base_frame)
            t0 = time.monotonic()
            while (self._map_pose() is None and time.monotonic() - t0 < 5.0
                   and not self._stop_evt.is_set()):
                time.sleep(0.1)
            if self._map_pose() is None:
                self._set_status('没有 map->base TF')
                self.log('没有 map -> %s 的 TF，转向无法闭环；先确认 AMCL/定位'
                         % self.base_frame)
                return

        self._set_status('运行中')
        prev = None
        try:
            for idx, step in enumerate(self.plan):
                if self._stop_evt.is_set():
                    break
                via_strafe = is_strafe_leg(prev, step, self.strafe_ids)
                next_step = self.plan[idx + 1] if idx + 1 < len(self.plan) else None
                next_is_strafe = is_strafe_leg(step, next_step, self.strafe_ids)
                self._set_step('[%d/%d] %s %s' % (idx + 1, len(self.plan),
                                                  step['id'], step['name']),
                               step['id'], idx)
                self.log('')
                self.log('[%d/%d] %s %s' % (idx + 1, len(self.plan),
                                            step['id'], step['name']))

                final_park = (idx == len(self.plan) - 1 and step['id'] == 19)
                if final_park:
                    if prev is not None:
                        self.log('  倒车前先对齐 %s (%.3f, %.3f)'
                                 % (prev['id'], prev['x'], prev['y']))
                        self._align_to(prev['x'], prev['y'], tol=0.03, speed=0.12,
                                       timeout=10.0, hold_yaw=prev['yaw'])
                        if self._stop_evt.is_set():
                            break
                    self.log('  倒车入库：先转到 0°（朝红绿灯），再倒进 19')
                    self._rotate_to(0.0, '倒车入库朝向')
                    if self._stop_evt.is_set():
                        break
                    self.log('  倒车到 19 (%.3f, %.3f)' % (step['x'], step['y']))
                    arrived = self._align_to(step['x'], step['y'], tol=0.02,
                                             speed=self.reverse_speed,
                                             timeout=15.0, hold_yaw=0.0)
                elif via_strafe:
                    self._rotate_to(step['yaw'], '平移前朝向')
                    if self._stop_evt.is_set():
                        break
                    dist = math.hypot(step['x'] - prev['x'], step['y'] - prev['y'])
                    self.log('  闭环平移至 %s（%.2f m，保持车头 %.0f°）'
                             % (step['id'], dist, math.degrees(step['yaw'])))
                    arrived = self._align_to(
                        step['x'], step['y'], tol=0.02,
                        speed=self.strafe_speed,
                        timeout=dist / self.strafe_speed * 2.0 + 5.0,
                        hold_yaw=step['yaw'])
                else:
                    pose = self._map_pose()
                    bearing = self._bearing_to(step['x'], step['y'])
                    if bearing is None:
                        bearing = step['yaw']
                        self.log('  没有位姿，按记录朝向行驶')
                    dist_to_goal = (math.hypot(step['x'] - pose[0], step['y'] - pose[1])
                                    if pose is not None else None)
                    close = dist_to_goal is not None and dist_to_goal < 0.25
                    goal_yaw = step['yaw'] if close else bearing
                    if close:
                        self.log('  距点位 %.2f m，跳过预转向，按记录朝向过去' % dist_to_goal)
                    else:
                        self.log('  预转向到方位 %.0f°' % math.degrees(bearing))
                        self._rotate_to(bearing, '对准 %s' % step['id'])
                        if self._stop_evt.is_set():
                            break
                    travel_dist = dist_to_goal if dist_to_goal is not None else 1.0
                    if travel_dist > 2.0:
                        travel_speed = self.drive_long
                    elif travel_dist > 1.0:
                        travel_speed = self.drive_mid
                    else:
                        travel_speed = self.drive_short
                    self.log('  闭环直线 %.2f m（%.2f m/s）' % (travel_dist, travel_speed))
                    arrived = self._align_to(
                        step['x'], step['y'], tol=0.02,
                        speed=travel_speed,
                        timeout=travel_dist / travel_speed * 2.0 + 5.0,
                        hold_yaw=goal_yaw)
                    # 有识别任务时不再先转“到达朝向”，直接让第一个识别朝向去转，
                    # 少一次来回；只有无任务且下一段是平移时才需要先摆正
                    if arrived and not step['tasks'] and next_is_strafe:
                        if abs(self._norm_angle(step['yaw'] - goal_yaw)) > 0.05:
                            self.log('  到点转到 %.0f°' % math.degrees(step['yaw']))
                            self._rotate_to(step['yaw'], '到达朝向')
                            if self._stop_evt.is_set():
                                break

                if not arrived:
                    if self._stop_evt.is_set():
                        break
                    self.log('  到达 %s 失败' % step['id'])
                    prev = step
                    continue

                last_yaw = step['yaw']
                for name, yaw in step['tasks']:
                    if self._stop_evt.is_set():
                        break
                    self.log('  识别：%s，转向 %.0f°' % (name, math.degrees(yaw)))
                    if self._rotate_to(yaw, name):
                        last_yaw = yaw
                        self._dwell(self.dwell)
                    elif not self._stop_evt.is_set():
                        self.log('  转向 %s 失败' % name)

                if next_is_strafe and angle_diff(last_yaw, step['yaw']) > 0.05:
                    self.log('  转回行驶朝向 %.0f° 再继续平移'
                             % math.degrees(step['yaw']))
                    self._rotate_to(step['yaw'], '恢复朝向')

                if idx == len(self.plan) - 1 and not final_park:
                    self.log('  最终对准 %s (%.3f, %.3f)' % (step['id'], step['x'], step['y']))
                    self._align_to(step['x'], step['y'])

                prev = step
            if self._stop_evt.is_set():
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
        self.dist = self._build_dist()
        self.scan_buffer = ScanBuffer()
        self.calibrating = False
        import rospy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                                           queue_size=1)
        from geometry_msgs.msg import Twist
        self._twist_cls = Twist
        self.teleop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.teleop_keys = {}
        self._teleop_active = False
        self.worker = None
        self.log_queue = queue.Queue()
        self.pose_tracker = None
        self.current_id = None
        self.run_timer = None

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
        self._teleop_tick()

    def _build_dist(self):
        import numpy as np
        from scipy import ndimage
        gray = np.array(self.map_img.convert('L'))
        return ndimage.distance_transform_edt(gray >= 100) * self.res

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
        ttk.Spinbox(top, from_=0.02, to=0.15, increment=0.01, width=5,
                    textvariable=self.speed_var).pack(side='left', padx=(4, 12))

        self.skip0_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text='跳过0号点', variable=self.skip0_var,
                        command=self._reload_plan).pack(side='left', padx=(0, 10))
        ttk.Button(top, text='载入计划', command=self._reload_plan).pack(side='left')

        speed_frame = ttk.Frame(self.root, padding=(6, 0, 6, 4))
        speed_frame.pack(fill='x')
        ttk.Label(speed_frame, text='直线速度 长').pack(side='left')
        self.long_speed_var = tk.StringVar(value='%.2f' % DRIVE_SPEED_LONG)
        ttk.Spinbox(speed_frame, from_=0.10, to=0.60, increment=0.01, width=5,
                    textvariable=self.long_speed_var).pack(side='left', padx=(2, 8))
        ttk.Label(speed_frame, text='中').pack(side='left')
        self.mid_speed_var = tk.StringVar(value='%.2f' % DRIVE_SPEED_MID)
        ttk.Spinbox(speed_frame, from_=0.10, to=0.60, increment=0.01, width=5,
                    textvariable=self.mid_speed_var).pack(side='left', padx=(2, 8))
        ttk.Label(speed_frame, text='短').pack(side='left')
        self.short_speed_var = tk.StringVar(value='%.2f' % DRIVE_SPEED_SHORT)
        ttk.Spinbox(speed_frame, from_=0.10, to=0.60, increment=0.01, width=5,
                    textvariable=self.short_speed_var).pack(side='left', padx=(2, 12))
        ttk.Label(speed_frame, text='转向 rad/s').pack(side='left')
        self.rot_speed_var = tk.StringVar(value='1.00')
        ttk.Spinbox(speed_frame, from_=0.20, to=1.00, increment=0.05, width=5,
                    textvariable=self.rot_speed_var).pack(side='left', padx=(2, 12))
        ttk.Label(speed_frame, text='倒车 m/s').pack(side='left')
        self.reverse_speed_var = tk.StringVar(value='0.30')
        ttk.Spinbox(speed_frame, from_=0.05, to=0.30, increment=0.01, width=5,
                    textvariable=self.reverse_speed_var).pack(side='left', padx=(2, 12))
        ttk.Label(speed_frame, text='（点击“开始”时生效）').pack(side='left')

        body = ttk.Frame(self.root, padding=6)
        body.pack(fill='both', expand=True)

        left = ttk.Frame(body)
        left.pack(side='left', fill='both')
        self.canvas = tk.Canvas(left, width=MAP_W, height=MAP_H,
                                highlightthickness=1, highlightbackground='#666')
        self.canvas.pack()
        self.pose_var = tk.StringVar(value='位姿：等待 map -> base_footprint')
        ttk.Label(left, textvariable=self.pose_var).pack(anchor='w', pady=(4, 0))
        ttk.Label(left, text='暂停后键盘：W/S 前后  A/D 平移  Q/E 旋转  空格停',
                  foreground='#666').pack(anchor='w')

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
        rec_btn = ttk.Button(bottom, text='记录点位', command=self.record_point)
        self.cal_btn = ttk.Button(bottom, text='扫描校准', command=self.calibrate)
        for i, b in enumerate((self.start_btn, self.pause_btn, self.resume_btn,
                               self.stop_btn, snap_btn, rec_btn, self.cal_btn)):
            b.grid(row=0, column=i, padx=4, sticky='ew')
            bottom.columnconfigure(i, weight=1)

        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.root.bind_all('<KeyPress>', self._on_key)

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
                order = [x for x in text.replace(' ', '').split(',') if x != '']
            except ValueError:
                self._log('顺序格式不对：%s' % text)
                return []
            if self.skip0_var.get() and order and str(order[0]).rstrip('pP') == '0':
                order = order[1:]
            return order
        route = self.route_var.get()
        if route in self.routes:
            order = list(self.routes[route])
            if self.skip0_var.get() and order and str(order[0]).rstrip('pP') == '0':
                order = order[1:]
            return order
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
            if idx == len(self.plan) - 1 and step['id'] == 19:
                mode = '倒车入库'
            elif is_strafe_leg(prev, step, self._strafe_ids()):
                mode = '平移'
            else:
                mode = '闭环直线'
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
        self._flush_log_queue()
        self.root.after(100, self._drain_log)

    def _flush_log_queue(self):
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
        if self.run_timer is not None:
            elapsed = self.run_timer.snapshot()['total']
            self.root.title('巡航调试  用时 %02d:%02d'
                            % (int(elapsed) // 60, int(elapsed) % 60))
        # 一次巡航结束后重新放开“开始”，不用重启 GUI
        if self.worker is not None and not self.worker.is_alive():
            if str(self.start_btn['state']) == 'disabled':
                self.start_btn.configure(state='normal')
            if str(self.stop_btn['state']) == 'normal':
                self.stop_btn.configure(state='disabled')
                self.pause_btn.configure(state='disabled')
                self.resume_btn.configure(state='disabled')
        if not self.calibrating and str(self.cal_btn['state']) == 'disabled':
            self.cal_btn.configure(state='normal')
        self.root.after(200, self._poll)

    # ---------- 按钮 ----------
    def start(self):
        if self.worker is not None and self.worker.is_alive():
            return
        if not self.plan:
            self._log('先载入计划')
            return
        self.current_id = None
        self._draw_highlight()
        self.teleop_keys.clear()
        self._teleop_zero()
        self._teleop_active = False
        dwell = float(self.dwell_var.get())
        speed = max(0.02, min(0.15, float(self.speed_var.get())))
        self.speed_var.set('%.2f' % speed)
        long_speed = max(0.10, min(0.60, float(self.long_speed_var.get())))
        mid_speed = max(0.10, min(0.60, float(self.mid_speed_var.get())))
        short_speed = max(0.10, min(0.60, float(self.short_speed_var.get())))
        rot_speed = max(0.20, min(1.00, float(self.rot_speed_var.get())))
        reverse_speed = max(0.05, min(0.30, float(self.reverse_speed_var.get())))
        for var, val in ((self.long_speed_var, long_speed), (self.mid_speed_var, mid_speed),
                         (self.short_speed_var, short_speed), (self.rot_speed_var, rot_speed),
                         (self.reverse_speed_var, reverse_speed)):
            var.set('%.2f' % val)
        self.run_timer = CruiseTimer()
        self.log_file.write('\n===== 开始巡航 %s =====\n'
                            % datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        self.worker = CruiseWorker(self.plan, dwell, self._strafe_ids(), speed,
                                   self.args.map_frame, self._log, lambda: None,
                                   pose_get=self.pose_tracker.get,
                                   base_frame=self.args.base_frame,
                                   run_timer=self.run_timer,
                                   drive_long=long_speed, drive_mid=mid_speed,
                                   drive_short=short_speed, rot_speed=rot_speed,
                                   reverse_speed=reverse_speed)
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
            self.teleop_keys.clear()
            self._teleop_zero()
            self._teleop_active = False
            self.pause_btn.configure(state='disabled')
            self.resume_btn.configure(state='normal')

    def resume(self):
        if self.worker is not None:
            self.teleop_keys.clear()
            self._teleop_zero()
            self._teleop_active = False
            self.worker.resume()
            self.pause_btn.configure(state='normal')
            self.resume_btn.configure(state='disabled')

    def stop(self):
        if self.worker is not None:
            self.teleop_keys.clear()
            self._teleop_zero()
            self._teleop_active = False
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

    def record_point(self):
        """记录当前位姿和备注，写到当前巡航日志里，附最近点位和 YAML 参考。"""
        from tkinter import simpledialog
        pose = self.pose_tracker.get() if self.pose_tracker else None
        if pose is None:
            self._log('点位记录：还没有位姿数据')
            return
        nearest_id, nearest_name, nearest_dist = None, '', 1e9
        for pid, p in self.points.items():
            d = math.hypot(pose[0] - float(p['x']), pose[1] - float(p['y']))
            if d < nearest_dist:
                nearest_id, nearest_name, nearest_dist = pid, p.get('name', ''), d
        note = simpledialog.askstring('记录点位', '备注（可留空）：', parent=self.root) or ''
        self._log('点位记录：x=%.3f y=%.3f yaw=%.1f° 最近=%s %s（%.2f m） 备注=%s'
                  % (pose[0], pose[1], math.degrees(pose[2]),
                     nearest_id, nearest_name, nearest_dist, note))
        if nearest_dist < 0.5:
            self._log('  YAML 参考：id=%s x=%.3f y=%.3f yaw_deg=%.1f'
                      % (nearest_id, pose[0], pose[1], math.degrees(pose[2])))

    def _teleop_enabled(self):
        # 只在巡航暂停时开放键盘，避免未开始时误碰底盘
        if self.worker is None or not self.worker.is_alive():
            return False
        _step, status, _pid, _idx = self.worker.status()
        return status == '已暂停'

    def _teleop_zero(self):
        try:
            self.teleop_pub.publish(self._twist_cls())
        except Exception:  # noqa: BLE001
            pass

    def _on_key(self, event):
        import tkinter as tk
        from tkinter import ttk
        if not self._teleop_enabled():
            return
        focus = self.root.focus_get()
        if isinstance(focus, (tk.Entry, tk.Text, ttk.Entry, ttk.Spinbox, ttk.Combobox)):
            return
        key = event.keysym.lower()
        if key in ('w', 'a', 's', 'd', 'q', 'e'):
            self.teleop_keys[key] = time.monotonic()
        elif key == 'space':
            self.teleop_keys.clear()
            self._teleop_zero()
            self._teleop_active = False

    def _teleop_tick(self):
        try:
            if self._teleop_enabled():
                now = time.monotonic()
                active = {k: t for k, t in self.teleop_keys.items() if now - t < 0.3}
                self.teleop_keys = active
                if active:
                    twist = self._twist_cls()
                    v = 0.12
                    if 'w' in active:
                        twist.linear.x += v
                    if 's' in active:
                        twist.linear.x -= v
                    if 'a' in active:
                        twist.linear.y += v
                    if 'd' in active:
                        twist.linear.y -= v
                    if 'q' in active:
                        twist.angular.z += 0.5
                    if 'e' in active:
                        twist.angular.z -= 0.5
                    self.teleop_pub.publish(twist)
                    self._teleop_active = True
                elif self._teleop_active:
                    self._teleop_zero()
                    self._teleop_active = False
            else:
                self.teleop_keys.clear()
                if self._teleop_active:
                    self._teleop_zero()
                    self._teleop_active = False
        finally:
            self.root.after(50, self._teleop_tick)

    def calibrate(self):
        if self.calibrating:
            return
        if self.worker is not None and self.worker.is_alive():
            self._log('巡航进行中，先暂停或终止再校准')
            return
        self.calibrating = True
        self.cal_btn.configure(state='disabled')
        threading.Thread(target=self._do_calibrate, daemon=True).start()

    def _do_calibrate(self):
        import numpy as np
        import rospy
        import tf.transformations as tf_trans
        from geometry_msgs.msg import PoseWithCovarianceStamped

        try:
            scan = self.scan_buffer.get()
            if scan is None:
                self._log('校准：没有 /scan 数据')
                return
            ranges = np.array(scan.ranges, dtype=float)
            angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
            valid = (np.isfinite(ranges) & (ranges > scan.range_min)
                     & (ranges < scan.range_max))
            idx = np.where(valid)[0]
            if len(idx) < 20:
                self._log('校准：有效激光点太少')
                return
            idx = idx[::max(1, len(idx) // 72)]
            r = ranges[idx]
            a = angles[idx]

            tx, ty, ly = 0.0, 0.0, 0.0
            try:
                trans, rot = self.pose_tracker.listener.lookupTransform(
                    'base_footprint', 'laser_link', rospy.Time(0))
                tx, ty = trans[0], trans[1]
                ly = tf_trans.euler_from_quaternion(rot)[2]
            except Exception:  # noqa: BLE001
                self._log('校准：没有 base->laser TF，按无旋转处理')

            lx = r * np.cos(a)
            ly_ = r * np.sin(a)
            bx = lx * math.cos(ly) - ly_ * math.sin(ly) + tx
            by = lx * math.sin(ly) + ly_ * math.cos(ly) + ty
            points = np.column_stack([bx, by])

            self._log('校准：正在用 %d 个激光点匹配地图 ...' % len(points))
            x, y, yaw, score = estimate_pose(points, self.dist, self.res, self.ox, self.oy)
            cur = self.pose_tracker.get()
            if cur is not None:
                self._log('校准：当前 TF (%.3f, %.3f, %.1f°)，匹配结果 (%.3f, %.3f, %.1f°)，得分 %.1f'
                          % (cur[0], cur[1], math.degrees(cur[2]),
                             x, y, math.degrees(yaw), score))
            else:
                self._log('校准：匹配结果 (%.3f, %.3f, %.1f°)，得分 %.1f'
                          % (x, y, math.degrees(yaw), score))

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.args.map_frame
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            q = tf_trans.quaternion_from_euler(0.0, 0.0, yaw)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.2 ** 2
            cov[35] = 0.1 ** 2
            msg.pose.covariance = cov
            for _ in range(3):
                self.initial_pub.publish(msg)
                time.sleep(0.1)
            self._log('校准：已发布 /initialpose，等 AMCL 收敛')
        except Exception as exc:  # noqa: BLE001
            self._log('校准失败：%s' % exc)
        finally:
            self.calibrating = False

    def on_close(self):
        self.teleop_keys.clear()
        self._teleop_zero()
        if self.worker is not None:
            self.worker.stop()
            self.worker.finish_timing()
        self._flush_log_queue()
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
    ap.add_argument('--dwell', type=float, default=1.0)
    ap.add_argument('--strafe-ids', default=DEFAULT_STRAFE_IDS)
    ap.add_argument('--strafe-speed', type=float, default=0.15)
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
