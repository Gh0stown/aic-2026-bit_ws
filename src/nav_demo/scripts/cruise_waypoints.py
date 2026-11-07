#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按点位顺序巡航：move_base 开到点位，识别点原地转向；14→18 段用麦轮平移。

流程：
    1. 普通点位：发 move_base 目标 (x, y, 点位记录朝向)，车开到点位；
    2. 识别点：对每个 task 发同坐标、不同 yaw 的目标 → 原地转向，停留 --dwell 秒；
    3. 14→18 段：不再用 move_base 目标，而是像 photo_capture 的区域扫描那样，
       直接发 linear.y 做低速、里程计闭环的横向平移，车头保持不动；
       到 15、17 再原地转向识别，识别完转回行驶朝向继续平移。

用法：
    rosrun nav_demo cruise_waypoints.py --route fast --dry-run
    rosrun nav_demo cruise_waypoints.py --route fast --dwell 3
    rosrun nav_demo cruise_waypoints.py --order 0,1,2,3,4,5,13,12,11,9,14,15,16,17,18,19

参数：
    --strafe-ids    走平移的点位（默认 14,15,16,17,18；相邻两点之间用平移）
    --strafe-speed  平移速度 m/s（默认 0.06，范围 0.02~0.10）
    --no-strafe     关闭平移，全部用 move_base
"""

import argparse
import math
import os
import sys
import threading
import time

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_POINTS = os.path.join(PKG_DIR, 'waypoints', 'inspection_waypoints.yaml')
DEFAULT_ROUTES = os.path.join(PKG_DIR, 'config', 'inspection_routes.yaml')
DEFAULT_STRAFE_IDS = '14,15,16,17,18'


def load_yaml(path):
    if not os.path.isfile(path):
        sys.exit('文件不存在：%s' % path)
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def snap90(deg):
    """就近对齐到 90° 的整数倍。"""
    s = round(deg / 90.0) * 90
    if s == -180:
        s = 180
    return s


def angle_diff(a, b):
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(d)


def build_plan(points, order):
    plan = []
    for pid in order:
        p = points.get(pid)
        if p is None:
            print('[警告] 点位 %s 不存在，跳过' % pid)
            continue
        tasks = [(t.get('name', ''), float(t.get('yaw', 0.0)))
                 for t in (p.get('tasks') or [])]
        arrive_yaw = math.radians(snap90(math.degrees(float(p.get('yaw', 0.0)))))
        plan.append({
            'id': pid,
            'name': p.get('name', ''),
            'x': float(p['x']),
            'y': float(p['y']),
            'yaw': arrive_yaw,
            'tasks': tasks,
        })
    return plan


def plan_modes(plan, strafe_ids):
    """给每一步标注到点方式：strafe / move_base。"""
    modes = []
    prev = None
    for step in plan:
        if prev is not None and prev['id'] in strafe_ids and step['id'] in strafe_ids:
            modes.append('strafe')
        else:
            modes.append('move_base')
        prev = step
    return modes


def lateral_delta(prev_step, step):
    """从 prev 到 step 的横向位移（以 step 的到达朝向为基准），正=左。"""
    dx = step['x'] - prev_step['x']
    dy = step['y'] - prev_step['y']
    yaw = step['yaw']
    return -math.sin(yaw) * dx + math.cos(yaw) * dy


def print_plan(plan, dwell, strafe_ids):
    modes = plan_modes(plan, strafe_ids)
    print('%-4s %-26s %-22s %-10s %-26s' % ('ID', '点位', '到达坐标/朝向', '到点方式', '识别朝向'))
    for step, mode in zip(plan, modes):
        arrive = '(%.3f, %.3f, %.0f°)' % (
            step['x'], step['y'], math.degrees(step['yaw']))
        if step['tasks']:
            ts = ' → '.join('%s %.0f°' % (n, math.degrees(y)) for n, y in step['tasks'])
        else:
            ts = '（仅通过/行驶）'
        print('%-4s %-26s %-22s %-10s %-26s'
              % (step['id'], step['name'][:24], arrive, mode, ts))
    print('\n共 %d 个点，每个识别朝向停留 %.1f 秒' % (len(plan), dwell))


class StrafeController(object):
    """麦轮横向平移：只发 linear.y，/odom 闭环，低速、斜坡、超时、必停车。"""

    def __init__(self, speed=0.06, dry_run=False):
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        self._rospy = rospy
        self._twist = Twist
        self.speed = max(0.02, min(0.10, float(speed)))
        self.dry_run = dry_run
        self.pub = None
        self._pose = None
        self._lock = threading.Lock()
        if dry_run:
            return

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)
        deadline = time.monotonic() + 3.0
        while self._fresh_pose() is None and time.monotonic() < deadline:
            if rospy.is_shutdown():
                break
            time.sleep(0.05)
        if self._fresh_pose() is None:
            raise RuntimeError('/odom 没有数据：平移需要 EKF 的 /odom，先启动 nav777')

    def _odom_cb(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z +
                               p.orientation.x * p.orientation.y),
                         1.0 - 2.0 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        with self._lock:
            self._pose = (p.position.x, p.position.y, yaw, time.monotonic())

    def _fresh_pose(self, max_age=0.5):
        with self._lock:
            pose = self._pose
        if pose is None or time.monotonic() - pose[3] > max_age:
            return None
        return pose

    @staticmethod
    def _delta_y(start, current):
        dx = current[0] - start[0]
        dy = current[1] - start[1]
        return -math.sin(start[2]) * dx + math.cos(start[2]) * dy

    def strafe(self, distance, direction):
        if self.dry_run:
            print('    [dry-run] 平移 %.2fm %s' % (distance, direction))
            return True
        distance = abs(float(distance))
        sign = 1.0 if direction == 'left' else -1.0
        start = self._fresh_pose()
        if start is None:
            self._stop()
            return False

        timeout = distance / self.speed * 2.0 + 3.0
        started = time.monotonic()
        current = 0.0
        rate = self._rospy.Rate(20)
        try:
            while not self._rospy.is_shutdown():
                pose = self._fresh_pose()
                if pose is None:
                    self._rospy.logerr('平移中 /odom 中断，停车')
                    return False
                moved = self._delta_y(start, pose) * sign
                if moved >= distance - 0.005:
                    return True
                if time.monotonic() - started > timeout:
                    self._rospy.logerr('平移超时（%.0fs）', timeout)
                    return False
                current = min(self.speed, current + 0.02)
                twist = self._twist()
                twist.linear.y = sign * current
                self.pub.publish(twist)
                rate.sleep()
        finally:
            self._stop()
        return False

    def _stop(self):
        if self.pub is None:
            return
        try:
            for _ in range(3):
                self.pub.publish(self._twist())
                time.sleep(0.05)
        except Exception:  # noqa: BLE001
            pass


def run(plan, dwell, timeout, retries, strict, no_tasks, map_frame,
        strafe_ids, strafe_speed, no_strafe):
    import actionlib
    import rospy
    from actionlib_msgs.msg import GoalStatus
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from tf.transformations import quaternion_from_euler

    rospy.init_node('cruise_waypoints')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo('等待 move_base action server ...')
    if not client.wait_for_server(rospy.Duration(10.0)):
        sys.exit('连不上 move_base，确认导航已启动：roslaunch ~/bit_ws/src/nav777.launch')

    rospy.on_shutdown(client.cancel_all_goals)

    strafe = None
    if not no_strafe and strafe_ids:
        strafe = StrafeController(speed=strafe_speed)
        rospy.loginfo('麦轮平移已启用：%s（%.2f m/s）',
                      ','.join(str(i) for i in sorted(strafe_ids)), strafe_speed)

    def send_goal(x, y, yaw, label):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        for attempt in range(retries + 1):
            if rospy.is_shutdown():
                return False
            rospy.loginfo('  -> %s (%.3f, %.3f, %.0f°) [第 %d 次]',
                          label, x, y, math.degrees(yaw), attempt + 1)
            client.send_goal(goal)
            if client.wait_for_result(rospy.Duration(timeout)):
                if client.get_state() == GoalStatus.SUCCEEDED:
                    return True
                rospy.logwarn('    未成功（state=%s）', client.get_state())
            else:
                rospy.logwarn('    超时（%.0fs），取消', timeout)
                client.cancel_goal()
        return False

    ok_count, fail_count = 0, 0
    prev = None
    for idx, step in enumerate(plan):
        if rospy.is_shutdown():
            break
        via_strafe = (strafe is not None and prev is not None
                      and prev['id'] in strafe_ids and step['id'] in strafe_ids)
        rospy.loginfo('[%d/%d] %s %s（%s）', idx + 1, len(plan),
                      '平移前往' if via_strafe else '前往', step['id'], step['name'])

        arrived = False
        if via_strafe:
            lateral = lateral_delta(prev, step)
            dist = abs(lateral)
            direction = 'left' if lateral > 0 else 'right'
            rospy.loginfo('  [平移] %.2f m %s（保持车头 %.0f°）',
                          dist, direction, math.degrees(step['yaw']))
            if dist < 0.01:
                arrived = True
            else:
                arrived = strafe.strafe(dist, direction)
        else:
            arrived = send_goal(step['x'], step['y'], step['yaw'], '点位 %s' % step['id'])

        if not arrived:
            rospy.logerr('到达 %s 失败', step['id'])
            fail_count += 1
            if strict:
                sys.exit('严格模式：中止')
            prev = step
            continue
        ok_count += 1

        last_yaw = step['yaw']
        if not no_tasks:
            for name, yaw in step['tasks']:
                rospy.loginfo('   识别：%s，转向 %.0f°', name, math.degrees(yaw))
                if send_goal(step['x'], step['y'], yaw, name):
                    ok_count += 1
                    last_yaw = yaw
                    rospy.sleep(dwell)
                else:
                    rospy.logwarn('   转向 %s 失败', name)
                    fail_count += 1

        # 下一段还是平移的话，先把车头转回行驶朝向再平移
        next_step = plan[idx + 1] if idx + 1 < len(plan) else None
        next_is_strafe = (strafe is not None and next_step is not None
                          and step['id'] in strafe_ids and next_step['id'] in strafe_ids
                          and not no_tasks)
        if next_is_strafe and angle_diff(last_yaw, step['yaw']) > 0.05:
            rospy.loginfo('   转回行驶朝向 %.0f° 再继续平移', math.degrees(step['yaw']))
            if send_goal(step['x'], step['y'], step['yaw'], '恢复朝向'):
                ok_count += 1
            else:
                fail_count += 1

        prev = step

    rospy.loginfo('巡航结束：成功 %d，失败 %d', ok_count, fail_count)


def main():
    ap = argparse.ArgumentParser(description='按点位顺序巡航并转向识别')
    ap.add_argument('--points', default=DEFAULT_POINTS)
    ap.add_argument('--routes', default=DEFAULT_ROUTES)
    ap.add_argument('--route', default=None, help='routes 文件里的路线名')
    ap.add_argument('--order', default=None, help='直接给顺序，如 0,1,2,3')
    ap.add_argument('--dwell', type=float, default=2.0, help='每个识别朝向停留秒数')
    ap.add_argument('--timeout', type=float, default=120.0, help='单个目标超时秒数')
    ap.add_argument('--retries', type=int, default=1, help='单个目标重试次数')
    ap.add_argument('--start', type=int, default=0, help='从路线第几个点开始（0 起）')
    ap.add_argument('--strict', action='store_true', help='任一点失败就中止')
    ap.add_argument('--no-tasks', action='store_true', help='只行驶，不做识别转向')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--strafe-ids', default=DEFAULT_STRAFE_IDS,
                    help='走麦轮平移的点位，相邻两点之间平移')
    ap.add_argument('--strafe-speed', type=float, default=0.06, help='平移速度 m/s')
    ap.add_argument('--no-strafe', action='store_true', help='关闭平移，全部用 move_base')
    ap.add_argument('--dry-run', action='store_true', help='只打印计划，不连 ROS')
    args = ap.parse_args()

    data = load_yaml(args.points)
    points = {int(p['id']): p for p in data.get('points', [])}
    routes = load_yaml(args.routes).get('routes', {}) if os.path.isfile(args.routes) else {}

    if args.order:
        order = [int(x) for x in args.order.replace(' ', '').split(',') if x != '']
    elif args.route:
        if args.route not in routes:
            sys.exit('路线 %s 不存在，可选：%s' % (args.route, ', '.join(routes) or '(空)'))
        order = [int(x) for x in routes[args.route]]
    else:
        sys.exit('需要 --route 或 --order')

    if args.start > 0:
        order = order[args.start:]

    strafe_ids = set()
    if args.strafe_ids and not args.no_strafe:
        strafe_ids = {int(x) for x in args.strafe_ids.replace(' ', '').split(',') if x != ''}

    plan = build_plan(points, order)
    print_plan(plan, args.dwell, strafe_ids)

    if args.dry_run:
        return

    run(plan, args.dwell, args.timeout, args.retries, args.strict, args.no_tasks,
        args.map_frame, strafe_ids, args.strafe_speed, args.no_strafe)


if __name__ == '__main__':
    main()
