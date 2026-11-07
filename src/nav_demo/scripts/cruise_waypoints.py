#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按点位顺序巡航，并在识别点原地转向到各个识别朝向。

流程（每个点位）：
    1. 发 move_base 目标 (x, y, 点位记录朝向) → 车开到点位；
    2. 若该点有 tasks：对每个 task 发同坐标、不同 yaw 的目标 → 车原地转向；
       每转到一个识别朝向停留 --dwell 秒（识别/拍照时间）；
    3. 前往下一个点位（下一步的目标朝向会自然把车头带过去）。

用法：
    rosrun nav_demo cruise_waypoints.py --route left
    rosrun nav_demo cruise_waypoints.py --route right --dwell 3
    rosrun nav_demo cruise_waypoints.py --order 0,1,2,3,4,5,13,12,11,9,14,15,16,17,18,19
    rosrun nav_demo cruise_waypoints.py --route left --dry-run     # 只打印计划

点位和 tasks 来自：
    src/nav_demo/waypoints/inspection_waypoints.yaml
路线顺序来自：
    src/nav_demo/config/inspection_routes.yaml

注意：路线顺序和点位编号不是一回事。点位表只是“记录”，实际怎么走由 routes 里的
列表决定；比如 13 和 14 之间跨度大，可以改成经过 11、9 的走法，直接在 routes 里调整顺序。
"""

import argparse
import math
import os
import sys

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_POINTS = os.path.join(PKG_DIR, 'waypoints', 'inspection_waypoints.yaml')
DEFAULT_ROUTES = os.path.join(PKG_DIR, 'config', 'inspection_routes.yaml')


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


def build_plan(points, order):
    plan = []
    for pid in order:
        p = points.get(pid)
        if p is None:
            print('[警告] 点位 %s 不存在，跳过' % pid)
            continue
        tasks = [(t.get('name', ''), float(t.get('yaw', 0.0)))
                 for t in (p.get('tasks') or [])]
        # 到点朝向沿用车头记录值（对齐 90°），这样 14→18 段可以保持车头、
        # 用麦轮平移；识别朝向在到点后再转
        arrive_yaw = math.radians(snap90(math.degrees(float(p.get('yaw', 0.0)))))
        plan.append({
            'id': pid,
            'name': p.get('name', ''),
            'x': float(p['x']),
            'y': float(p['y']),
            'yaw': arrive_yaw,
            'record_yaw': float(p.get('yaw', 0.0)),
            'tasks': tasks,
        })
    return plan


def print_plan(plan, dwell):
    print('%-4s %-28s %-20s %-26s' % ('ID', '点位', '到达坐标/朝向', '识别朝向'))
    for step in plan:
        arrive = '(%.3f, %.3f, %.0f°)' % (
            step['x'], step['y'], math.degrees(step['yaw']))
        if step['tasks']:
            ts = ' → '.join('%s %.0f°' % (n, math.degrees(y)) for n, y in step['tasks'])
        else:
            ts = '（仅通过/行驶）'
        print('%-4s %-28s %-20s %-26s' % (step['id'], step['name'][:26], arrive, ts))
    total = sum(1 + len(s['tasks']) for s in plan)
    print('\n共 %d 个点、%d 次移动/转向，每个识别朝向停留 %.1f 秒'
          % (len(plan), total, dwell))


def run(plan, dwell, timeout, retries, strict, no_tasks, map_frame):
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
                state = client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    return True
                rospy.logwarn('    未成功（state=%s）', state)
            else:
                rospy.logwarn('    超时（%.0fs），取消', timeout)
                client.cancel_goal()
        return False

    ok_count, fail_count = 0, 0
    for idx, step in enumerate(plan):
        if rospy.is_shutdown():
            break
        rospy.loginfo('[%d/%d] 前往 %s（%s）', idx + 1, len(plan), step['id'], step['name'])
        if not send_goal(step['x'], step['y'], step['yaw'], '点位 %s' % step['id']):
            rospy.logerr('到达 %s 失败', step['id'])
            fail_count += 1
            if strict:
                sys.exit('严格模式：中止')
            continue
        ok_count += 1

        if not no_tasks:
            for name, yaw in step['tasks']:
                rospy.loginfo('   识别：%s，转向 %.0f°', name, math.degrees(yaw))
                if send_goal(step['x'], step['y'], yaw, name):
                    ok_count += 1
                    rospy.sleep(dwell)
                else:
                    rospy.logwarn('   转向 %s 失败', name)
                    fail_count += 1

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

    plan = build_plan(points, order)
    print_plan(plan, args.dwell)

    if args.dry_run:
        return

    run(plan, args.dwell, args.timeout, args.retries, args.strict, args.no_tasks, args.map_frame)


if __name__ == '__main__':
    main()
