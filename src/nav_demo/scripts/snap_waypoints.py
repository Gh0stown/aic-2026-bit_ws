#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把录好的航点收敛（投影）到预设的安全路线上。

为什么需要：
    遥控记点时，车可能停在离墙/障碍很近的地方，或者定位有几厘米抖动。
    这样的点直接发给 move_base，容易出现"目标在障碍物内/代价过高"导致
    规划失败或原地打转。把点投影到提前规划好的"走廊中线路线"上，
    既能保留大致的停靠位置，又保证目标点一定在安全区域。

输入：
    --input   航点文件（默认 nav_demo/waypoints/waypoints.yaml，record_waypoint.py 生成）
    --routes  路线文件（默认 nav_demo/config/safe_routes.yaml）
    --route   使用哪条路线（例如 route_left / route_right）

输出：
    默认写 nav_demo/waypoints/waypoints_snapped.yaml，不改原文件；
    加 --inplace 才覆盖原航点文件。

用法：
    rosrun nav_demo snap_waypoints.py --route route_left
    rosrun nav_demo snap_waypoints.py --route route_left --max-dist 0.25 --yaw route
    rosrun nav_demo snap_waypoints.py --route route_left --inplace

--yaw keep   保留录点时的朝向（默认）
--yaw route  朝向改成路线在该点的切线方向（适合只要求沿走廊行驶的点）

距离超过 --max-dist 的点不吸附（避免把点吸到别的走廊/别的房间），只给警告。
"""

import argparse
import math
import os
import sys

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_INPUT = os.path.join(PKG_DIR, 'waypoints', 'waypoints.yaml')
DEFAULT_ROUTES = os.path.join(PKG_DIR, 'config', 'safe_routes.yaml')
DEFAULT_OUTPUT = os.path.join(PKG_DIR, 'waypoints', 'waypoints_snapped.yaml')


def load_yaml(path):
    if not os.path.isfile(path):
        sys.exit('文件不存在: %s' % path)
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def as_points(seq):
    """把 [x, y] / {x: .., y: ..} 统一成 (x, y) 列表。"""
    pts = []
    for item in seq:
        if isinstance(item, dict):
            pts.append((float(item['x']), float(item['y'])))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            pts.append((float(item[0]), float(item[1])))
        else:
            raise ValueError('无法解析路线点: %r' % (item,))
    return pts


def project_to_segment(px, py, ax, ay, bx, by):
    """点 (px,py) 投影到线段 ab 上，返回 (qx, qy, 距离, 线段方向角)。"""
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-12:
        qx, qy = ax, ay
    else:
        t = ((px - ax) * dx + (py - ay) * dy) / seg2
        t = max(0.0, min(1.0, t))
        qx, qy = ax + t * dx, ay + t * dy
    dist = math.hypot(px - qx, py - qy)
    heading = math.atan2(dy, dx) if seg2 > 1e-12 else 0.0
    return qx, qy, dist, heading


def nearest_on_route(px, py, route):
    best = None
    for (ax, ay), (bx, by) in zip(route, route[1:]):
        qx, qy, dist, heading = project_to_segment(px, py, ax, ay, bx, by)
        if best is None or dist < best[2]:
            best = (qx, qy, dist, heading)
    return best


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def main():
    ap = argparse.ArgumentParser(description='把航点收敛到预设安全路线上')
    ap.add_argument('--input', default=DEFAULT_INPUT)
    ap.add_argument('--routes', default=DEFAULT_ROUTES)
    ap.add_argument('--route', required=True, help='路线名，如 route_left')
    ap.add_argument('--output', default=None)
    ap.add_argument('--inplace', action='store_true', help='直接覆盖 --input 文件')
    ap.add_argument('--max-dist', type=float, default=None,
                    help='吸附距离上限（米），超过则不吸附；默认取路线文件的 '
                         'snap_tolerance，缺省 0.30')
    ap.add_argument('--yaw', choices=['keep', 'route'], default='keep',
                    help='keep=保留记录朝向；route=取路线切线方向')
    args = ap.parse_args()

    wp_data = load_yaml(args.input)
    routes_data = load_yaml(args.routes)
    routes = routes_data.get('routes') or {}
    if args.route not in routes:
        sys.exit('路线 %s 不存在，可选: %s' % (args.route, ', '.join(routes) or '(空)'))
    route = as_points(routes[args.route])
    if len(route) < 2:
        sys.exit('路线 %s 至少需要 2 个点' % args.route)

    tol = args.max_dist
    if tol is None:
        tol = float(routes_data.get('snap_tolerance', 0.30))
    waypoints = wp_data.get('waypoints') or []
    if not waypoints:
        sys.exit('航点文件里没有 waypoints: %s' % args.input)

    changed, skipped = 0, 0
    print('%-8s %9s %9s %9s %8s %8s' % ('name', 'old_x', 'old_y', 'new_x', 'new_y', 'dist'))
    for wp in waypoints:
        if 'x' not in wp or 'y' not in wp:
            continue
        ox, oy = float(wp['x']), float(wp['y'])
        qx, qy, dist, heading = nearest_on_route(ox, oy, route)
        if dist <= tol:
            wp['x'] = round(qx, 3)
            wp['y'] = round(qy, 3)
            wp['snapped'] = True
            wp['snap_dist'] = round(dist, 3)
            if args.yaw == 'route':
                wp['yaw'] = round(normalize_angle(heading), 4)
                wp['yaw_deg'] = round(math.degrees(wp['yaw']), 1)
            changed += 1
        else:
            wp['snapped'] = False
            wp['snap_dist'] = round(dist, 3)
            skipped += 1
        print('%-8s %9.3f %9.3f %9.3f %9.3f %8.3f%s' % (
            wp.get('name', '?'), ox, oy, wp['x'], wp['y'], dist,
            '' if dist <= tol else '  <-- 超出阈值，保持原样'))

    wp_data['route'] = args.route
    out = args.input if args.inplace else (args.output or DEFAULT_OUTPUT)
    out_dir = os.path.dirname(out)
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    header = ('# 航点文件（已收敛到路线 %s，max_dist=%.2fm，yaw=%s）\n'
              % (args.route, tol, args.yaw))
    with open(out, 'w') as f:
        f.write(header)
        f.write(yaml.dump(wp_data, allow_unicode=True,
                          default_flow_style=False, sort_keys=False))
    print('\n完成：%d 个点已吸附，%d 个点超出 %.2fm 未动' % (changed, skipped, tol))
    print('输出：%s' % out)


if __name__ == '__main__':
    main()
