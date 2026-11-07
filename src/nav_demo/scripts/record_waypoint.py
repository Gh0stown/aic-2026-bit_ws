#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""遥控到目标位置后，记录小车当前位姿（map 坐标系下的 x, y, yaw）为航点。

为什么不用 rviz 的 2D Nav Goal 记点：
    Nav Goal 是"让 move_base 把车开到某个坐标"，车在目标点附近要反复修正，
    停下后的位姿不一定是你想要的位置和朝向。这里改成"人把车开到满意位置，
    按一下记录"，记下的就是车真实停住的位置和朝向，不受 DWA 收敛精度影响。

使用前提：
    已经启动导航（AMCL 在跑，能查到 map -> base_footprint 的 TF）：
        roslaunch ~/bit_ws/src/nav777.launch

用法一（推荐，终端里回车记录）：
    rosrun nav_demo record_waypoint.py
    然后用 mbot_teleop 遥控，停稳后在本终端按回车，就记录一个点。

用法二（服务方式，适合 roslaunch 启动）：
    rosrun nav_demo record_waypoint.py
    rosservice call /record_waypoint          # 记录一个点
    rosservice call /undo_waypoint            # 撤销上一个点

输出文件默认：nav_demo/waypoints/waypoints.yaml
    frame_id: map
    waypoints:
    - name: wp_1
      x: 1.234
      y: 0.567
      yaw: 1.5708      # 弧度
      yaw_deg: 90.0
      stamp: '2026-11-07 10:30:00'

参数（~ 私有）：
    ~map_frame   默认 map
    ~base_frame  默认 base_footprint
    ~output      默认 <nav_demo>/waypoints/waypoints.yaml
    ~name_prefix 默认 wp
"""

import math
import os
import sys
import threading
from datetime import datetime

import rospy
import tf
import tf.transformations as tf_trans
import yaml
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


class WaypointRecorder(object):
    def __init__(self):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.prefix = rospy.get_param('~name_prefix', 'wp')

        default_output = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'waypoints', 'waypoints.yaml')
        self.output = os.path.expanduser(rospy.get_param('~output', default_output))
        self.output = os.path.abspath(self.output)

        self.listener = tf.TransformListener()
        self.lock = threading.Lock()
        self.data = self._load()

        self.marker_pub = rospy.Publisher(
            'waypoints_markers', MarkerArray, queue_size=1, latch=True)
        self._publish_markers()

        rospy.Service('record_waypoint', Trigger, self._srv_record)
        rospy.Service('undo_waypoint', Trigger, self._srv_undo)

        rospy.loginfo('航点记录器已启动')
        rospy.loginfo('  坐标系: %s -> %s', self.map_frame, self.base_frame)
        rospy.loginfo('  保存到: %s', self.output)
        rospy.loginfo('  当前已有 %d 个航点', len(self.data['waypoints']))
        if sys.stdin is not None and sys.stdin.isatty():
            rospy.loginfo('  停稳后按回车记录；Ctrl-C 退出')

    # ---------- 文件读写 ----------
    def _load(self):
        data = {'frame_id': self.map_frame, 'waypoints': []}
        if not os.path.isfile(self.output):
            return data
        try:
            with open(self.output, 'r') as f:
                loaded = yaml.safe_load(f) or {}
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('读取 %s 失败，将新建：%s', self.output, exc)
            return data
        if isinstance(loaded, dict) and isinstance(loaded.get('waypoints'), list):
            data['waypoints'] = loaded['waypoints']
            data['frame_id'] = loaded.get('frame_id', self.map_frame)
            if data['frame_id'] != self.map_frame:
                rospy.logwarn('文件里的 frame_id=%s 与当前 map_frame=%s 不一致，'
                              '确认地图/坐标没换过', data['frame_id'], self.map_frame)
        return data

    def _save(self):
        out_dir = os.path.dirname(self.output)
        if out_dir and not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        text = '# 航点文件：由 record_waypoint.py 生成（x/y 单位米，yaw 单位弧度）\n'
        text += yaml.dump(self.data, allow_unicode=True,
                          default_flow_style=False, sort_keys=False)
        tmp = self.output + '.tmp'
        with open(tmp, 'w') as f:
            f.write(text)
        os.replace(tmp, self.output)

    # ---------- 位姿 ----------
    def _current_pose(self):
        self.listener.waitForTransform(
            self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(2.0))
        trans, rot = self.listener.lookupTransform(
            self.map_frame, self.base_frame, rospy.Time(0))
        yaw = tf_trans.euler_from_quaternion(rot)[2]
        return trans[0], trans[1], yaw

    # ---------- 记录 / 撤销 ----------
    def record(self):
        try:
            x, y, yaw = self._current_pose()
        except Exception as exc:  # noqa: BLE001
            msg = ('读不到 TF %s -> %s：%s（先确认导航/AMCL 已启动）'
                   % (self.map_frame, self.base_frame, exc))
            rospy.logerr(msg)
            return False, msg

        with self.lock:
            name = '%s_%d' % (self.prefix, len(self.data['waypoints']) + 1)
            wp = {
                'name': name,
                'x': round(x, 3),
                'y': round(y, 3),
                'yaw': round(yaw, 4),
                'yaw_deg': round(math.degrees(yaw), 1),
                'stamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            }
            self.data['waypoints'].append(wp)
            try:
                self._save()
            except Exception as exc:  # noqa: BLE001
                self.data['waypoints'].pop()
                msg = '写入 %s 失败：%s' % (self.output, exc)
                rospy.logerr(msg)
                return False, msg

        self._publish_markers()
        msg = ('已记录 %s: x=%.3f y=%.3f yaw=%.4f rad (%.1f°)'
               % (name, wp['x'], wp['y'], wp['yaw'], wp['yaw_deg']))
        rospy.loginfo(msg)
        rospy.loginfo('  共 %d 个航点 -> %s',
                      len(self.data['waypoints']), self.output)
        return True, msg

    def undo(self):
        with self.lock:
            if not self.data['waypoints']:
                return False, '没有可撤销的航点'
            wp = self.data['waypoints'].pop()
            self._save()
        self._publish_markers()
        msg = '已撤销 %s（剩 %d 个）' % (wp.get('name'), len(self.data['waypoints']))
        rospy.loginfo(msg)
        return True, msg

    # ---------- rviz 可视化 ----------
    def _publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, wp in enumerate(self.data['waypoints']):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = rospy.Time.now()
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = wp['x']
            m.pose.position.y = wp['y']
            m.pose.position.z = 0.02
            q = tf_trans.quaternion_from_euler(0.0, 0.0, wp['yaw'])
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            m.scale.x = 0.35
            m.scale.y = 0.06
            m.scale.z = 0.06
            m.color = ColorRGBA(0.1, 0.9, 0.1, 0.9)
            arr.markers.append(m)

            t = Marker()
            t.header.frame_id = self.map_frame
            t.header.stamp = rospy.Time.now()
            t.ns = 'waypoint_labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = wp['x']
            t.pose.position.y = wp['y']
            t.pose.position.z = 0.18
            t.scale.z = 0.12
            t.color = ColorRGBA(0.0, 0.0, 0.0, 0.9)
            t.text = wp.get('name', 'wp_%d' % (i + 1))
            arr.markers.append(t)
        self.marker_pub.publish(arr)

    def _srv_record(self, _req):
        ok, msg = self.record()
        return TriggerResponse(success=ok, message=msg)

    def _srv_undo(self, _req):
        ok, msg = self.undo()
        return TriggerResponse(success=ok, message=msg)

    # ---------- 终端回车 ----------
    def stdin_loop(self):
        while not rospy.is_shutdown():
            try:
                line = input()
            except EOFError:
                return
            except KeyboardInterrupt:
                return
            if rospy.is_shutdown():
                return
            if line.strip().lower() in ('q', 'quit', 'exit'):
                rospy.signal_shutdown('用户退出')
                return
            self.record()


def main():
    rospy.init_node('record_waypoint')
    recorder = WaypointRecorder()
    if sys.stdin is not None and sys.stdin.isatty():
        threading.Thread(target=recorder.stdin_loop, daemon=True).start()
    rospy.spin()


if __name__ == '__main__':
    main()
