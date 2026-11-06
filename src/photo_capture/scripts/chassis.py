#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""区域扫描用的底盘平移控制（只做横向平移，不做旋转）。

依赖 ROS：
  * /cmd_vel   发给 newt.py（cmd_vel_listener）
  * /odom      EKF 融合后的里程计，用于闭环停步
  * /scan      可选，用于平移前净空检查

安全策略：低速、短步长、每步归零、看门狗、退出/异常必停车。
"""

import math
import threading
import time
from typing import Optional


class ChassisError(RuntimeError):
    """底盘不可用或运动失败。"""


class MoveAborted(ChassisError):
    """用户停止 / 外部中止。"""


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class ChassisController:
    def __init__(self, feedback: str = "auto", clearance: float = 0.25,
                 force: bool = False, logger=print, dry_run: bool = False):
        self.feedback = feedback if feedback in ("auto", "odom", "time") else "auto"
        self.clearance = max(0.0, float(clearance))
        self.force = bool(force)
        self.logger = logger
        self.dry_run = bool(dry_run)

        self.connected = False
        self._rospy = None
        self._pub = None
        self._cmd_lock = threading.Lock()
        self._target = (0.0, 0.0)
        self._current = (0.0, 0.0)
        self._last_heartbeat = 0.0
        self._running = False
        self._thread = None
        self._pose_lock = threading.Lock()
        self._pose = None
        self._scan_lock = threading.Lock()
        self._scan = None
        self._warned_no_odom = False
        self._warned_no_scan = False

    # -- 连接/关闭 -----------------------------------------------------
    def connect(self) -> None:
        if self.connected:
            return
        if self.dry_run:
            self.connected = True
            self.logger("[chassis] 干跑模式：不连接 ROS，不产生真实运动")
            return

        try:
            import rospy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import LaserScan
        except Exception as error:  # pragma: no cover - 取决于 ROS 环境
            raise ChassisError(
                "无法导入 ROS 模块：{}\n"
                "请先 source /opt/ros/noetic/setup.bash 和 devel/setup.bash。".format(error))

        try:
            import rosgraph
            rosgraph.Master("/photo_capture_scan_check").getPid()
        except Exception as error:
            raise ChassisError(
                "连不上 ROS master（请先启动 roscore / bringup）：{}".format(error))

        try:
            rospy.init_node("photo_capture_scan", anonymous=True, disable_signals=True)
        except Exception as error:
            raise ChassisError("初始化 ROS 节点失败：{}".format(error))

        self._rospy = rospy
        self._pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self._scan_callback, queue_size=1)

        deadline = time.monotonic() + 2.0
        while self._pub.get_num_connections() < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._pub.get_num_connections() < 1:
            raise ChassisError(
                "没有节点订阅 /cmd_vel。\n"
                "请先启动底盘：rosrun car_bringup newt.py")

        self._check_cmd_vel_conflict()

        deadline = time.monotonic() + 0.8
        while self._pose is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._pose is None:
            self.logger("[chassis] 未收到 /odom，自动退回计时运动（建议先启动 bringup）")

        self._running = True
        self._thread = threading.Thread(
            target=self._publish_loop, name="chassis-publish", daemon=True)
        self._thread.start()
        self.connected = True
        self.logger("[chassis] 已连接，反馈模式：{}".format(
            "计时" if self.feedback == "time" or self._pose is None else "里程计"))

    def close(self) -> None:
        self.stop()
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._pub is not None:
            try:
                self._pub.unregister()
            except Exception:
                pass
            self._pub = None
        self.connected = False

    # -- 状态 ---------------------------------------------------------
    def _odom_callback(self, message) -> None:
        pose = message.pose.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        with self._pose_lock:
            self._pose = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": yaw,
                "stamp": time.time(),
                "mono": time.monotonic(),
            }

    def _scan_callback(self, message) -> None:
        with self._scan_lock:
            self._scan = {
                "ranges": list(message.ranges),
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "mono": time.monotonic(),
            }

    def last_pose(self) -> Optional[dict]:
        with self._pose_lock:
            if self._pose is None:
                return None
            return dict(self._pose)

    def _fresh_pose(self, max_age: float = 0.5) -> Optional[dict]:
        pose = self.last_pose()
        if pose is None or time.monotonic() - pose["mono"] > max_age:
            return None
        return pose

    # -- 安全检查 -----------------------------------------------------
    def _check_cmd_vel_conflict(self) -> None:
        try:
            response = self._rospy.get_master().getSystemState()
            state = response[2] if len(response) == 3 else response
        except Exception:
            return
        my_name = self._rospy.get_name()
        publishers = []
        subscribers = []
        for topic, nodes in state[0]:
            if topic == "/cmd_vel":
                publishers.extend(nodes)
        for topic, nodes in state[1]:
            if topic == "/cmd_vel":
                subscribers.extend(nodes)

        others = [name for name in publishers if name != my_name]
        if others and not self.force:
            raise ChassisError(
                "检测到其他节点也在发布 /cmd_vel：{}\n"
                "请先停止遥控/导航，或加 --force 强制继续。".format("、".join(others)))
        if others:
            self.logger("[chassis] 警告：其他 /cmd_vel 发布者：{}".format("、".join(others)))

        if not subscribers:
            raise ChassisError(
                "没有节点订阅 /cmd_vel。\n"
                "请先启动底盘：rosrun car_bringup newt.py")
        if not any(str(name).endswith("cmd_vel_listener") for name in subscribers):
            self.logger(
                "[chassis] 警告：/cmd_vel 订阅者不是 cmd_vel_listener：{}".format(
                    "、".join(subscribers)))

    def _check_clearance(self, direction: str, clearance: float) -> None:
        if clearance <= 0:
            return
        with self._scan_lock:
            scan = self._scan
        if scan is None or time.monotonic() - scan["mono"] > 0.5:
            if not self._warned_no_scan:
                self._warned_no_scan = True
                self.logger("[chassis] 没有 /scan 数据，跳过平移净空检查")
            return

        low = math.radians(60.0)
        high = math.radians(120.0)
        if direction == "right":
            low, high = -high, -low

        nearest = None
        for index, value in enumerate(scan["ranges"]):
            if not isinstance(value, float) or math.isnan(value):
                continue
            if value < scan["range_min"] or value > scan["range_max"]:
                continue
            angle = scan["angle_min"] + index * scan["angle_increment"]
            if low <= angle <= high:
                if nearest is None or value < nearest:
                    nearest = value
        if nearest is not None and nearest < clearance:
            raise ChassisError(
                "平移方向净空不足：最近障碍 {:.2f}m < {:.2f}m，已拒绝移动".format(
                    nearest, clearance))

    # -- 运动 ---------------------------------------------------------
    def strafe(self, distance: float, speed: float, direction: str,
               stop_event=None, clearance: Optional[float] = None,
               feedback: Optional[str] = None) -> dict:
        distance = max(0.01, min(5.0, float(distance)))
        speed = max(0.02, min(0.10, float(speed)))
        sign = 1.0 if direction != "right" else -1.0
        mode = feedback or self.feedback
        clearance = self.clearance if clearance is None else clearance

        if not self.connected:
            raise ChassisError("底盘未连接")
        self._check_clearance(direction, clearance)

        start_pose = self._fresh_pose()
        use_odom = mode == "odom" or (mode == "auto" and start_pose is not None)
        if mode == "odom" and start_pose is None:
            raise ChassisError("指定的里程计反馈不可用（/odom 没有数据）")
        if not use_odom and not self._warned_no_odom:
            self._warned_no_odom = True
            self.logger("[chassis] 使用计时平移，距离精度取决于底盘速度标定")

        self._set_target(0.0, sign * speed)
        started = time.monotonic()
        last_heartbeat = started
        timeout = distance / speed * 2.0 + 3.0
        moved = 0.0
        try:
            while True:
                if stop_event is not None and stop_event.is_set():
                    raise MoveAborted()
                if self._rospy is not None and self._rospy.is_shutdown():
                    raise ChassisError("ROS 已关闭")
                now = time.monotonic()
                if now - last_heartbeat >= 0.1:
                    self._set_target(0.0, sign * speed)
                    last_heartbeat = now
                if now - started > timeout:
                    raise ChassisError("平移超时（{:.1f}s）".format(timeout))
                if use_odom:
                    pose = self._fresh_pose()
                    if pose is None:
                        raise ChassisError("平移中 /odom 中断，已停车")
                    if self._base_delta_y(start_pose, pose) * sign >= distance - 0.005:
                        moved = self._base_delta_y(start_pose, pose) * sign
                        break
                elif now - started >= distance / speed:
                    moved = distance
                    break
                time.sleep(0.02)
        finally:
            self.stop()

        if use_odom:
            pose = self._fresh_pose()
            if pose is not None:
                moved = self._base_delta_y(start_pose, pose) * sign
        return {"distance": moved, "direction": direction, "feedback": "odom" if use_odom else "time"}

    @staticmethod
    def _base_delta_y(start: dict, current: dict) -> float:
        dx = current["x"] - start["x"]
        dy = current["y"] - start["y"]
        yaw = start["yaw"]
        return -math.sin(yaw) * dx + math.cos(yaw) * dy

    # -- 速度发布 -----------------------------------------------------
    def _publish_loop(self) -> None:
        from geometry_msgs.msg import Twist
        while self._running:
            now = time.monotonic()
            with self._cmd_lock:
                if now - self._last_heartbeat > 0.5:
                    self._target = (0.0, 0.0)
                target_x, target_y = self._target
                current_x, current_y = self._current
            # 简单斜坡，避免速度突变
            step = 0.4 * 0.05
            current_x += max(-step, min(step, target_x - current_x))
            current_y += max(-step, min(step, target_y - current_y))
            if target_x == 0.0 and target_y == 0.0:
                current_x, current_y = 0.0, 0.0
            with self._cmd_lock:
                self._current = (current_x, current_y)
            message = Twist()
            message.linear.x = current_x
            message.linear.y = current_y
            try:
                self._pub.publish(message)
            except Exception:
                pass
            time.sleep(0.05)

    def _set_target(self, vx: float, vy: float) -> None:
        with self._cmd_lock:
            self._target = (float(vx), float(vy))
            self._last_heartbeat = time.monotonic()

    def stop(self) -> None:
        with self._cmd_lock:
            self._target = (0.0, 0.0)
            self._current = (0.0, 0.0)
            self._last_heartbeat = 0.0
        if self._pub is not None:
            try:
                from geometry_msgs.msg import Twist
                self._pub.publish(Twist())
            except Exception:
                pass
