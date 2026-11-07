#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘串口驱动：订阅 /cmd_vel，按固定频率把速度写给底盘。

相对旧版改动：
    1. 不再“来一条 /cmd_vel 才写一包”，而是以 ~rate Hz 持续重复发送最后一条速度，
       底盘收到的指令是连续的，不会因为 move_base 只有 5Hz 而一冲一停；
    2. 看门狗：超过 ~watchdog 秒没收到新 /cmd_vel，自动发零速度停车；
    3. 退出（Ctrl-C）时发零速度并关串口，避免“脚本停了车还在跑”；
    4. 去掉了每条消息里的 print，减少回调阻塞。
协议保持不变：AA BB 0A 12 02 xL xH yL yH thL thH 00。
"""

import threading
import time

import rospy
import serial
from geometry_msgs.msg import Twist


def encode(v):
    """速度（m/s 或 rad/s）→ 两个字节（低字节在前），×1000 取 16 位补码。"""
    n = int(v * 1000) & 0xFFFF
    return n & 0xFF, (n >> 8) & 0xFF


class ChassisDriver(object):
    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/carserial')
        self.rate_hz = float(rospy.get_param('~rate', 20.0))
        self.watchdog = float(rospy.get_param('~watchdog', 0.5))

        self.ser = serial.Serial(self.port, 115200, timeout=5)
        rospy.loginfo('串口已打开：%s', self.port)

        self.lock = threading.Lock()
        self.last = Twist()
        self.last_time = 0.0

        self._init_chassis()

        rospy.Subscriber('cmd_vel', Twist, self._cmd_cb, queue_size=1)
        rospy.on_shutdown(self._shutdown)
        threading.Thread(target=self._send_loop, daemon=True).start()
        rospy.loginfo('底盘驱动就绪：%.0fHz 重复发送，%.1fs 无指令自动停车',
                      self.rate_hz, self.watchdog)

    def _init_chassis(self):
        self.ser.write(b'\x11')
        for _ in range(8):
            time.sleep(0.001)
            self.ser.write(b'\x00')

    def _cmd_cb(self, msg):
        with self.lock:
            self.last = msg
            self.last_time = time.time()

    def _write(self, twist):
        xl, xh = encode(twist.linear.x)
        yl, yh = encode(twist.linear.y)
        tl, th = encode(twist.angular.z)
        packet = [0xAA, 0xBB, 0x0A, 0x12, 0x02,
                  xl, xh, yl, yh, tl, th, 0x00]
        # 保持和旧驱动一致的逐字节发送 + 间隔，避免部分底盘固件吞包
        for b in packet:
            self.ser.write(bytes([b]))
            time.sleep(0.0004)

    def _send_loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                twist = self.last
                fresh = (time.time() - self.last_time) <= self.watchdog
            if not fresh:
                twist = Twist()
            try:
                self._write(twist)
            except Exception as exc:  # noqa: BLE001
                rospy.logerr_throttle(1.0, '串口写入失败：%s', exc)
            rate.sleep()

    def _shutdown(self):
        try:
            self._write(Twist())
        except Exception:  # noqa: BLE001
            pass
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        rospy.loginfo('已发送零速度并关闭串口')


def main():
    rospy.init_node('cmd_vel_listener')
    ChassisDriver()
    rospy.spin()


if __name__ == '__main__':
    main()
