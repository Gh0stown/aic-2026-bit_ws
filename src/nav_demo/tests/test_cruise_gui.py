#!/usr/bin/env python3
"""GUI 展示层回归：真实 Tk + 模拟 worker，不启动 ROS 或发布车辆消息。

需要 DISPLAY；没有图形环境时自动跳过。
"""
import io
import math
import os
import queue
import sys
import tkinter as tk
import types
import unittest
from unittest.mock import Mock

from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from cruise_debug import App, DRIVE_SPEED_LONG, DRIVE_SPEED_MID, DRIVE_SPEED_SHORT


class GuiTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest('Tk display unavailable: %s' % exc)
        self.addCleanup(self.destroy_root)
        self.app = app = App.__new__(App)
        app.root = self.root
        app.map_img = Image.new('RGB', (320, 240), 'white')
        app.res, app.ox, app.oy = 0.025, -1.0, -2.0
        app.args = types.SimpleNamespace(dwell=1.0, strafe_speed=0.15,
                                        no_strafe=False, strafe_ids='14,15,16,17,18')
        app.points = {
            pid: {'x': pid / 10.0, 'y': -pid / 10.0, 'yaw': 0.0,
                  'name': '测试点位 %s' % pid,
                  'tasks': [{'name': '识别', 'yaw': math.pi / 2}]}
            for pid in (0, 7, 14, 15, 18, 19)}
        app.routes = {'left': [0, '7p', 14, 15, 18, 19]}
        app.plan = []
        app.worker = None
        app.run_timer = None
        app.current_id = None
        app.pose_tracker = None
        app.calibrating = False
        app.log_queue = queue.Queue()
        app.log_file = io.StringIO()
        app.teleop_keys = {}
        app.teleop_pub = Mock()
        app._teleop_active = False
        app._twist_cls = lambda: types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0, y=0.0),
            angular=types.SimpleNamespace(z=0.0))
        app._setup_fonts()
        app._build_ui()
        app._select_route('left')
        app._reload_plan()
        self.root.update()
        app._resize_map()
        app._update_dashboard()

    def destroy_root(self):
        # 多个 Tk 根窗口顺序测试时，先取消定时器再销毁对应 Tcl 命令。
        for job in self.root.tk.call('after', 'info'):
            self.root.after_cancel(job)
        self.root.destroy()

    def worker(self, status='运行中', alive=True, index=1):
        worker = Mock()
        worker.status.return_value = ('前往测试点位', status, 14, index)
        worker.is_alive.return_value = alive
        worker.plan = self.app.plan
        self.app.worker = worker
        return worker

    def test_default_values_and_pass_through(self):
        app = self.app
        self.assertFalse(app.advanced_frame.winfo_manager())
        for var, expected in ((app.long_speed_var, DRIVE_SPEED_LONG),
                              (app.mid_speed_var, DRIVE_SPEED_MID),
                              (app.short_speed_var, DRIVE_SPEED_SHORT),
                              (app.rot_speed_var, 1.0), (app.reverse_speed_var, 0.30)):
            self.assertEqual(float(var.get()), expected)
        self.assertEqual([s['id'] for s in app.plan], [7, 14, 15, 18, 19])
        self.assertEqual(app.plan[0]['tasks'], [])  # 7p 不识别。
        self.assertEqual(app.tree.item('0', 'values')[3], '—')
        self.assertEqual(app.tree.item('4', 'values')[2], '倒车入库')
        self.assertEqual(app.plan_count_var.get(), '5 个点位')

    def test_dashboard_statuses_are_read_only(self):
        app = self.app
        for status in ('待开始', '等待 move_base', 'move_base 未启动',
                       '没有 map->base TF', '运行中', '已暂停', '已终止', '结束'):
            for alive in (True, False):
                with self.subTest(status=status, alive=alive):
                    worker = self.worker(status, alive)
                    app._update_dashboard()
                    self.assertIn(status, app.status_var.get())
                    self.assertEqual('键盘已开放' in app.teleop_hint_var.get(),
                                     alive and status == '已暂停')
                    self.assertEqual(set(c[0] for c in worker.mock_calls), {'status', 'is_alive'})
        self.worker('运行中', False)
        app._update_dashboard()
        self.assertTrue(app.status_var.get().startswith('已退出'))
        self.assertNotIn('完成', app.status_var.get())
        app.calibrating = True
        app._update_dashboard()
        self.assertEqual(app.status_var.get(), '扫描校准中')
        app.teleop_pub.publish.assert_not_called()

    def test_reloaded_plan_does_not_show_previous_run_position(self):
        self.worker('结束', False, index=4)
        self.app._reload_plan()
        self.app._update_dashboard()
        self.assertEqual(self.app.status_var.get(), '待开始')
        self.assertEqual(self.app.position_var.get(), '执行位置 00 / 05')
        self.assertIn('上轮状态：结束', self.app.step_var.get())
        self.app.teleop_pub.publish.assert_not_called()

    def test_progress_is_position_not_success_count(self):
        self.worker(index=2)
        self.app._update_dashboard()
        self.assertEqual(self.app.position_var.get(), '执行位置 03 / 05')
        self.assertEqual(float(self.app.progress['value']), 3)

    def test_map_resize_preserves_world_projection(self):
        app = self.app
        for size in ('1280x900', '1020x700'):
            self.root.geometry(size)
            self.root.update()
            app._resize_map()
            left, bottom = app._w2c(app.ox, app.oy)
            right, top = app._w2c(app.ox + 320 * app.res, app.oy + 240 * app.res)
            self.assertAlmostEqual((right - left) / (bottom - top), 320 / 240)
            self.assertAlmostEqual((left + right) / 2, app.canvas.winfo_width() / 2, delta=1)
            self.assertAlmostEqual((bottom + top) / 2, app.canvas.winfo_height() / 2, delta=1)
            self.assertGreaterEqual(left, 0)
            self.assertGreaterEqual(top, 0)
            self.assertLessEqual(right, app.canvas.winfo_width())
            self.assertLessEqual(bottom, app.canvas.winfo_height())

    def test_controls_visible_at_minimum_size_with_advanced_panel(self):
        app = self.app
        self.root.geometry('1020x700')
        for expanded in (True, False):
            app._toggle_advanced()
            self.root.update()
            self.assertEqual(bool(app.advanced_frame.winfo_manager()), expanded)
            for widget in (app.start_btn, app.stop_btn, app.cal_btn, app.tree, app.log_box,
                           app.pose_label, app.teleop_hint):
                with self.subTest(expanded=expanded, widget=widget):
                    self.assertTrue(widget.winfo_viewable())
                    x = widget.winfo_rootx() - self.root.winfo_rootx()
                    y = widget.winfo_rooty() - self.root.winfo_rooty()
                    self.assertGreaterEqual(x, 0)
                    self.assertGreaterEqual(y, 0)
                    self.assertLessEqual(x + widget.winfo_width(), self.root.winfo_width())
                    self.assertLessEqual(y + widget.winfo_height(), self.root.winfo_height())
                    self.assertGreater(widget.winfo_height(), 12)
            self.assertGreaterEqual(app.log_box.winfo_height(), 38)
            self.assertGreaterEqual(app.tree.winfo_height(), 68)  # 至少显示表头和一行。

    def test_log_color_does_not_change_record_text(self):
        app = self.app
        app._flush_log_queue()
        lines = ('点位记录：x=0.100 y=0.200 yaw=90.0° 最近=7 测试（0.01 m） 备注=',
                 '  YAML 参考：id=7 x=0.100 y=0.200 yaw_deg=90.0')
        for line in lines:
            app._log(line)
        app._flush_log_queue()
        stored = app.log_file.getvalue()
        displayed = app.log_box.get('1.0', 'end-1c')
        self.assertEqual(stored, displayed)
        for line in lines:
            self.assertIn(line + '\n', stored)
        self.assertTrue(app.log_box.tag_ranges('record'))

    def test_keyboard_pause_and_input_focus_guards(self):
        app = self.app
        self.worker('运行中')
        app.canvas.focus_force()
        self.root.update()
        app._on_key(types.SimpleNamespace(keysym='w'))
        self.assertFalse(app.teleop_keys)
        self.worker('已暂停')
        app.route_combo.focus_force()
        self.root.update()
        app._on_key(types.SimpleNamespace(keysym='w'))
        self.assertFalse(app.teleop_keys)
        app.canvas.focus_force()
        self.root.update()
        app._on_key(types.SimpleNamespace(keysym='w'))
        self.assertIn('w', app.teleop_keys)
        app._teleop_tick()
        self.assertEqual(app.teleop_pub.publish.call_args.args[0].linear.x, 0.12)
        app._on_key(types.SimpleNamespace(keysym='space'))
        self.assertFalse(app.teleop_keys)
        self.assertEqual(app.teleop_pub.publish.call_args.args[0].linear.x, 0.0)


if __name__ == '__main__':
    unittest.main()
