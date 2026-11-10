#!/usr/bin/env python3
"""点位工作台回归：模拟 TF，使用临时输出文件，不初始化 ROS 节点。"""
import copy
import math
import os
import sys
import tempfile
import tkinter as tk
import types
import unittest
from unittest.mock import Mock, patch

from PIL import Image
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import waypoint_gui as gui


class WaypointGuiTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest('Tk display unavailable: %s' % exc)
        self.addCleanup(self.destroy_root)
        self.tmp = tempfile.TemporaryDirectory(prefix='waypoint_gui_test_')
        self.addCleanup(self.tmp.cleanup)
        self.output = os.path.join(self.tmp.name, 'waypoints.yaml')
        self.pose = (1.23456, -0.45678, 1.23456)
        self.source = Mock()
        self.source.get.return_value = (self.pose, True)
        presets = [{'id': pid, 'name': '巡检测试点 %s' % pid,
                    'note': '预设备注', 'guess': pid == 3} for pid in range(20)]
        recorded = {
            4: {'id': 4, 'name': '已有记录', 'x': 1.0, 'y': 1.2, 'yaw': 0.0,
                'yaw_deg': 0.0, 'note': '旧备注必须保留',
                'tasks': [{'name': '禁止通行指示牌识别', 'yaw': math.pi / 2, 'yaw_deg': 90},
                          {'name': '另一识别方向', 'yaw': math.pi, 'yaw_deg': 180}]},
            7: {'id': 7, 'name': '测试点七', 'x': 2.0, 'y': 0.4, 'yaw': 0.0},
        }
        with patch.object(gui, 'PoseSource', return_value=self.source):
            self.app = gui.WaypointGui(
                self.root, Image.new('RGB', (320, 240), 'white'),
                {'resolution': 0.025, 'origin': [-1.0, -2.0, 0.0], 'image': 'test.pgm'},
                presets, recorded, self.output)
        self.root.update()
        self.app._resize_map()

    def destroy_root(self):
        for job in self.root.tk.call('after', 'info'):
            self.root.after_cancel(job)
        self.root.destroy()

    def select(self, pid):
        self.app.tree.selection_set(str(pid))
        self.app._on_select()
        self.root.update()

    def test_task_details_and_progress_are_read_only(self):
        app = self.app
        before = copy.deepcopy(app.recorded)
        self.select(4)
        self.assertEqual(app.tree.item('4', 'values')[3], '90° / 180°')
        self.assertIn('禁止通行指示牌识别 90°', app.task_detail_var.get())
        self.assertEqual(float(app.progress['value']), 2)
        self.assertIn('缺失 18', app.summary_var.get())
        self.assertIn('推测', app.tree.item('3', 'values')[4])
        self.assertEqual(app.recorded, before)
        self.assertFalse(os.path.exists(self.output))

    def test_filter_refresh_and_hidden_selection(self):
        app = self.app
        self.select(4)
        app.missing_only_var.set(True)
        app._apply_filter()
        self.assertEqual(len(app.tree.get_children()), 18)
        self.assertFalse(app.tree.selection())
        app.refresh()  # 刷新必须同时删除脱离树的旧行，不能产生重复 iid。
        self.assertEqual(len(app.tree.get_children()), 18)
        app.search_var.set('测试点 19')
        self.assertEqual(app.tree.get_children(), ('19',))
        app.search_var.set('')
        app.missing_only_var.set(False)
        app._apply_filter()
        self.assertEqual(tuple(app.tree.get_children()), tuple(map(str, range(20))))

    def test_record_under_missing_filter_does_not_leave_hidden_selection(self):
        app = self.app
        app.missing_only_var.set(True)
        app._apply_filter()
        self.select(1)
        self.assertTrue(app._record(1, '新记录的点位', confirm=False))
        self.root.update()
        self.assertNotIn('1', app.tree.get_children())
        self.assertFalse(app.tree.selection())
        self.assertIn('选择左侧点位', app.detail_var.get())

    def test_zoom_pan_inverse_and_cursor_anchor(self):
        app = self.app
        point = (1.23, -0.78)
        for factor in (1.25, 1.25, 0.8):
            anchor = app.canvas_to_world(130, 120)
            app._zoom_map(factor, 130, 120)
            after = app.canvas_to_world(130, 120)
            for a, b in zip(anchor, after):
                self.assertAlmostEqual(a, b)
            for a, b in zip(point, app.canvas_to_world(*app.world_to_canvas(*point))):
                self.assertAlmostEqual(a, b)
        app._start_map_drag(types.SimpleNamespace(x=100, y=100))
        app._drag_map(types.SimpleNamespace(x=140, y=125))
        app._end_map_drag(types.SimpleNamespace(x=140, y=125))
        for a, b in zip(point, app.canvas_to_world(*app.world_to_canvas(*point))):
            self.assertAlmostEqual(a, b)
        app._fit_map()
        self.assertEqual((app.view_zoom, app.view_pan_x, app.view_pan_y), (1, 0, 0))

    def test_map_selection_and_next_missing_never_save(self):
        app = self.app
        before = copy.deepcopy(app.recorded)
        with patch.object(app, '_save') as save:
            app.search_var.set('不存在')
            app._select_map_point(*app.world_to_canvas(1.0, 1.2))
            self.assertEqual(app.tree.selection(), ('4',))
            self.assertTrue(app.canvas.find_withtag('selected'))
            app._select_next_missing()
            self.assertEqual(app.tree.selection(), ('5',))
            save.assert_not_called()
        self.assertEqual(app.recorded, before)

    def test_overlapping_point_labels_are_separate_and_selectable(self):
        app = self.app
        app.recorded[7]['x'], app.recorded[7]['y'] = 1.0, 1.2
        before = copy.deepcopy(app.recorded)
        app._draw_map()
        a = app.canvas.bbox(app.pose_items[4][1])
        b = app.canvas.bbox(app.pose_items[7][1])
        self.assertFalse(a[0] < b[2] and a[2] > b[0] and a[1] < b[3] and a[3] > b[1])
        app._select_map_point((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)
        self.assertEqual(app.tree.selection(), ('7',))
        self.assertEqual(app.recorded, before)

    def test_record_uses_tf_preserves_tasks_and_saves_atomically(self):
        app = self.app
        old_tasks = copy.deepcopy(app.recorded[4]['tasks'])
        app._zoom_map(2.0)
        app.view_pan_x, app.view_pan_y = 80, -40
        app._render_map()
        with patch.object(gui.messagebox, 'askyesno', return_value=True) as confirm:
            with patch.object(gui.os, 'replace', wraps=os.replace) as replace:
                self.assertTrue(app._record(4, '测试点四'))
                replace.assert_called_once_with(self.output + '.tmp', self.output)
            confirm.assert_called_once()
        with open(self.output, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        saved = next(p for p in data['points'] if p['id'] == 4)
        self.assertEqual((data['frame_id'], data['map']), ('map', 'test.pgm'))
        self.assertEqual((saved['x'], saved['y'], saved['yaw'], saved['yaw_deg']),
                         (1.235, -0.457, 1.2346, 70.7))
        self.assertEqual(saved['tasks'], old_tasks)
        self.assertEqual(saved['note'], '旧备注必须保留')
        self.assertIn('stamp', saved)
        self.assertFalse(os.path.exists(self.output + '.tmp'))

    def test_record_and_clear_keep_confirmation_and_tf_guards(self):
        app = self.app
        before = copy.deepcopy(app.recorded)
        with patch.object(gui.messagebox, 'askyesno', return_value=False):
            self.assertFalse(app._record(4, '覆盖测试'))
            self.select(4)
            app.clear_selected()
        self.assertEqual(app.recorded, before)
        self.source.get.return_value = (None, False)
        with patch.object(gui.messagebox, 'showerror') as error:
            self.assertFalse(app._record(1, '无 TF 不记录'))
            error.assert_called_once()
        self.assertEqual(app.recorded, before)
        self.assertFalse(os.path.exists(self.output))

    def test_custom_id_and_new_record_keep_original_schema(self):
        app = self.app
        app.custom_var.set('自定义巡检点')
        app.add_custom()
        self.assertEqual(app.recorded[100]['name'], '自定义巡检点')
        self.assertEqual(app.custom_var.get(), '')
        self.assertTrue(app._record(1, '新预设点', confirm=False))
        self.assertNotIn('note', app.recorded[1])  # 本次不隐式改变保存行为。
        self.assertNotIn('tasks', app.recorded[1])

    def test_lost_tf_removes_stale_arrow(self):
        app = self.app
        app._tick()
        self.assertIsNotNone(app.arrow)
        self.source.get.return_value = (self.pose, False)
        app._tick()
        self.assertIsNone(app.arrow)
        self.assertIn('等待定位', app.connection_var.get())

    def test_minimum_window_keeps_controls_and_map_footer_visible(self):
        app = self.app
        self.root.geometry('1100x760')
        self.select(4)
        self.root.update()
        for widget in (app.record_btn, app.clear_btn, app.next_btn, app.custom_entry,
                       app.add_btn, app.pose_label, app.canvas, app.tree):
            with self.subTest(widget=widget):
                self.assertTrue(widget.winfo_viewable())
                x = widget.winfo_rootx() - self.root.winfo_rootx()
                y = widget.winfo_rooty() - self.root.winfo_rooty()
                self.assertGreaterEqual(x, 0)
                self.assertGreaterEqual(y, 0)
                self.assertLessEqual(x + widget.winfo_width(), self.root.winfo_width())
                self.assertLessEqual(y + widget.winfo_height(), self.root.winfo_height())
        self.assertGreaterEqual(app.tree.winfo_height(), 68)
        self.assertGreater(app.canvas.winfo_height(), 150)


if __name__ == '__main__':
    unittest.main()
