#!/usr/bin/env python3
"""计时回归测试：不启动 ROS、Tk 窗口或车辆。"""
import io
import os
import queue
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from cruise_debug import App, CruiseTimer, CruiseWorker, format_duration


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class TimerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.timer = CruiseTimer(self.clock)

    def test_normal_finish_is_frozen(self):
        self.clock.advance(325)
        result = self.timer.finish()
        self.assertEqual(result, {'total': 325, 'paused': 0,
                                  'active': 325, 'finished': True})
        self.clock.advance(100)
        self.assertEqual(self.timer.finish(), result)
        self.assertEqual(self.timer.snapshot(), result)

    def test_multiple_pauses_and_live_elapsed(self):
        self.clock.advance(5)
        self.timer.pause()
        self.clock.advance(30)
        self.assertEqual(self.timer.snapshot()['total'], 35)
        self.assertEqual(self.timer.snapshot()['active'], 5)
        self.timer.pause()  # 重复暂停不能重置暂停起点。
        self.timer.resume()
        self.timer.resume()
        self.clock.advance(10)
        self.timer.pause()
        self.clock.advance(7)
        self.timer.resume()
        self.clock.advance(3)
        self.assertEqual(self.timer.finish(), {'total': 55, 'paused': 37,
                                              'active': 18, 'finished': True})

    def test_finish_while_paused_and_late_buttons(self):
        self.clock.advance(5)
        self.timer.pause()
        self.clock.advance(30)
        result = self.timer.finish()
        self.assertEqual(result['paused'], 30)
        self.assertEqual(result['active'], 5)
        self.clock.advance(100)
        self.timer.resume()
        self.timer.pause()
        self.assertEqual(self.timer.snapshot(), result)

    def test_new_run_gets_new_timer(self):
        self.clock.advance(20)
        self.timer.finish()
        second = CruiseTimer(self.clock)
        self.assertEqual(second.snapshot()['total'], 0)
        self.assertEqual(second.snapshot()['paused'], 0)

    def test_duration_format(self):
        self.assertEqual(format_duration(325), '5 分 25.0 秒')
        self.assertEqual(format_duration(59.99), '1 分 00.0 秒')
        self.assertEqual(format_duration(3600), '60 分 00.0 秒')


class IntegrationTests(unittest.TestCase):
    def make_worker(self, clock, log):
        return CruiseWorker([], 1, set(), 0.1, 'map', log,
                            lambda: None, run_timer=CruiseTimer(clock))

    def test_worker_logs_once_on_normal_early_stop_and_exception(self):
        for outcome in ('normal', 'early', 'stopped', 'exception'):
            with self.subTest(outcome=outcome):
                clock = Clock()
                lines = []
                worker = self.make_worker(clock, lines.append)

                def plan():
                    clock.advance(5)
                    if outcome == 'early':
                        return  # 对应启动/等待失败的提前返回。
                    worker.pause()
                    clock.advance(30)
                    if outcome == 'stopped':
                        worker._stop_evt.set()
                        return
                    worker.resume()
                    clock.advance(10)
                    if outcome == 'exception':
                        raise RuntimeError('test error')

                worker._run_plan = plan
                if outcome == 'exception':
                    with self.assertRaisesRegex(RuntimeError, 'test error'):
                        worker.run()
                else:
                    worker.run()
                result = worker.run_timer.snapshot()
                self.assertTrue(result['finished'])
                self.assertEqual(result['total'], {'normal': 45, 'early': 5,
                                                  'stopped': 35, 'exception': 45}[outcome])
                worker.finish_timing()
                self.assertEqual(sum('计时汇总' in line for line in lines), 1)
                self.assertIn('（含暂停）', lines[-1])
                self.assertIn('（不含暂停）', lines[-1])
                clock.advance(99)
                self.assertEqual(worker.run_timer.snapshot(), result)

    def test_poll_before_start_and_after_finish(self):
        clock = Clock()
        app = App.__new__(App)
        app.pose_tracker = None
        app.worker = None
        app.run_timer = None
        app.root = Mock()
        app.calibrating = False
        app.cal_btn = {'state': 'normal'}
        app._draw_pose = Mock()
        app._update_dashboard = Mock()  # 计时回归不构建只读状态面板。
        app._poll()  # 初次打开，未开始计时不能报 AttributeError。
        app.root.title.assert_not_called()
        app.run_timer = CruiseTimer(clock)
        clock.advance(325)
        app.run_timer.finish()
        for _ in range(2):
            clock.advance(20)
            app._poll()
            app.root.title.assert_called_with('巡航调试  用时 05:25')

    def test_start_shares_fresh_timer_with_worker(self):
        app = App.__new__(App)
        app.worker = None
        app.plan = [{'id': 14}]
        app._draw_highlight = Mock()
        app.dwell_var = Mock(get=lambda: '1')
        app.speed_var = Mock(get=lambda: '0.1')
        app.route_var = Mock(get=lambda: 'left')  # 适配现有路线优化开关，不更改巡航逻辑。
        app.teleop_keys = {}
        app._teleop_zero = Mock()  # 禁止测试触及实际底盘发布器。
        app._teleop_active = False
        for name, value in (('long_speed_var', '0.60'), ('mid_speed_var', '0.60'),
                            ('short_speed_var', '0.60'), ('rot_speed_var', '1.00'),
                            ('reverse_speed_var', '0.30')):
            setattr(app, name, Mock(get=Mock(return_value=value)))
        app._strafe_ids = lambda: {14, 15, 16, 17, 18}
        app.args = types.SimpleNamespace(map_frame='map', base_frame='base_footprint')
        app.pose_tracker = Mock()
        app._log = Mock()
        app.log_file = io.StringIO()
        for name in ('start_btn', 'pause_btn', 'stop_btn', 'resume_btn'):
            setattr(app, name, Mock())
        with patch('cruise_debug.CruiseWorker') as worker_type:
            app.start()
            self.assertIs(worker_type.call_args.kwargs['run_timer'], app.run_timer)
            first_timer = app.run_timer
            worker_type.return_value.is_alive.return_value = False
            app.start()
            self.assertIsNot(app.run_timer, first_timer)
            self.assertIs(worker_type.call_args.kwargs['run_timer'], app.run_timer)

    def test_close_flushes_summary_before_file_close(self):
        clock = Clock()
        app = App.__new__(App)
        app.log_queue = queue.Queue()
        app.worker = self.make_worker(clock, app.log_queue.put)
        app.worker.stop = Mock()  # 不执行任何 ROS 或底盘调用。
        app.teleop_keys = {}
        app._teleop_zero = Mock()
        app._teleop_active = False
        app.root = Mock()
        app.log_box = Mock()
        app.log_file = Mock()
        writes = []
        app.log_file.write.side_effect = writes.append
        app.log_file.close.side_effect = lambda: self.assertTrue(
            any('计时汇总' in line for line in writes))
        clock.advance(12)
        with patch.dict(sys.modules, {'rospy': Mock()}):
            app.on_close()
        self.assertEqual(sum('计时汇总' in line for line in writes), 1)
        app.log_file.close.assert_called_once()
        app.root.destroy.assert_called_once()
        app.worker.finish_timing()  # 退出清理与关闭窗口竞态不能重复汇总。
        self.assertTrue(app.log_queue.empty())


if __name__ == '__main__':
    unittest.main()
