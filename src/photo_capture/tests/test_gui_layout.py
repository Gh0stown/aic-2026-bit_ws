#!/usr/bin/env python3
"""真实 Tk 展示回归；不初始化相机、检测器、ROS 或扫描任务。

与 nav_demo/tests 一致使用 unittest；无 DISPLAY 自动跳过 GUI 测试。
"""
from pathlib import Path
import sys
import tempfile
import tkinter as tk
import tkinter.font as tkfont
import unittest
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import capture_photos as gui


APP_CLASSES = (gui.PhotoApp, gui.DetectApp, gui.ScanApp, gui.GaugeApp)
CALLBACKS = ('shoot', 'change_type', '_on_count_change', 'open_folder',
             'save_snapshot', 'save_crops', 'change_model', '_sync_conf',
             'start_scan', 'stop_scan', 'detect_panorama', 'view_panorama',
             'save_readings', 'recalibrate', 'show_rectified')


def fake_app(root, cls):
    """只构建布局：刻意绕过会启动 tick 或创建 rectifier 的构造器。"""
    app = cls.__new__(cls)
    app.root = root
    app.camera = Mock(name='fake_camera')
    app.detector = Mock(name='fake_detector')
    app.detector.conf = 0.25
    app.output_root = Path('/tmp/visual-preview')
    app.snapshot_root = app.output_root / '模型测试'
    app._bold_font = tkfont.Font(root=root, size=12, weight='bold')
    app._font_path = None
    app._font_cache = {}
    app._fps = 0.0
    app._dialog_open = False
    app._closed = False
    app._readings = []
    app.settings = gui.scan_module.ScanSettings()
    for name, value in (('type_var', '正面'), ('count_var', '3'),
                        ('model_var', 'example.pt'), ('conf_var', '0.25'),
                        ('infer_var', '推理：--'), ('det_var', '目标：0'),
                        ('fps_var', '预览：--'), ('status_var', '测试窗口 · 未连接任何设备'),
                        ('mode_var', '自动平移'), ('progress_var', '等待开始')):
        setattr(app, name, tk.StringVar(master=root, value=value))
    for name in CALLBACKS:
        setattr(app, name, Mock(name=name))
    app._build_ui()
    root.update()
    return app


def widgets(widget):
    yield widget
    for child in widget.winfo_children():
        yield from widgets(child)


class LayoutTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest('Tk display unavailable: %s' % exc)
        self.addCleanup(self.cleanup_root)
        self.video_capture = patch.object(gui.cv2, 'VideoCapture', side_effect=AssertionError('hardware access'))
        self.video_capture.start()
        self.addCleanup(self.video_capture.stop)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def cleanup_root(self):
        for job in self.root.tk.call('after', 'info'):
            self.root.after_cancel(job)
        self.root.destroy()

    def clear(self):
        for job in self.root.tk.call('after', 'info'):
            self.root.after_cancel(job)
        for child in self.root.winfo_children():
            child.destroy()

    def assert_inside(self, widget):
        self.assertTrue(widget.winfo_viewable(), str(widget))
        x = widget.winfo_rootx() - self.root.winfo_rootx()
        y = widget.winfo_rooty() - self.root.winfo_rooty()
        self.assertGreaterEqual(x, 0)
        self.assertGreaterEqual(y, 0)
        self.assertLessEqual(x + widget.winfo_width(), self.root.winfo_width())
        self.assertLessEqual(y + widget.winfo_height(), self.root.winfo_height())

    def test_four_workspaces_keep_primary_controls_and_preview_accessible(self):
        for cls in APP_CLASSES:
            for size in ('1140x800', '760x560', '560x420'):
                with self.subTest(app=cls.__name__, size=size):
                    self.clear()
                    app = fake_app(self.root, cls)
                    self.root.geometry(size)
                    self.root.update()
                    self.assert_inside(app.preview)
                    self.assertGreater(app.preview.winfo_width(), 80)
                    self.assertGreater(app.preview.winfo_height(), 80)
                    self.assert_inside(app.status_label)
                    if cls is gui.ScanApp:
                        self.assert_inside(app.start_button)
                        self.assert_inside(app.stop_button)
                        self.assertEqual(app.stop_button['style'], 'Capture.Danger.TButton')
                        self.assertEqual(str(app.stop_button['state']), 'disabled')
                    for name in CALLBACKS:
                        getattr(app, name).assert_not_called()
                    app.camera.assert_not_called()
                    app.detector.assert_not_called()

    def test_buttons_keep_original_callbacks_and_initial_states(self):
        expected = {
            gui.PhotoApp: {'拍照（空格）': 'shoot', '更换类型 (T)': 'change_type',
                           '打开保存目录': 'open_folder'},
            gui.DetectApp: {'保存当前帧 (S)': 'save_snapshot', '保存裁剪图 (C)': 'save_crops',
                            '更换模型 (M)': 'change_model'},
            gui.ScanApp: {'开始扫描': 'start_scan', '停止': 'stop_scan', '查看全景': 'view_panorama',
                          '检测全景': 'detect_panorama', '保存裁剪 (C)': 'save_crops',
                          '打开目录': 'open_folder'},
            gui.GaugeApp: {'保存读数 (R)': 'save_readings', '保存当前帧 (S)': 'save_snapshot',
                           '正视化 (V)': 'show_rectified', '重新标定 (T)': 'recalibrate',
                           '打开记录目录': 'open_folder', '更换模型 (M)': 'change_model'},
        }
        for cls in APP_CLASSES:
            self.clear()
            app = fake_app(self.root, cls)
            buttons = {w['text']: w for w in widgets(self.root) if isinstance(w, gui.ttk.Button)}
            self.assertEqual(set(buttons), set(expected[cls]))
            for text, callback in expected[cls].items():
                with self.subTest(app=cls.__name__, button=text):
                    button = buttons[text]
                    if cls is gui.ScanApp and text != '开始扫描':
                        self.assertEqual(str(button['state']), 'disabled')
                    button.configure(state='normal')  # 仅测试绑定，不运行真实 callbacks。
                    button.invoke()
                    getattr(app, callback).assert_called_once_with()

    def test_preview_resize_is_display_only_and_retains_static_frame(self):
        app = fake_app(self.root, gui.ScanApp)
        image = Image.new('RGB', (1600, 900), '#255b70')
        original_bytes = image.tobytes()
        photo = app.preview.fit_image(image)
        app.preview.configure(image=photo, text='')
        for size in ('1140x800', '560x420', '900x650'):
            self.root.geometry(size)
            self.root.update()
            displayed = app.preview._display_photo
            self.assertLessEqual(displayed.width(), app.preview.winfo_width())
            self.assertLessEqual(displayed.height(), app.preview.winfo_height())
            self.assertAlmostEqual(displayed.width() / displayed.height(), 1600 / 900, delta=0.03)
            self.assertIs(app.preview._source_image, image)
            self.assertEqual(image.tobytes(), original_bytes)

    def test_renderers_do_not_modify_source_frames(self):
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        original = frame.copy()
        for cls in APP_CLASSES:
            self.clear()
            app = fake_app(self.root, cls)
            app._get_font = Mock(return_value=None)
            app._annotate = Mock(side_effect=lambda image, *args, **kwargs: image)
            if cls is gui.PhotoApp:
                app._render(frame, 0.0)
            elif cls is gui.ScanApp:
                app._render_frame(frame)
            else:
                app._render(frame, None)
            self.assertTrue(np.array_equal(frame, original))
            self.assertIsNotNone(app.preview._display_photo)

    def test_sidebar_scroll_reaches_lower_tools_and_spin_defaults_unchanged(self):
        app = fake_app(self.root, gui.GaugeApp)
        self.root.geometry('560x420')
        self.root.update()
        canvas = app.sidebar_canvas
        canvas.yview_moveto(1.0)
        self.root.update()
        model_button = next(w for w in widgets(self.root)
                            if isinstance(w, gui.ttk.Button) and w['text'] == '更换模型 (M)')
        self.assertLessEqual(model_button.winfo_rooty() + model_button.winfo_height(),
                             canvas.winfo_rooty() + canvas.winfo_height())
        self.clear()
        app = fake_app(self.root, gui.DetectApp)
        self.assertEqual(app.conf_spin['from'], 0.05)
        self.assertEqual(app.conf_spin['to'], 0.95)
        self.assertEqual(app.conf_spin['increment'], 0.05)
        self.assertEqual(app.conf_var.get(), '0.25')
        self.clear()
        app = fake_app(self.root, gui.PhotoApp)
        self.assertEqual(int(app.count_spin['from']), 1)
        self.assertEqual(int(app.count_spin['to']), gui.MAX_BURST)

    def test_sidebar_wheel_on_labels_and_buttons_is_local(self):
        app = fake_app(self.root, gui.GaugeApp)
        self.root.geometry('560x420')
        self.root.update()
        canvas = app.sidebar_canvas
        content = canvas.nametowidget(canvas.itemcget(1, 'window'))
        descendants = list(widgets(content))
        label = next(w for w in descendants if isinstance(w, gui.ttk.Label))
        button = next(w for w in descendants if isinstance(w, gui.ttk.Button))
        for target in (label, button):
            canvas.yview_moveto(0.0)
            before = canvas.yview()[0]
            target.event_generate('<Button-5>')
            self.root.update()
            self.assertGreater(canvas.yview()[0], before)
        self.clear()
        app = fake_app(self.root, gui.DetectApp)
        self.assertFalse(any(str(tag).startswith('CaptureSidebar:')
                             for tag in app.conf_spin.bindtags()))
        self.assertFalse(any(str(tag).startswith('CaptureSidebar:')
                             for tag in self.root.bindtags()))

    def test_keyboard_text_focus_guards_remain_effective(self):
        for cls, handler, callback in ((gui.PhotoApp, '_on_space', 'shoot'),
                                       (gui.DetectApp, '_on_key_s', 'save_snapshot'),
                                       (gui.ScanApp, '_on_key_c', 'save_crops'),
                                       (gui.GaugeApp, '_on_key_r', 'save_readings')):
            self.clear()
            app = fake_app(self.root, cls)
            entry = gui.ttk.Entry(self.root)
            entry.pack()
            self.root.update()
            entry.focus_force()
            self.root.update()
            getattr(app, handler)()
            getattr(app, callback).assert_not_called()
            app.preview.focus_force()
            self.root.update()
            getattr(app, handler)()
            getattr(app, callback).assert_called_once_with()

    def test_dialogs_keep_defaults_validation_and_small_screen_reachability(self):
        dialogs = []
        output = Path(self.temp.name)
        (output / '正面').mkdir()
        model = output / 'test.pt'
        model.touch()
        dialogs.append(gui.AskTypeDialog(self.root, output, '侧面', 4))
        self.assertEqual(dialogs[-1].type_var.get(), '侧面')
        self.assertEqual(dialogs[-1].count_var.get(), '4')
        dialogs.append(gui.ModelDialog(self.root, output, model))
        self.assertEqual(dialogs[-1].selected, model)
        scan = gui.ScanDialog(self.root, gui.scan_module.ScanSettings())
        dialogs.append(scan)
        self.assertEqual(scan._collect(False).speed, 0.15)
        scan.mode_var.set('manual')
        self.assertEqual(str(scan.speed_spin['state']), 'disabled')
        self.assertEqual(str(scan.fps_spin['state']), 'normal')
        gauge = gui.GaugeSettingsDialog(self.root, gui.gauge_module.GaugeSettings())
        dialogs.append(gauge)
        self.assertEqual(gauge._arc(), 'long')
        self.assertEqual(gauge._pointer(), 'radon')
        self.assertEqual(gauge.temp_max_var.get(), '100.0')
        for dialog in dialogs:
            self.root.update()
            self.assertLess(dialog.winfo_height(), 820)
            self.assertLess(dialog.winfo_width(), 1000)
            for button in (w for w in widgets(dialog) if isinstance(w, gui.ttk.Button)):
                self.assertTrue(button.winfo_viewable())
                self.assertLessEqual(button.winfo_rooty() + button.winfo_height(),
                                     dialog.winfo_rooty() + dialog.winfo_height())
            dialog.destroy()

    def test_theme_uses_existing_font_discovery_and_handles_missing_font(self):
        with patch.object(gui, 'find_cjk_font', return_value=None) as finder:
            family = gui._setup_gui_theme(self.root)
        finder.assert_called_once_with()
        self.assertIn(family, tkfont.families(self.root))

    def test_mode_dialog_available_and_unavailable_branches_use_same_grid(self):
        for available in (True, False):
            with patch.multiple(gui, YOLO_RUNTIME_AVAILABLE=available,
                                SCAN_MODULE_AVAILABLE=available, GAUGE_MODULE_AVAILABLE=available):
                dialog = gui.ModeDialog(self.root)
                self.root.update()
                self.assertLess(dialog.winfo_width(), 760)
                self.assertLess(dialog.winfo_height(), 700)
                dialog._choose(gui.MODE_DETECT)
                if available:
                    self.assertEqual(dialog.result, gui.MODE_DETECT)
                else:
                    self.assertIsNone(dialog.result)
                    dialog.destroy()


class LifecycleTests(unittest.TestCase):
    def app(self, cls):
        app = cls.__new__(cls)
        app.root = Mock()
        app.camera = Mock()
        app.detector = Mock()
        app._closed = False
        app._after_id = 'fake-after'
        app._rectified_window = None
        app._close_when_done = False
        app._job = None
        app._session = None
        app.chassis = Mock()
        return app

    def test_all_close_paths_cancel_after_release_camera_and_are_idempotent(self):
        for cls in APP_CLASSES:
            with self.subTest(app=cls.__name__):
                app = self.app(cls)
                app.close()
                app.close()
                self.assertTrue(app._closed)
                self.assertIsNone(app._after_id)
                app.root.after_cancel.assert_called_once_with('fake-after')
                app.camera.release.assert_called_once_with()
                app.root.destroy.assert_called_once_with()
                if cls in (gui.DetectApp, gui.GaugeApp):
                    app.detector.close.assert_called_once_with()

    def test_scan_start_stop_and_escape_close_use_only_mock_jobs(self):
        app = self.app(gui.ScanApp)
        app.settings = gui.scan_module.ScanSettings()
        app.output_root = Path('/unused/mock-output')
        app.camera.frame_size = (1280, 720)
        app.stitch_max_width = 1600
        app._detect_thread = None
        app._set_status = Mock()
        for name in ('view_button', 'crop_button', 'detect_button', 'open_button',
                     'start_button', 'stop_button', 'progress', 'progress_var'):
            setattr(app, name, Mock())
        with patch.object(gui.scan_module, 'ScanSession') as session, \
                patch.object(gui.scan_module, 'ScanJob') as job:
            app.start_scan()
            job.return_value.start.assert_called_once_with()
            self.assertIs(app._session, session.return_value)
            app.stop_scan()
            job.return_value.stop.assert_called_once_with()
            # Esc 的原绑定仍调用 close；运行中先请求停止，完成后再清理。
            app._bind_keys()
            escape = next(call.args[1] for call in app.root.bind.call_args_list
                          if call.args[0] == '<Escape>')
            with patch.object(gui.messagebox, 'askyesno', return_value=True):
                escape(None)
            self.assertTrue(app._close_when_done)
            app.camera.release.assert_not_called()
            app._destroy()
            app.camera.release.assert_called_once_with()
            job.return_value.join.assert_called_once_with(2.0)
            app.chassis.close.assert_called_once_with()

    def test_headless_aliases_dispatch_to_cli_without_gui_or_hardware(self):
        defaults = gui.parse_args([])
        self.assertEqual((defaults.width, defaults.height, defaults.sharpness), (1280, 720, 3))
        self.assertEqual(defaults.scan_speed, 0.15)
        self.assertEqual(defaults.pixel_format, 'auto')
        for flag in ('--no-gui', '--headless'):
            with tempfile.TemporaryDirectory() as output, \
                    patch.object(gui, 'run_cli', return_value=37) as cli, \
                    patch.object(gui, 'run_gui') as graphical, \
                    patch.object(gui.cv2, 'VideoCapture') as camera:
                self.assertEqual(gui.main([flag, '--mode', 'photo', '-d', output]), 37)
                self.assertTrue(cli.call_args.args[0].no_gui)
                graphical.assert_not_called()
                camera.assert_not_called()


if __name__ == '__main__':
    unittest.main()
