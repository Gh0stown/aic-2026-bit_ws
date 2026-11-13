#!/usr/bin/env python3
"""Calibration UI regression with real Tk and synthetic frames only.

No Camera, Decoder, serial port or model may be constructed. Tk tests skip when
DISPLAY is unavailable; non-Tk preview helpers and projection remain testable.
"""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import tkinter as tk
import unittest
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import capture_photos as gui


def thermal_frame(offset=0.0):
    temps = np.arange(24 * 32, dtype=np.float32).reshape(24, 32) / 100 + 20 + offset
    return SimpleNamespace(
        temps=temps, bgr=np.zeros((330, 384, 3), dtype=np.uint8),
        hotspot=SimpleNamespace(row=6.25, col=9.5, row_index=6,
                                col_index=9, peak_c=float(temps[6, 9])),
        floor='3F', ambient=22.0, monotonic=123.0 + offset)


def correspondence_points():
    return [[row, col, 10 + 5 * col, 15 + 4 * row]
            for row, col in ((2, 3), (2, 27), (20, 27), (20, 3))]


def fake_session():
    return SimpleNamespace(
        port='/dev/not-a-real-thermal-device', capture='', baud=460800,
        scale=12, tmin=20.0, tmax=35.0, delta_c=2.0, roi=None,
        replay_rate=2.0, loop=True,
        floor_table=SimpleNamespace(to_text=lambda: '3F:0-11,1F:12-23'),
        start=Mock(), stop=Mock(), latest=Mock(return_value=None),
        error=Mock(return_value=None))


class CalibrationHelperTests(unittest.TestCase):
    def test_clean_ir_has_native_data_extent_without_caption_and_preserves_input(self):
        frame = thermal_frame()
        before_temps, before_bgr = frame.temps.copy(), frame.bgr.copy()
        image = gui.calibration_ir_image(frame, 12)
        self.assertEqual(image.shape, (24 * 12, 32 * 12, 3))
        np.testing.assert_array_equal(image[:12, :12],
                                      np.broadcast_to(image[0, 0], (12, 12, 3)))
        np.testing.assert_array_equal(frame.temps, before_temps)
        np.testing.assert_array_equal(frame.bgr, before_bgr)

    def test_preview_inverse_rejects_empty_image_and_letterbox(self):
        preview = SimpleNamespace(_source_image=None, _display_photo=None)
        self.assertIsNone(gui.calibration_preview_point(preview, 0, 0))
        preview._source_image = Image.new('RGB', (1280, 720))
        preview._display_photo = SimpleNamespace(width=lambda: 640, height=lambda: 360)
        preview.winfo_width, preview.winfo_height = lambda: 800, lambda: 600
        self.assertEqual(gui.calibration_preview_point(preview, 400, 300), (640, 360))
        for point in ((79, 300), (720, 300), (400, 119), (400, 480)):
            with self.subTest(point=point):
                self.assertIsNone(gui.calibration_preview_point(preview, *point))

    def test_runtime_rgb_projection_is_display_only_and_keeps_ir_floor(self):
        app = gui.ThermalApp.__new__(gui.ThermalApp)
        frame = thermal_frame()
        rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        app._shown, app._rgb_bgr = frame, rgb
        app._calibration = gui.calibration_math.fit_calibration(correspondence_points(), (320, 240))
        temps, bgr, original_rgb = frame.temps.copy(), frame.bgr.copy(), rgb.copy()
        projected = app._project_rgb()
        self.assertIsNot(projected, rgb)
        self.assertTrue(np.any(projected != rgb))
        np.testing.assert_array_equal(rgb, original_rgb)
        np.testing.assert_array_equal(frame.temps, temps)
        np.testing.assert_array_equal(frame.bgr, bgr)
        self.assertEqual(frame.floor, '3F')


class CalibrationWindowTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest('Tk display unavailable: %s' % exc)
        self.root.withdraw()
        self.addCleanup(self.cleanup_root)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.camera_guard = patch.object(gui.cv2, 'VideoCapture',
                                         side_effect=AssertionError('camera hardware forbidden'))
        self.video_capture = self.camera_guard.start()
        self.addCleanup(self.camera_guard.stop)
        self.session_guard = patch.object(gui, 'ThermalSession',
                                          side_effect=AssertionError('real decoder forbidden'))
        self.session_constructor = self.session_guard.start()
        self.addCleanup(self.session_guard.stop)
        self.session = fake_session()
        self.camera = Mock(name='shared_camera')
        with patch.object(gui.ThermalApp, '_tick'):
            self.app = gui.ThermalApp(self.root, self.session, self.temp.name,
                                      camera=self.camera)
        self.frame = thermal_frame()
        self.rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        self.app._shown, self.app._thermal_bgr = self.frame, self.frame.bgr
        self.app._rgb_bgr = self.rgb
        self.app.open_calibration()
        self.window = self.app.calibration_window
        self.window.root.geometry('900x650')
        self.root.update()

    def cleanup_root(self):
        try:
            for job in self.root.tk.call('after', 'info'):
                self.root.after_cancel(job)
            self.root.destroy()
        except tk.TclError:
            pass

    def event_at(self, preview, x, y):
        """Convert source-image pixels to a widget-local mouse event."""
        photo, source = preview._display_photo, preview._source_image
        left = (preview.winfo_width() - photo.width()) / 2
        top = (preview.winfo_height() - photo.height()) / 2
        return SimpleNamespace(x=left + x * photo.width() / source.width,
                               y=top + y * photo.height() / source.height)

    def set_points(self, points=None):
        window = self.window
        window.points = correspondence_points() if points is None else points
        window.temperatures = [25.0] * len(window.points)
        window.snapshots = [None] * len(window.points)
        window._points_rgb_size = (320, 240)
        window._refresh_table()

    def test_reuses_existing_preview_and_never_opens_another_stream(self):
        self.assertIsInstance(self.window.ir_preview, gui._CapturePreview)
        self.assertIsInstance(self.window.rgb_preview, gui._CapturePreview)
        self.app.open_calibration()
        self.assertIs(self.app.calibration_window, self.window)
        self.video_capture.assert_not_called()
        self.session_constructor.assert_not_called()
        self.camera.read.assert_not_called()
        self.session.start.assert_not_called()

    def test_real_preview_letterbox_inverse_survives_window_resize(self):
        for size in ('900x650', '1150x790', '820x620'):
            with self.subTest(size=size):
                self.window.root.geometry(size)
                self.root.update()
                preview = self.window.ir_preview
                event = self.event_at(preview, 123.5, 87.5)
                point = gui.calibration_preview_point(preview, event.x, event.y)
                np.testing.assert_allclose(point, (123.5, 87.5), atol=1e-8)
                self.assertEqual(preview._source_image.size, (384, 288))
                self.assertLessEqual(preview._display_photo.width(), preview.winfo_width())
                self.assertLessEqual(preview._display_photo.height(), preview.winfo_height())
                self.assertIsNone(gui.calibration_preview_point(preview, 0, 0))
                for button in self.window.buttons.values():
                    x = button.winfo_rootx() - self.window.root.winfo_rootx()
                    y = button.winfo_rooty() - self.window.root.winfo_rooty()
                    self.assertGreaterEqual(x, 0)
                    self.assertGreaterEqual(y, 0)
                    self.assertLessEqual(x + button.winfo_width(), self.window.root.winfo_width())
                    self.assertLessEqual(y + button.winfo_height(), self.window.root.winfo_height())

    def test_ir_last_pixel_is_valid_and_letterbox_is_not_a_caption_point(self):
        preview = self.window.ir_preview
        self.window._click_ir(self.event_at(preview, 31.5 * 12, 23.5 * 12))
        self.assertEqual(self.window._manual_ir, (23.0, 31.0))
        self.window.next_point()
        self.window._click_ir(SimpleNamespace(x=0, y=0))
        self.assertIsNone(self.window._manual_ir)
        self.assertIsNone(self.window._frozen)

    def test_freeze_selects_same_pair_when_live_frames_continue_and_records_row_col_temperature(self):
        window = self.window
        window._click_ir(self.event_at(window.ir_preview, 7.5 * 12, 5.5 * 12))
        frozen_rgb = window._frozen[1].copy()
        replacement = thermal_frame(10)
        new_rgb = np.full_like(self.rgb, 190)
        window.update_live(replacement, new_rgb, self.app._source_generation)
        window._click_rgb(self.event_at(window.rgb_preview, 140.25, 90.25))
        window.capture_point()
        self.assertEqual(window.points, [[5.0, 7.0, 140.0, 90.0]])
        self.assertEqual(window.temperatures, [float(self.frame.temps[5, 7])])
        self.assertIs(window.snapshots[0][0], self.frame)
        np.testing.assert_array_equal(window.snapshots[0][1], frozen_rgb)
        self.assertEqual(len(window.table.get_children()), 1)
        window.next_point()
        self.assertIs(window._pair()[0], replacement)
        self.assertIsNone(window._manual_ir)
        self.assertIsNone(window._rgb_point)

    def test_auto_hotspot_preserves_subpixel_coordinates_and_overrides_manual_point(self):
        window = self.window
        window._click_ir(self.event_at(window.ir_preview, 2.5 * 12, 3.5 * 12))
        window.auto_hotspot()
        window._click_rgb(self.event_at(window.rgb_preview, 100.25, 80.25))
        window.capture_point()
        self.assertEqual(window.points[0], [6.25, 9.5, 100.0, 80.0])
        self.assertEqual(window.temperatures[0], self.frame.hotspot.peak_c)

    def test_missing_rgb_and_duplicate_correspondence_do_not_add_rows(self):
        window = self.window
        window.capture_point()
        self.assertEqual(window.points, [])
        self.assertIn('RGB', window.status_var.get())
        window._rgb_point = (100.0, 80.0)
        window.capture_point()
        window.capture_point()
        self.assertEqual(len(window.points), 1)
        self.assertIn('重复', window.status_var.get())

    def test_compute_button_gate_and_collinear_failure_invalidate_h(self):
        self.assertEqual(str(self.window.buttons['计算 H']['state']), 'disabled')
        self.set_points(correspondence_points()[:3])
        self.assertEqual(str(self.window.buttons['计算 H']['state']), 'disabled')
        self.window.compute()
        self.assertIsNone(self.window.calibration)
        self.assertIn('计算失败', self.window.status_var.get())
        self.set_points()
        self.window.compute()
        self.assertIsNotNone(self.window.calibration)
        self.assertLess(self.window.calibration.rms_px, 1e-5)
        self.set_points([[i, i, 10 + i * 10, 20 + i * 10] for i in range(4)])
        self.window.compute()
        self.assertIsNone(self.window.calibration)
        self.assertIsNone(self.app._calibration)
        self.assertIn('计算失败', self.window.status_var.get())

    def test_delete_and_clear_invalidate_parent_and_disable_save_verify(self):
        self.set_points()
        self.window.compute()
        self.window.delete_last()
        self.assertEqual(len(self.window.points), 3)
        self.assertIsNone(self.window.calibration)
        self.assertIsNone(self.app._calibration)
        for text in ('保存', '验证', '计算 H'):
            self.assertEqual(str(self.window.buttons[text]['state']), 'disabled')
        self.window.clear_points()
        self.assertEqual(self.window.points, [])
        self.assertEqual(self.window.temperatures, [])
        self.assertEqual(self.window.snapshots, [])
        self.assertEqual(self.window.table.get_children(), ())
        self.assertIsNone(self.window._points_rgb_size)

    def test_table_review_uses_recorded_pair_and_blocks_accidental_collection(self):
        window = self.window
        window._click_rgb(self.event_at(window.rgb_preview, 100.25, 80.25))
        window.capture_point()
        window.next_point()
        window.update_live(thermal_frame(10), np.full_like(self.rgb, 100),
                           self.app._source_generation)
        window.table.selection_set('0')
        window._review_point()
        self.assertTrue(window._review)
        self.assertIs(window._pair()[0], self.frame)
        window.capture_point()
        self.assertEqual(len(window.points), 1)
        self.assertIn('回看', window.status_var.get())

    def test_save_load_valid_schema_and_reject_current_session_mismatch(self):
        self.set_points()
        self.window.note_var.set('synthetic GUI fixture')
        self.window.compute()
        self.window.save()
        path = self.app.calibration_path
        self.assertTrue(path.exists())
        payload = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(payload['version'], 1)
        self.assertEqual(payload['ir_size'], [32, 24])
        self.assertEqual(payload['rgb_size'], [320, 240])
        self.assertEqual(payload['mirror_x'], True)
        self.assertEqual(payload['note'], 'synthetic GUI fixture')
        self.window.clear_points()
        self.window.load()
        self.assertEqual(len(self.window.points), 4)
        self.assertIsNotNone(self.app._calibration)
        self.assertEqual(self.window.temperatures, [None] * 4)
        self.window.clear_points()
        self.app._rgb_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        self.window.load()
        self.assertIsNone(self.app._calibration)
        self.assertEqual(self.window.points, [])
        self.assertIn('mismatch', self.window.status_var.get())

    def test_gui_load_rejects_wrong_orientation_ir_size_and_missing_h(self):
        self.set_points()
        self.window.compute()
        self.window.save()
        path = self.app.calibration_path
        original = json.loads(path.read_text(encoding='utf-8'))
        for change in ('mirror_x', 'ir_size', 'H'):
            with self.subTest(change=change):
                self.window.clear_points()
                payload = dict(original)
                if change == 'H':
                    del payload['H']
                elif change == 'ir_size':
                    payload['ir_size'] = [24, 32]
                else:
                    payload['mirror_x'] = False
                path.write_text(json.dumps(payload), encoding='utf-8')
                self.window.load()
                self.assertIsNone(self.window.calibration)
                self.assertIsNone(self.app._calibration)
                self.assertEqual(self.window.points, [])
                self.assertIn('加载失败', self.window.status_var.get())
                self.assertEqual(str(self.window.buttons['验证']['state']), 'disabled')

    def test_verify_and_overlays_never_modify_raw_input(self):
        self.set_points()
        self.window.compute()
        before_rgb, before_temps, before_bgr = self.rgb.copy(), self.frame.temps.copy(), self.frame.bgr.copy()
        self.window.verify()
        np.testing.assert_allclose(self.window._validation_uv, (57.5, 40.0), atol=1e-6)
        np.testing.assert_array_equal(self.rgb, before_rgb)
        np.testing.assert_array_equal(self.frame.temps, before_temps)
        np.testing.assert_array_equal(self.frame.bgr, before_bgr)
        self.assertEqual(self.frame.floor, '3F')

    def test_source_generation_change_discards_frozen_selection(self):
        self.window._click_rgb(self.event_at(self.window.rgb_preview, 100.25, 80.25))
        self.assertIsNotNone(self.window._frozen)
        self.window.update_live(None, self.rgb, self.app._source_generation + 1)
        self.assertIsNone(self.window._frozen)
        self.assertIsNone(self.window._manual_ir)
        self.assertIsNone(self.window._rgb_point)
        self.assertIsNone(self.window.ir_preview._source_image)

    def test_invalid_roi_never_stops_or_constructs_a_session(self):
        self.assertFalse(self.app.apply_calibration_roi('0,0,25,32'))
        self.session.stop.assert_not_called()
        self.session_constructor.assert_not_called()
        self.assertIs(self.app.session, self.session)

    def test_roi_rebuild_stops_before_construct_start_and_clears_frozen_old_source(self):
        self.window._click_rgb(self.event_at(self.window.rgb_preview, 100.25, 80.25))
        order = []
        self.session.stop.side_effect = lambda: order.append('stop-old')
        replacement = fake_session()
        replacement.roi = (3, 4, 22, 30)
        replacement.start.side_effect = lambda: order.append('start-new')

        def construct(**options):
            order.append('construct-new')
            self.assertEqual(options['roi'], (3, 4, 22, 30))
            self.assertEqual(options['port'], self.session.port)
            self.assertIsNone(options['capture'])
            self.assertEqual(options['floor_table'], self.session.floor_table.to_text())
            self.assertTrue(options['mirror_x'])
            return replacement

        self.session_constructor.side_effect = construct
        self.assertTrue(self.app.apply_calibration_roi('3,4,22,30'))
        self.assertEqual(order, ['stop-old', 'construct-new', 'start-new'])
        self.assertIs(self.app.session, replacement)
        self.assertEqual(self.app.roi_var.get(), '3,4,22,30')
        self.assertIsNone(self.app._shown)
        self.assertIsNone(self.app._thermal_bgr)
        self.assertIsNone(self.window._frozen)
        self.assertIsNone(self.window._rgb_point)

    def test_changed_live_rgb_size_rejects_frozen_capture_compute_and_verify(self):
        self.set_points()
        self.window.compute()
        calibration = self.window.calibration
        self.window._freeze()
        self.window._rgb_point = (100.0, 80.0)
        self.app._rgb_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        original_points = [list(point) for point in self.window.points]
        for action in ('capture_point', 'compute', 'verify'):
            with self.subTest(action=action):
                self.window.calibration = calibration
                self.app._calibration = calibration
                self.window._validation_uv = None
                getattr(self.window, action)()
                self.assertEqual(self.window.points, original_points)
                self.assertIsNone(self.window._validation_uv)
                self.assertIn('尺寸或镜像已改变', self.window.status_var.get())
                if action == 'compute':
                    self.assertIsNone(self.window.calibration)
                    self.assertIsNone(self.app._calibration)

    def test_saving_verified_loaded_h_preserves_matrix_and_created_without_refitting(self):
        self.set_points()
        self.window.compute()
        self.window.save()
        path = self.app.calibration_path
        payload = json.loads(path.read_text(encoding='utf-8'))
        # A valid loaded matrix need not be the least-squares fit of its points.
        payload['H'][0][2] += 1.25
        payload['created'] = '2025-12-01T08:30:00+08:00'
        path.write_text(json.dumps(payload), encoding='utf-8')
        self.window.load()
        loaded = self.window.calibration
        self.assertIsNotNone(loaded)
        self.window.verify()
        self.assertIsNotNone(self.window._validation_uv)
        self.window.note_var.set('verified loaded matrix')
        with patch.object(gui.calibration_math, 'fit_calibration',
                          side_effect=AssertionError('must save the verified H')) as fit:
            self.window.save()
            fit.assert_not_called()
        saved = json.loads(path.read_text(encoding='utf-8'))
        np.testing.assert_allclose(saved['H'], loaded.H, atol=1e-10)
        self.assertEqual(saved['created'], payload['created'])
        self.assertEqual(saved['note'], 'verified loaded matrix')
        self.assertIn('已保存', self.window.status_var.get())

    def test_twelve_rgb_read_failures_clear_cached_rgb_and_block_new_points(self):
        self.session.latest.return_value = self.frame
        self.camera.read.return_value = None
        self.window.next_point()
        with patch.object(gui.time, 'monotonic', return_value=124.0):
            for attempt in range(12):
                self.app._tick()
                after_id = self.app._after_id
                self.assertIsNotNone(after_id)
                self.root.after_cancel(after_id)
                self.app._after_id = None
                self.assertNotIn(after_id, self.root.tk.call('after', 'info'))
                if attempt < 11:
                    self.assertIs(self.app._rgb_bgr, self.rgb)
        self.assertEqual(self.camera.read.call_count, 12)
        self.camera.release.assert_called_once_with()
        self.assertIsNone(self.app.camera)
        self.assertIsNone(self.app._rgb_bgr)
        self.assertIsNone(self.window._live_rgb)
        self.window.capture_point()
        self.assertEqual(self.window.points, [])
        self.assertIn('有效画面', self.window.status_var.get())
        self.session_constructor.assert_not_called()
        self.video_capture.assert_not_called()

    def test_stale_ir_tick_clears_ir_caches_and_suspends_projection(self):
        self.set_points()
        self.window.compute()
        self.window.clear_points()
        self.app._calibration = gui.calibration_math.fit_calibration(
            correspondence_points(), (320, 240))
        self.session.latest.return_value = self.frame
        self.camera.read.return_value = self.rgb
        self.window._freeze()
        with patch.object(gui.time, 'monotonic', return_value=self.frame.monotonic + 3.01):
            self.app._tick()
        after_id = self.app._after_id
        self.assertIsNotNone(after_id)
        self.root.after_cancel(after_id)
        self.app._after_id = None
        self.assertNotIn(after_id, self.root.tk.call('after', 'info'))
        self.assertIsNone(self.app._shown)
        self.assertIsNone(self.app._thermal_bgr)
        self.assertIsNone(self.window._live_frame)
        self.assertIs(self.app._project_rgb(), self.rgb)
        self.assertIn('超过 3 秒', self.app.status_var.get())
        self.window.capture_point()
        self.assertEqual(self.window.points, [])
        self.assertIn('等待 IR', self.window.status_var.get())
        self.session_constructor.assert_not_called()
        self.video_capture.assert_not_called()

    def test_child_close_is_idempotent_and_does_not_stop_shared_devices(self):
        self.window.close()
        self.window.close()
        self.assertIsNone(self.app.calibration_window)
        self.session.stop.assert_not_called()
        self.camera.release.assert_not_called()
        self.assertFalse(self.app._closed)

    def test_parent_close_closes_child_and_stops_shared_devices_once(self):
        self.app.close()
        self.app.close()
        self.assertTrue(self.window._closed)
        self.assertTrue(self.app._closed)
        self.session.stop.assert_called_once_with()
        self.camera.release.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
