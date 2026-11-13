#!/usr/bin/env python3
"""Real-Tk window-template regressions using synthetic RGB and a fake worker.

No camera, serial transport, thermal decoder, or YOLO model may be opened.
Tk cases explicitly skip if a display is unavailable.
"""
import importlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import tkinter as tk
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import capture_photos as gui


CORNERS = np.array(((20, 20), (300, 20), (300, 220), (20, 220)), dtype=float)
CENTERS = {'1F': (160, 195), '2F': (160, 145),
           '3F': (160, 95), '4F': (160, 45)}


def detection():
    return gui.building_windows.BuildingDetection(
        CORNERS.copy(), .95, (20, 20, 300, 220), (.95,) * 4)


class FakeWorker:
    """Synchronous completion by default; pending/error modes test Tk polling."""
    def __init__(self):
        self.result = None
        self.detections = [detection()]
        self.complete = True
        self.last_error = ''
        self.submit = Mock(side_effect=self._submit)
        self.close = Mock()

    def _submit(self, frame, request_id):
        if self.complete:
            self.result = request_id, self.detections

    def latest(self):
        return self.result


class WindowCentersGuiTests(unittest.TestCase):
    def setUp(self):
        self.guards = []
        forbidden = AssertionError('real hardware/model construction forbidden')
        for owner, name in ((gui.cv2, 'VideoCapture'), (gui, 'Camera'),
                            (gui, 'ThermalSession'),
                            (gui.building_windows, 'YoloDetector'),
                            (gui.yolo_detect, 'YoloDetector')):
            if owner is not None:
                self.guard(owner, name, side_effect=forbidden)
        # Patch the imported decoder and transport aliases as well as their
        # definitions. Importing these modules does not construct devices.
        for module_name, names in (
                ('mlx90640_ros.core', ('Decoder',)),
                ('mlx90640_ros.thermal_session', ('ThermalSession', 'SerialTransport', 'ReplayTransport')),
                ('mlx90640_ros.transport', ('SerialTransport', 'ReplayTransport')),
                ('serial', ('Serial', 'serial_for_url'))):
            try:
                module = importlib.import_module(module_name)
            except ImportError:
                continue
            for name in names:
                if hasattr(module, name):
                    self.guard(module, name, side_effect=forbidden)
        # Block a lazy `from ultralytics import YOLO`, without importing the
        # heavyweight real runtime even if it is installed.
        self.guard_modules = patch.dict(sys.modules, {
            'ultralytics': SimpleNamespace(YOLO=Mock(side_effect=forbidden))})
        self.guard_modules.start()
        self.addCleanup(self.guard_modules.stop)
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest('Tk display unavailable: %s' % exc)
        self.root.withdraw()
        self.addCleanup(self.cleanup_root)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.session = SimpleNamespace(
            roi=None, floor_table=SimpleNamespace(to_text=lambda: '4F:0-5,3F:6-11,2F:12-17,1F:18-23'),
            start=Mock(), stop=Mock(), latest=Mock(return_value=None), error=Mock(return_value=None))
        self.camera = Mock(name='shared_camera', mirror=False)
        self.worker = FakeWorker()
        self.factory = self.guard(gui.building_windows, 'BuildingPoseDetector', return_value=self.worker)
        with patch.object(gui.ThermalApp, '_tick'):
            self.app = gui.ThermalApp(self.root, self.session, self.temp.name, camera=self.camera)
        self.app._building_model_path = Path(self.temp.name) / 'building_d.pt'
        self.app._building_model_path.write_bytes(b'fake weights for fingerprint only; never load')
        self.rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        self.rgb[:, :, 0] = np.arange(320, dtype=np.uint16) % 256
        self.app._rgb_bgr = self.rgb
        self.app.open_window_centers()
        self.window = self.app.window_centers_window
        self.window.root.geometry('900x700')
        self.root.update()
        self.addCleanup(self.assert_no_hardware)

    def guard(self, owner, name, **kwargs):
        guard = patch.object(owner, name, **kwargs)
        result = guard.start()
        self.addCleanup(guard.stop)
        if name != 'BuildingPoseDetector':
            self.guards.append(result)
        return result

    def assert_no_hardware(self):
        for constructor in self.guards:
            constructor.assert_not_called()
        self.camera.read.assert_not_called()
        self.session.start.assert_not_called()

    def cleanup_root(self):
        try:
            for job in self.root.tk.call('after', 'info'):
                self.root.after_cancel(job)
            self.root.destroy()
        except tk.TclError:
            pass

    def event_at(self, u, v):
        preview = self.window.preview
        photo, source = preview._display_photo, preview._source_image
        return SimpleNamespace(
            x=(preview.winfo_width() - photo.width()) / 2 + u * photo.width() / source.width,
            y=(preview.winfo_height() - photo.height()) / 2 + v * photo.height() / source.height)

    def freeze(self):
        self.window.freeze_and_detect()
        self.root.update()
        np.testing.assert_array_equal(self.window._corners, CORNERS)

    def four_centers(self):
        for floor, (u, v) in CENTERS.items():
            self.assertEqual(self.window.floor_var.get(), floor)
            self.window._click(self.event_at(u + .25, v + .25))
        self.assertEqual(self.window.centers, CENTERS)

    def template(self):
        return gui.building_windows.make_template(CORNERS, CENTERS, (320, 240),
            model_path=str(self.app._building_model_path))

    def pending_poll(self):
        self.worker.complete = False
        self.window.freeze_and_detect()
        self.assertIsNotNone(self.window._after_id)
        after_id = self.window._after_id
        self.window.root.after_cancel(after_id)
        self.window._after_id = None
        return self.window._request_id

    def runtime_pose(self, timestamp=100.0):
        self.app._building_detector = self.worker
        self.app._window_template = self.template()
        self.app._window_template_checked = True
        self.app._building_pose = dict(time=timestamp, size=(320, 240), mirror=False,
                                       detection=detection())
        self.app._shown = SimpleNamespace(hotspot=SimpleNamespace(col=160.0, row=195.0), floor='IR-original')
        self.app._calibration = SimpleNamespace(H=np.eye(3))

    def test_singleton_preview_and_no_device_or_model_load_before_freeze(self):
        self.assertIsInstance(self.window.preview, gui._CapturePreview)
        self.app.open_window_centers()
        self.assertIs(self.app.window_centers_window, self.window)
        self.factory.assert_not_called()
        self.assertIsNone(self.app._shown)  # RGB annotation does not require IR.
        self.freeze()
        self.factory.assert_called_once_with(str(self.app._building_model_path))
        self.worker.submit.assert_called_once()

    def test_four_points_follow_floor_order_and_explicit_floor_edit(self):
        self.freeze()
        self.assertEqual(str(self.window.buttons['保存四窗模板']['state']), 'disabled')
        self.four_centers()
        self.assertEqual(str(self.window.buttons['保存四窗模板']['state']), 'normal')
        self.window.table.selection_set('2F')
        self.window._select_floor()
        self.assertEqual(self.window.floor_var.get(), '2F')
        self.window._click(self.event_at(170.25, 145.25))
        self.assertEqual(len(self.window.centers), 4)
        self.assertEqual(self.window.centers['2F'], (170, 145))
        self.assertEqual(self.window.centers['1F'], CENTERS['1F'])

    def test_letterbox_rejected_and_inverse_preserved_across_resize(self):
        self.freeze()
        for size in ('900x700', '1050x780', '740x600'):
            with self.subTest(size=size):
                self.window.root.geometry(size)
                self.root.update()
                self.window.clear()
                self.window._click(SimpleNamespace(x=0, y=0))
                self.assertEqual(self.window.centers, {})
                event = self.event_at(160.25, 195.25)
                np.testing.assert_allclose(gui.calibration_preview_point(
                    self.window.preview, event.x, event.y), (160.25, 195.25))
                self.window._click(event)
                self.assertEqual(self.window.centers, {'1F': (160, 195)})
                for button in self.window.buttons.values():
                    self.assertLessEqual(button.winfo_rootx() + button.winfo_width(),
                                         self.window.root.winfo_rootx() + self.window.root.winfo_width())

    def test_snapshot_and_input_remain_unchanged_while_live_rgb_advances(self):
        before = self.rgb.copy()
        self.freeze()
        self.assertIsNot(self.window._snapshot, self.rgb)
        submitted = self.worker.submit.call_args.args[0]
        self.assertIsNot(submitted, self.window._snapshot)
        self.app._rgb_bgr = np.full_like(self.rgb, 190)
        self.four_centers()
        self.window._refresh()
        np.testing.assert_array_equal(self.window._snapshot, before)
        np.testing.assert_array_equal(self.rgb, before)
        np.testing.assert_array_equal(submitted, before)

    def test_clear_preserves_file_and_frozen_image_and_resets_floor(self):
        self.freeze()
        self.four_centers()
        self.window.save()
        saved = self.app.window_template_path.read_bytes()
        snapshot = self.window._snapshot.copy()
        self.window.clear()
        self.assertEqual(self.window.centers, {})
        self.assertEqual(self.window.floor_var.get(), '1F')
        self.assertEqual(str(self.window.buttons['保存四窗模板']['state']), 'disabled')
        np.testing.assert_array_equal(self.window._snapshot, snapshot)
        self.assertEqual(self.app.window_template_path.read_bytes(), saved)

    def test_save_load_standalone_json_preserves_original_h_file_and_ir_state(self):
        self.app.calibration_path.parent.mkdir(parents=True)
        sentinel = b'{"sentinel": "existing IR homography must remain byte-identical"}'
        self.app.calibration_path.write_bytes(sentinel)
        calibration = self.app._calibration = object()
        shown = self.app._shown = SimpleNamespace(floor='original-IR-floor')
        self.freeze()
        self.four_centers()
        with patch.object(self.app, '_set_calibration', side_effect=AssertionError('H must not change')):
            self.window.save()
            self.assertTrue(self.app.window_template_path.exists(), self.window.status_var.get())
            payload = json.loads(self.app.window_template_path.read_text(encoding='utf-8'))
            self.assertEqual(payload['floor_order'], ['1F', '2F', '3F', '4F'])
            self.assertEqual(payload['corner_order'], ['TL', 'TR', 'BR', 'BL'])
            self.assertEqual(payload['rgb_size'], [320, 240])
            self.assertEqual(payload['model_filename'], 'building_d.pt')
            self.assertEqual(len(payload['model_sha256']), 64)
            self.assertNotIn('H', payload)
            self.app._window_template = None
            self.window.load()
        self.assertIsNotNone(self.app._window_template)
        self.assertEqual(self.app._window_template.reference_centers, CENTERS)
        self.assertIs(self.app._calibration, calibration)
        self.assertIs(self.app._shown, shown)
        self.assertEqual(shown.floor, 'original-IR-floor')
        self.assertEqual(self.app.calibration_path.read_bytes(), sentinel)

    def test_incomplete_wrong_floor_order_and_off_facade_cannot_save(self):
        self.freeze()
        self.window._click(self.event_at(5.25, 5.25))
        self.assertEqual(self.window.centers, {})
        self.window.save()
        self.assertFalse(self.app.window_template_path.exists())
        self.four_centers()
        self.window.centers['1F'], self.window.centers['4F'] = CENTERS['4F'], CENTERS['1F']
        self.window.save()
        self.assertFalse(self.app.window_template_path.exists())
        self.assertIn('保存失败', self.window.status_var.get())

    def test_rgb_size_or_mirror_change_blocks_edit_and_save(self):
        self.freeze()
        self.four_centers()
        original = self.window.centers.copy()
        for kind in ('size', 'mirror'):
            with self.subTest(kind=kind):
                self.app._rgb_bgr = np.zeros((480, 640, 3), dtype=np.uint8) if kind == 'size' else self.rgb
                self.camera.mirror = kind == 'mirror'
                self.window._click(self.event_at(170.25, 45.25))
                self.assertEqual(self.window.centers, original)
                self.window.save()
                self.assertFalse(self.app.window_template_path.exists())
                self.assertIn('尺寸或镜像', self.window.status_var.get())

    def test_load_rejects_wrong_size_mirror_and_changed_weights(self):
        self.freeze()
        self.four_centers()
        self.window.save()
        for kind in ('size', 'mirror', 'weights'):
            with self.subTest(kind=kind):
                self.app._window_template = None
                self.app._rgb_bgr = np.zeros((480, 640, 3), dtype=np.uint8) if kind == 'size' else self.rgb
                self.camera.mirror = kind == 'mirror'
                if kind == 'weights':
                    self.app._building_model_path.write_bytes(b'different fake model')
                self.window.load()
                self.assertIsNone(self.app._window_template)
                self.assertIn('加载失败', self.window.status_var.get())

    def test_missing_rgb_and_missing_weights_fail_without_constructing_worker(self):
        self.app._rgb_bgr = None
        self.window.freeze_and_detect()
        self.assertIn('白光', self.window.status_var.get())
        self.app._rgb_bgr = self.rgb
        self.app._building_model_path = Path(self.temp.name) / 'missing.pt'
        self.window.freeze_and_detect()
        self.assertIn('找不到', self.window.status_var.get())
        self.factory.assert_not_called()
        self.assertIsNone(self.window._request_id)

    def test_model_constructor_failure_is_reported_and_button_reenabled(self):
        self.factory.side_effect = RuntimeError('synthetic model load failure')
        self.window.freeze_and_detect()
        self.assertIn('synthetic model load failure', self.window.status_var.get())
        self.assertEqual(str(self.window.buttons['冻结 RGB / 识别四角']['state']), 'normal')
        self.assertIsNone(self.window._request_id)

    def test_zero_and_multiple_buildings_refuse_selection(self):
        for detections in ([], [detection(), detection()]):
            with self.subTest(count=len(detections)):
                self.worker.detections = detections
                self.window.freeze_and_detect()
                self.assertIsNone(self.window._corners)
                self.assertIsNone(self.window._request_id)
                self.window._click(SimpleNamespace(x=100, y=100))
                self.assertEqual(self.window.centers, {})
                self.assertEqual(str(self.window.buttons['保存四窗模板']['state']), 'disabled')

    def test_worker_error_timeout_and_other_request_are_handled(self):
        request = self.pending_poll()
        self.worker.result = request + 999, [detection()]
        self.window._poll()
        self.assertEqual(self.window._request_id, request)
        self.assertIsNone(self.window._corners)
        self.window.root.after_cancel(self.window._after_id)
        self.window._after_id = None
        self.worker.last_error = 'synthetic inference failed'
        self.window._poll()
        self.assertIsNone(self.window._request_id)
        self.assertIn('synthetic inference failed', self.window.status_var.get())
        self.worker.last_error = ''
        self.worker.result = None
        self.pending_poll()
        with patch.object(gui.time, 'monotonic', return_value=self.window._deadline + .01):
            self.window._poll()
        self.assertIsNone(self.window._request_id)
        self.assertIn('超时', self.window.status_var.get())

    def test_async_result_rejects_changed_rgb_context(self):
        request = self.pending_poll()
        self.camera.mirror = True
        self.worker.result = request, [detection()]
        self.window._poll()
        self.assertIsNone(self.window._corners)
        self.assertIn('尺寸或镜像', self.window.status_var.get())

    def test_load_displays_centers_on_current_pose_and_new_freeze(self):
        self.freeze()
        self.four_centers()
        self.window.save()
        self.window.clear()
        self.window.load()
        for floor, point in CENTERS.items():
            np.testing.assert_allclose(self.window.centers[floor], point, atol=1e-6)
        self.window.close()
        self.app.open_window_centers()
        self.window = self.app.window_centers_window
        self.window.root.geometry('900x700')
        self.root.update()
        self.window.load()
        self.assertEqual(self.window.centers, {})
        self.assertIsNone(self.window._snapshot)
        self.freeze()
        for floor, point in CENTERS.items():
            np.testing.assert_allclose(self.window.centers[floor], point, atol=1e-6)

    def test_out_of_frame_corners_are_rejected(self):
        self.worker.detections = [SimpleNamespace(corners=CORNERS + (100, 0))]
        self.window.freeze_and_detect()
        self.assertIsNone(self.window._corners)
        self.assertIn('画面内', self.window.status_var.get())
        self.assertEqual(str(self.window.buttons['保存四窗模板']['state']), 'disabled')

    def test_async_rgb_disappearing_is_reported_not_an_uncaught_tk_error(self):
        request = self.pending_poll()
        self.app._rgb_bgr = None
        self.worker.result = request, [detection()]
        self.window._poll()
        self.assertIsNone(self.window._corners)
        self.assertIn('白光', self.window.status_var.get())

    def test_child_close_cancels_poll_but_does_not_close_shared_devices_or_worker(self):
        self.pending_poll()
        self.window._poll()
        job = self.window._after_id
        self.assertIsNotNone(job)
        self.window.close()
        self.window.close()
        self.assertIsNone(self.app.window_centers_window)
        self.assertIsNone(self.window._after_id)
        self.assertNotIn(job, self.root.tk.call('after', 'info'))
        self.worker.close.assert_not_called()
        self.session.stop.assert_not_called()
        self.camera.release.assert_not_called()
        self.assertFalse(self.app._closed)

    def test_parent_close_cancels_poll_and_closes_shared_worker_exactly_once(self):
        self.pending_poll()
        self.app.close()
        self.app.close()
        self.assertTrue(self.window._closed)
        self.assertIsNone(self.window._after_id)
        self.worker.close.assert_called_once_with()
        self.session.stop.assert_called_once_with()
        self.camera.release.assert_called_once_with()

    def test_runtime_submissions_pause_while_editor_is_open(self):
        self.freeze()
        submitted = self.worker.submit.call_count
        with patch.object(gui.time, 'monotonic', return_value=self.app._building_last_submit + .5):
            self.app._update_building_tracking()
        self.assertEqual(self.worker.submit.call_count, submitted)
        self.window.close()
        with patch.object(gui.time, 'monotonic', return_value=self.app._building_last_submit + .5):
            self.app._update_building_tracking()
        self.assertEqual(self.worker.submit.call_count, submitted + 1)

    def test_runtime_candidates_are_geometric_and_never_replace_ir_floor(self):
        self.runtime_pose()
        before = self.rgb.copy()
        with patch.object(gui.time, 'monotonic', return_value=100.5):
            self.app._update_window_candidate()
            self.assertIn('1F', self.app.window_match_var.get())
            self.assertIn('非高温判定', self.app.window_match_var.get())
            image = self.app._draw_window_centers(self.rgb)
        self.assertIsNot(image, self.rgb)
        self.assertTrue(np.any(image != self.rgb))
        np.testing.assert_array_equal(self.rgb, before)
        self.assertEqual(self.app._shown.floor, 'IR-original')

    def test_stale_missing_or_context_mismatched_pose_does_not_draw_or_match_floor(self):
        self.runtime_pose()
        for kind in ('stale', 'missing', 'size', 'mirror'):
            with self.subTest(kind=kind):
                self.runtime_pose()
                if kind == 'missing':
                    self.app._building_pose = None
                elif kind == 'size':
                    self.app._building_pose['size'] = (640, 480)
                elif kind == 'mirror':
                    self.app._building_pose['mirror'] = True
                with patch.object(gui.time, 'monotonic', return_value=101.21 if kind == 'stale' else 100.5):
                    with patch.object(gui.building_windows, 'match_floor', side_effect=AssertionError('must not match stale pose')):
                        self.app._update_window_candidate()
                        self.assertIs(self.app._draw_window_centers(self.rgb), self.rgb)
                self.assertNotIn('1F', self.app.window_match_var.get())
                self.assertEqual(self.app._shown.floor, 'IR-original')

    def test_runtime_missing_or_ambiguous_result_invalidates_previous_pose(self):
        self.runtime_pose()
        self.app._building_submissions[123] = (100, (320, 240), False)
        for results in ([], [detection(), detection()]):
            with self.subTest(count=len(results)):
                self.app._building_pose = dict(time=100, size=(320, 240), mirror=False, detection=detection())
                self.app._building_seen_id = None
                self.worker.result = 123, results
                with patch.object(gui.time, 'monotonic', return_value=100.5):
                    self.app._update_building_tracking()
                self.assertIsNone(self.app._building_pose)
                self.assertNotIn('1F', self.app.window_match_var.get())


if __name__ == '__main__':
    unittest.main()
