"""Exp16 测试入口和读数界面的回归（不访问相机或实际权重）。"""
import json
from concurrent.futures import Future
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import capture_photos as gui
import gauge_exp16 as backend
from gauge_exp16_vendor import CropReading, UnreadableGauge


class Exp16ReaderTests(unittest.TestCase):
    def test_one_prediction_uses_best_boxes_and_isolates_unreadable_crop(self):
        raw = np.full((170, 240, 3), 140, dtype=np.uint8)
        model = Mock()
        boxes = SimpleNamespace(xyxy=[[40, 55, 150, 120], [-3.3, 18.4, 89.9, 98.1],
                                      [101.6, 44.0, 260, 178]],
                                cls=[0, 0, 1], conf=[0.6, 0.9, 0.8])
        model.predict.return_value = [SimpleNamespace(boxes=boxes, names={0: "wendu", 1: "shidu"})]
        reader = backend.Exp16Reader(None, device="cpu", model=model)
        with patch.object(backend, "read_temperature", return_value=
                          CropReading(15.065, (30, 30), 24.0, 42.0, 78, "tick-gap")) as temperature, \
                patch.object(backend, "read_humidity", side_effect=UnreadableGauge("missing ticks")) as humidity:
            result = reader.process(raw)
        self.assertEqual(model.predict.call_count, 1)
        self.assertIs(model.predict.call_args.args[0], raw)
        self.assertEqual(model.predict.call_args.kwargs,
                         {"imgsz": 960, "conf": 0.05, "verbose": False, "device": "cpu"})
        self.assertEqual(temperature.call_args.args[0].shape, (81, 90, 3))
        self.assertEqual(humidity.call_args.args[0].shape, (126, 139, 3))
        self.assertEqual(result.temperature.bbox, (0, 18, 89, 98))
        self.assertEqual(result.temperature.value, 15.065)
        self.assertEqual(result.humidity.status, "unreadable")
        self.assertIsNone(result.humidity.value)
        self.assertFalse(np.array_equal(result.annotated, raw))
        self.assertTrue(np.all(raw == 140))

    def test_empty_detection_and_inference_error_return_no_values(self):
        model = Mock()
        reader = backend.Exp16Reader(None, model=model)
        frame = np.zeros((10, 20, 3), dtype=np.uint8)
        model.predict.return_value = []
        result = reader.process(frame)
        self.assertEqual(result.temperature.status, "missing")
        self.assertEqual(result.humidity.status, "missing")
        model.predict.side_effect = RuntimeError("inference failed")
        result = reader.process(frame)
        self.assertEqual(result.temperature.status, "error")
        self.assertIn("inference failed", result.as_dict()["temperature"]["detail"])
        self.assertIsNone(result.temperature.value)
        self.assertIsNone(result.humidity.value)


class Exp16CliTests(unittest.TestCase):
    def test_cli_option_is_isolated_from_pose(self):
        args = gui.parse_args(["--mode", "gauge-exp16", "--gauge-exp16-model", "exp16.pt"])
        self.assertEqual(args.mode, gui.MODE_GAUGE_EXP16)
        self.assertEqual(args.gauge_exp16_model, "exp16.pt")
        self.assertEqual(args.pose_model, str(gui.DEFAULT_POSE_MODEL))
        with patch.object(gui.sys, "stderr"), patch.object(gui, "run_gauge_cli") as pose:
            self.assertEqual(gui.run_cli(args, Path(".")), 1)
        pose.assert_not_called()


@unittest.skipUnless(gui.TK_AVAILABLE and gui.PIL_AVAILABLE, "GUI dependencies unavailable")
class Exp16GuiTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = gui.tk.Tk()
        except gui.tk.TclError as error:
            self.skipTest(str(error))
        self.addCleanup(self._destroy_root)
        camera_open = patch.object(gui.Camera, "open", side_effect=gui.CameraError("无相机"))
        camera_open.start()
        self.addCleanup(camera_open.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)

    def _destroy_root(self):
        try:
            self.root.destroy()
        except gui.tk.TclError:
            pass

    def _app(self, reader_factory, *options):
        args = gui.parse_args(["--mode", "gauge-exp16", *options])
        app = gui.Exp16GaugeApp(self.root, args, self.directory, Path("exp16.pt"), reader_factory)
        self.addCleanup(app.close)
        self.root.update_idletasks()
        return app

    def test_mode_dialog_has_separate_entry(self):
        dialog = gui.ModeDialog(self.root)
        self.assertNotEqual(gui.MODE_GAUGE, gui.MODE_GAUGE_EXP16)
        dialog.gauge_exp16_button.invoke()
        self.assertEqual(dialog.result, gui.MODE_GAUGE_EXP16)

    def test_startup_opens_video_zero_without_camera_dialog(self):
        with patch.object(gui, "Camera") as camera, \
                patch.object(gui, "CameraSelectDialog") as dialog:
            camera.return_value.frame_size = (1280, 720)
            app = self._app(Mock())
        self.assertEqual(camera.call_args.args, (0,))
        camera.return_value.open.assert_called_once_with()
        dialog.assert_not_called()
        self.assertEqual(app._source, "0")

    def test_explicit_startup_device_overrides_video_zero(self):
        with patch.object(gui, "Camera") as camera:
            camera.return_value.frame_size = (1280, 720)
            app = self._app(Mock(), "--device", "2")
        self.assertEqual(camera.call_args.args, (2,))
        camera.return_value.open.assert_called_once_with()
        self.assertEqual(app._source, "2")

    def test_real_tk_route_bypasses_camera_dialog(self):
        args = gui.parse_args(["--mode", "gauge-exp16"])
        with patch.object(gui.tk, "Tk", return_value=self.root), \
                patch.object(gui, "CameraSelectDialog") as camera_dialog, \
                patch.object(gui, "run_gui_gauge_exp16", return_value=0) as route:
            self.assertEqual(gui.run_gui(args, self.directory), 0)
        camera_dialog.assert_not_called()
        route.assert_called_once_with(args, self.directory, self.root)

    def test_photo_runs_once_and_saves_the_original_and_matching_result(self):
        source = self.directory / "仪表原图.png"
        raw = np.full((160, 230, 3), (10, 40, 130), dtype=np.uint8)
        annotated = np.full_like(raw, (250, 20, 70))
        gui.Image.fromarray(cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)).save(source)
        output = SimpleNamespace(annotated=annotated,
                                 temperature=SimpleNamespace(status="ok", value=15.065),
                                 humidity=SimpleNamespace(status="unreadable", value=None),
                                 as_dict=lambda: {"temperature": {"value": 15.065},
                                                  "humidity": {"status": "unreadable"}})
        reader = Mock()
        reader.process.return_value = output
        factory = Mock(return_value=reader)
        app = self._app(factory)
        with patch.object(gui.filedialog, "askopenfilename", return_value=str(source)):
            app.choose_image()
        deadline = time.monotonic() + 2
        while app._result is None and time.monotonic() < deadline:
            self.root.update()
            time.sleep(0.005)
        self.assertIsNotNone(app._result)
        self.assertIn("15.1 °C", app.temp_var.get())
        self.assertIn("无法读数", app.humidity_var.get())
        self.assertIn("1/10", app.temp_average_var.get())
        self.assertIn("0/10", app.humidity_average_var.get())
        factory.assert_called_once_with(Path("exp16.pt"), device="auto")
        self.assertEqual(reader.process.call_count, 1)
        np.testing.assert_array_equal(reader.process.call_args.args[0], raw)

        app.save_sample()
        (record,) = app.output_root.glob("*/readings.json")
        values = json.loads(record.read_text(encoding="utf-8"))
        self.assertEqual(values["source"], str(source))
        self.assertEqual(values["frame_size"], [230, 160])
        self.assertEqual(values["temperature"]["value"], 15.065)
        for filename, expected in (("raw.png", raw), ("annotated.png", annotated)):
            image = cv2.imdecode(np.fromfile(record.parent / filename, dtype=np.uint8), cv2.IMREAD_COLOR)
            np.testing.assert_array_equal(image, expected)

    def test_averages_use_ten_valid_samples_per_dial_and_slide(self):
        app = self._app(Mock())
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        missing = backend.GaugeReading("missing")
        unreadable = backend.GaugeReading("unreadable")

        def ok(value):
            return backend.GaugeReading("ok", value)

        def show(temp, humidity):
            app._show_result(frame, backend.Exp16Result(frame, temp, humidity), "time", 0.0)

        for value in range(9):
            show(ok(value), ok(2 * value) if value != 4 else unreadable)
        self.assertIn("9/10", app.temp_average_var.get())
        self.assertIn("8/10", app.humidity_average_var.get())
        show(ok(9), ok(18))
        self.assertEqual(app.temp_average_var.get(), "温度：4.5 °C")  # 包含有效的 0°C。
        self.assertIn("9/10", app.humidity_average_var.get())
        show(missing, ok(8))
        self.assertIn("未检测到", app.temp_var.get())
        self.assertEqual(app.temp_average_var.get(), "温度：4.5 °C")
        self.assertEqual(app.humidity_average_var.get(), "湿度：9.0 %RH")
        show(ok(10), ok(20))
        self.assertEqual(app.temp_var.get(), "温度：10.0 °C")
        self.assertEqual(app.temp_average_var.get(), "温度：5.5 °C")
        self.assertEqual(app.humidity_average_var.get(), "湿度：11.0 %RH")

        app._reset_source("切换输入")
        self.assertIn("0/10", app.temp_average_var.get())
        self.assertIn("0/10", app.humidity_average_var.get())
        show(ok(20), ok(40))
        self.assertIn("1/10", app.temp_average_var.get())
        self.assertIn("1/10", app.humidity_average_var.get())

    def test_disconnection_clears_values_and_ignores_an_in_flight_result(self):
        app = self._app(Mock())
        app._camera = Mock()
        app._camera.read.return_value = None
        app.temp_var.set("温度：15.1 °C")
        app._update_averages(SimpleNamespace(
            temperature=backend.GaugeReading("ok", 15.1),
            humidity=backend.GaugeReading("ok", 40.0)))
        job = Future()
        app._job = (job, app._generation, np.zeros((10, 10, 3)), time.monotonic(), "time")
        self.root.after_cancel(app._after_id)
        with patch.object(app, "_show_result") as show:
            app._tick()
            self.assertEqual(app.temp_var.get(), "温度：--")
            self.assertIn("无画面", app.status_var.get())
            job.set_result(None)
            self.root.after_cancel(app._after_id)
            app._tick()
            show.assert_not_called()
        self.assertIn("0/10", app.temp_average_var.get())
        self.assertIn("0/10", app.humidity_average_var.get())
        self.assertEqual(str(app.save_button["state"]), "disabled")


if __name__ == "__main__":
    unittest.main()
