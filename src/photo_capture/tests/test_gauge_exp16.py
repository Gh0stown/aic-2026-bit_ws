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

    def test_experimental_path_uses_native_temperature_crop_and_preserves_humidity(self):
        raw = np.random.default_rng(7).integers(0, 256, size=(120, 170, 3), dtype=np.uint8)
        model = Mock()
        boxes = SimpleNamespace(xyxy=[[5, 6, 95, 85], [70, 66, 125, 108]],
                                cls=[0, 1], conf=[0.9, 0.8])
        model.predict.return_value = [SimpleNamespace(boxes=boxes, names={0: "wendu", 1: "shidu"})]
        reader = backend.Exp16Reader(None, model=model)
        with patch.object(backend, "read_temperature", return_value=
                          CropReading(15.0, (30, 30), 24.0, 42.0, 78, "tick-order")) as legacy, \
                patch.object(backend, "read_humidity", return_value=
                             CropReading(40.0, (20, 20), 18.0, 88.0, 38, "tick-order")) as humidity, \
                patch("gauge_exp16_vendor.read_temperature_auto_ring.read_temperature_auto_ring",
                      return_value=CropReading(16.0, (30, 30), 26, 42.0, 79, "auto-ring")) as experiment:
            first = reader.process(raw)
            selected = reader.process(raw, temp_auto_ring=True)
            restored = reader.process(raw)
        self.assertEqual((first.temperature.value, selected.temperature.value, restored.temperature.value),
                         (15.0, 16.0, 15.0))
        self.assertEqual(legacy.call_count, 2)
        experiment.assert_called_once()
        np.testing.assert_array_equal(experiment.call_args.args[0], raw[6:86, 5:96])
        self.assertEqual(first.humidity, selected.humidity)
        np.testing.assert_array_equal(humidity.call_args_list[0].args[0], humidity.call_args_list[1].args[0])
        np.testing.assert_array_equal(first.annotated, restored.annotated)

    def test_humidity_experiment_receives_native_pixels_independent_of_temperature(self):
        raw = np.random.default_rng(8).integers(0, 256, size=(120, 170, 3), dtype=np.uint8)
        model = Mock()
        boxes = SimpleNamespace(xyxy=[[5, 6, 95, 85], [70, 66, 125, 108]],
                                cls=[0, 1], conf=[0.9, 0.8])
        model.predict.return_value = [SimpleNamespace(boxes=boxes, names={0: "wendu", 1: "shidu"})]
        reader = backend.Exp16Reader(None, model=model)
        reading = CropReading(15.0, (30, 30), 24, 42.0, 78, "tick-order")
        with patch.object(backend, "read_temperature", return_value=reading) as temperature, \
                patch.object(backend, "read_humidity", return_value=
                             CropReading(50.0, (20, 20), 18, 88.0, 38, "tick-order")) as legacy, \
                patch("gauge_exp16_vendor.read_humidity_auto_scale.read_humidity_auto_scale",
                      return_value=CropReading(48.0, (20, 20), 18, 88.0, 42, "humidity_auto_scale")) as trial:
            first = reader.process(raw)
            selected = reader.process(raw, humidity_auto_scale=True)
            restored = reader.process(raw)
        self.assertEqual((first.humidity.value, selected.humidity.value, restored.humidity.value),
                         (50.0, 48.0, 50.0))
        self.assertEqual(legacy.call_count, 2)
        trial.assert_called_once()
        np.testing.assert_array_equal(trial.call_args.args[0], raw[66:109, 70:126])
        self.assertEqual(first.temperature, selected.temperature)
        np.testing.assert_array_equal(temperature.call_args_list[0].args[0], temperature.call_args_list[1].args[0])
        np.testing.assert_array_equal(first.annotated, restored.annotated)

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
        self.assertFalse(args.gauge_exp16_temp_auto_ring)
        self.assertFalse(args.gauge_exp16_humidity_auto_scale)
        self.assertTrue(gui.parse_args(["--gauge-exp16-temp-auto-ring"]).gauge_exp16_temp_auto_ring)
        humidity = gui.parse_args(["--gauge-exp16-humidity-auto-scale"])
        self.assertTrue(humidity.gauge_exp16_humidity_auto_scale)
        self.assertFalse(humidity.gauge_exp16_temp_auto_ring)
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

    def test_temperature_toggle_discards_in_flight_reading_without_reopening_camera(self):
        with patch.object(gui, "Camera") as camera:
            camera.return_value.frame_size = (1280, 720)
            reader = Mock()
            factory = Mock(return_value=reader)
            app = self._app(factory)
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        self.assertFalse(app.temp_auto_ring_var.get())
        generation = app._generation
        app._batch.add(15.0, 40.0)
        pending = Future()
        pending.set_result(None)
        app._job = (pending, generation, frame, time.monotonic(), "time")
        app.temp_mode_button.invoke()
        self.assertTrue(app.temp_auto_ring_var.get())
        self.assertGreater(app._generation, generation)
        self.assertEqual(app._batch.count, 0)
        self.assertIn("0/10", app.progress_var.get())
        self.root.after_cancel(app._after_id)
        app._camera.read.return_value = frame
        with patch.object(app, "_show_result") as show, patch.object(app, "_submit") as submit:
            app._tick()
            show.assert_not_called()
            submit.assert_called_once()
        app._process(frame, app.temp_auto_ring_var.get())
        self.assertEqual(reader.process.call_args.kwargs, {"temp_auto_ring": True})
        app.temp_mode_button.invoke()
        app._process(frame, app.temp_auto_ring_var.get())
        self.assertEqual(reader.process.call_args.kwargs, {})
        factory.assert_called_once()
        camera.return_value.open.assert_called_once_with()
        camera.return_value.release.assert_not_called()

    def test_temperature_toggle_requeues_open_photo(self):
        app = self._app(Mock(), "--gauge-exp16-temp-auto-ring")
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        app._source = "source.png"
        app._result = (frame, None, "time")
        self.assertTrue(app.temp_auto_ring_var.get())
        app.temp_mode_button.invoke()
        self.assertFalse(app.temp_auto_ring_var.get())
        self.assertEqual(app._source, "source.png")
        self.assertIs(app._pending_image, frame)
        self.assertIsNone(app._result)

    def test_humidity_switch_requeues_photo_and_preserves_temperature_algorithm(self):
        reader = Mock()
        factory = Mock(return_value=reader)
        app = self._app(factory, "--gauge-exp16-temp-auto-ring")
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        app._source = "source.png"
        app._result = (frame, None, "time")
        self.assertFalse(app.humidity_auto_scale_var.get())
        app.humidity_mode_button.invoke()
        self.assertTrue(app.humidity_auto_scale_var.get())
        self.assertTrue(app.temp_auto_ring_var.get())
        self.assertIs(app._pending_image, frame)
        self.assertIsNone(app._result)
        with patch.object(app._worker, "submit", return_value=Future()) as submit:
            app._submit(frame, time.monotonic())
        self.assertIs(submit.call_args.args[1], frame)
        self.assertEqual(submit.call_args.args[2:], (True, True))
        app._process(frame, True, True)
        self.assertEqual(reader.process.call_args.kwargs,
                         {"temp_auto_ring": True, "humidity_auto_scale": True})
        app.humidity_mode_button.invoke()
        self.assertTrue(app.temp_auto_ring_var.get())
        self.assertFalse(app.humidity_auto_scale_var.get())
        app._process(frame, True, False)
        self.assertEqual(reader.process.call_args.kwargs, {"temp_auto_ring": True})
        factory.assert_called_once()

    def test_photo_runs_once_and_saves_the_original_and_matching_result(self):
        source = self.directory / "仪表原图.png"
        raw = np.full((160, 230, 3), (10, 40, 130), dtype=np.uint8)
        annotated = np.full_like(raw, (250, 20, 70))
        gui.Image.fromarray(cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)).save(source)
        output = backend.Exp16Result(annotated, backend.GaugeReading("ok", 15.065),
                                     backend.GaugeReading("unreadable"))
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
        self.assertEqual(app.temp_var.get(), "温度：--")
        self.assertEqual(app.humidity_var.get(), "湿度：--")
        self.assertIn("单图检查", app.progress_var.get())
        self.assertEqual(app._batch.count, 0)
        np.testing.assert_array_equal(np.asarray(app.preview._source_image),
                                      cv2.cvtColor(raw, cv2.COLOR_BGR2RGB))
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

    def test_camera_publishes_only_completed_batch_and_stays_frozen_until_retry(self):
        with patch.object(gui, "Camera") as camera:
            camera.return_value.frame_size = (1280, 720)
            app = self._app(Mock())
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        for index in range(10):
            output = backend.Exp16Result(frame, backend.GaugeReading("ok", 35 if index == 9 else 15),
                                         backend.GaugeReading("ok", 22 if index in (2, 7) else 40))
            app._show_result(frame, output, "time", 0.0)
            if index < 9:
                self.assertIn("{}/10".format(index + 1), app.progress_var.get())
                self.assertEqual(app.temp_var.get(), "温度：--")
                self.assertEqual(app.humidity_var.get(), "湿度：--")
                self.assertEqual(str(app.save_button["state"]), "disabled")
        self.assertEqual(app.temp_var.get(), "温度：15.0 °C")
        self.assertEqual(app.humidity_var.get(), "湿度：40.0 %RH")
        self.assertIn("测量结束", app.progress_var.get())

        self.root.after_cancel(app._after_id)
        app._tick()
        camera.return_value.read.assert_not_called()  # 完成后不再读帧或推理。
        app.save_sample()
        (record,) = app.output_root.glob("*/readings.json")
        values = json.loads(record.read_text(encoding="utf-8"))
        self.assertEqual(values["temperature"]["value"], 35)
        self.assertEqual(values["stable_readings"]["temperature"]["value"], 15.0)

        with patch.object(gui, "Camera") as reopened:
            reopened.return_value.frame_size = (1280, 720)
            app.measure_again()
        camera.return_value.release.assert_called_once_with()
        reopened.return_value.open.assert_called_once_with()
        self.assertIsNone(app._last_batch)
        self.assertEqual(app._reported, {})
        self.assertEqual(app.temp_var.get(), "温度：--")
        self.assertIn("0/10", app.progress_var.get())

    def test_conflicting_samples_are_shown_only_as_reference_at_cap(self):
        app = self._app(Mock())
        app._camera = Mock()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        for index in range(20):
            temp = backend.GaugeReading("ok", 15) if index < 5 else backend.GaugeReading("missing")
            hum = backend.GaugeReading("ok", 40 if index % 10 < 5 else 50)
            app._show_result(frame, backend.Exp16Result(frame, temp, hum), "time", 0.0)
            if index == 9:
                self.assertIn("补采 10/20", app.progress_var.get())
                self.assertEqual(app.temp_var.get(), "温度：--")
                self.assertEqual(app.humidity_var.get(), "湿度：--")
        self.assertEqual(app.temp_var.get(), "温度：15.0 °C（低可信参考）")
        self.assertIn("低可信参考", app.humidity_var.get())
        self.assertIn(app._reported["humidity"]["value"], (40, 50))
        app.save_sample()
        (record,) = app.output_root.glob("*/readings.json")
        values = json.loads(record.read_text(encoding="utf-8"))
        self.assertEqual(values["displayed_readings"]["humidity"]["status"], "reference")
        self.assertEqual(values["stable_readings"]["frame_count"], 20)

    def test_camera_extends_sampling_when_humidity_is_sparse(self):
        app = self._app(Mock())
        app._camera = Mock()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        for index in range(12):
            temp = backend.GaugeReading("ok", 15 if index < 10 else 35)
            hum = (backend.GaugeReading("ok", 40) if index in (0, 3, 6, 9, 10, 11)
                   else backend.GaugeReading("unreadable"))
            app._show_result(frame, backend.Exp16Result(frame, temp, hum), "time", 0.0)
            if 9 <= index < 11:
                self.assertIn("补采 {}/20".format(index + 1), app.progress_var.get())
                self.assertIn("湿度有效 {}".format(index - 5), app.progress_var.get())
                self.assertEqual(app.humidity_var.get(), "湿度：--")
                self.assertEqual(str(app.save_button["state"]), "disabled")
        self.assertIn("12 帧", app.progress_var.get())
        self.assertEqual(app.temp_var.get(), "温度：15.0 °C")
        self.assertEqual(app.humidity_var.get(), "湿度：40.0 %RH")
        self.assertEqual(str(app.save_button["state"]), "normal")

    def test_missing_humidity_retries_without_changing_displayed_temperature(self):
        app = self._app(Mock())
        app._camera = Mock()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        app._camera.read.return_value = frame
        output = backend.Exp16Result(frame, backend.GaugeReading("ok", 15),
                                     backend.GaugeReading("unreadable"))
        for index in range(20):
            app._show_result(frame, output, "time", 0.0)
            if index == 18:
                self.assertIn("补采 19/20", app.progress_var.get())
        self.assertEqual(app._last_batch.frame_count, 20)
        self.assertIn("自动开始第 2 轮", app.progress_var.get())
        self.assertIn("无有效读数，续采中", app.humidity_var.get())
        self.assertEqual(app.temp_var.get(), "温度：15.0 °C")
        self.root.after_cancel(app._after_id)
        with patch.object(app, "_submit") as submit:
            app._tick()
            submit.assert_called_once()

        output = backend.Exp16Result(frame, backend.GaugeReading("ok", 35),
                                     backend.GaugeReading("ok", 40))
        for _ in range(10):
            app._show_result(frame, output, "time", 0.0)
        self.assertIn("测量结束 · 第 2 轮 10 帧", app.progress_var.get())
        self.assertEqual(app.temp_var.get(), "温度：15.0 °C")
        self.assertEqual(app.humidity_var.get(), "湿度：40.0 %RH")
        self.assertEqual(app._reported["temperature"]["round"], 1)
        self.assertEqual(app._reported["humidity"]["round"], 2)
        self.assertEqual(app._last_batch.temperature.status, "skipped")

    def test_all_unreadable_auto_retries_without_manufacturing_a_value(self):
        app = self._app(Mock())
        app._camera = Mock()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        output = backend.Exp16Result(frame, backend.GaugeReading("missing"),
                                     backend.GaugeReading("unreadable"))
        for _ in range(40):
            app._show_result(frame, output, "time", 0.0)
        self.assertIn("自动开始第 3 轮", app.progress_var.get())
        self.assertIn("温度：--", app.temp_var.get())
        self.assertIn("湿度：--", app.humidity_var.get())
        self.assertEqual(app._reported, {})

    def test_disconnection_clears_values_and_ignores_an_in_flight_result(self):
        app = self._app(Mock())
        app._camera = Mock()
        app._camera.read.return_value = None
        for _ in range(10):
            self.assertIsNone(app._batch.add(15.1, None))
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
        self.assertEqual(app._batch.count, 0)
        self.assertIsNone(app._last_batch)
        self.assertEqual(app._reported, {})
        self.assertEqual(str(app.save_button["state"]), "disabled")


if __name__ == "__main__":
    unittest.main()
