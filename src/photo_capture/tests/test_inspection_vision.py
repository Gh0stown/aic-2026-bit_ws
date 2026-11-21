#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""inspection_vision 节点的离线回归：不连 ROS、不开相机、不加载真实权重。

覆盖：
- 配置合并 / params 解析 / 请求截止与 request_id；
- 帧源（图片文件夹每次请求取同一张，请求之间换图）；
- 画图落盘、结果 JSONL；
- light 多帧投票、crowd 类别归一化、trash 24 类映射与空间分配；
- fire 楼宇名匹配、thermal 楼层表、meter 读数结构；
- VisionService 的分发、未知任务、非法 params、超时。
"""

import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import iv_common
import iv_service
import iv_tasks_detect
import iv_tasks_ebike
import iv_tasks_meter
import iv_tasks_ocr
import iv_tasks_thermal
import iv_tasks_trash
from iv_common import Detection, FrameSource, TaskResult, VisionContext, VisionTimeout


# ---------------------------------------------------------------------------
# 假件
# ---------------------------------------------------------------------------

class FakeModel:
    def __init__(self, path, names, detections):
        self.path = Path(path)
        self.names = names
        self.detections = list(detections)

    def predict(self, frame, conf=None, imgsz=None, classes=None, max_det=None):
        wanted = set(classes) if classes else None
        height, width = frame.shape[:2]
        return [det for det in self.detections
                if (wanted is None or det.name in wanted)
                and det.x2 <= width and det.y2 <= height]


class FakeModels:
    def __init__(self, models, model_dir="/tmp/models"):
        self.models = models
        self.model_dir = Path(model_dir)

    def resolve(self, value):
        return Path(str(value))

    def get(self, value, imgsz=640, conf=0.25):
        key = Path(str(value)).name
        if key not in self.models:
            raise iv_common.ModelUnavailable("测试里没有模型 %s" % key)
        return self.models[key]

    def loaded(self):
        return sorted(self.models)


class FakeFrames:
    description = "测试帧源"

    def __init__(self, size=(240, 320), count=3):
        self.frames = [np.full((size[0], size[1], 3), 30 + index * 10, np.uint8)
                       for index in range(count)]
        self.calls = 0
        self.advanced = 0
        self.last_count = None

    def grab(self, count=1, interval=0.0, warmup=None):
        self.last_count = count
        if self.calls >= len(self.frames):
            return [self.frames[-1].copy() for _ in range(count)]
        frame = self.frames[self.calls]
        self.calls += 1
        return [frame.copy() for _ in range(max(1, count))]

    def read(self):
        frame = self.frames[min(self.calls, len(self.frames) - 1)]
        self.calls += 1
        return frame.copy()

    def advance(self):
        self.advanced += 1

    def open(self):
        return self

    def close(self):
        pass


def make_context(directory, detections_by_model, frames=None, config_override=None):
    config = iv_common.load_config()
    if config_override:
        config = iv_common._deep_merge(config, config_override)
    models = {name: FakeModel(name, {0: "x"}, dets)
              for name, dets in detections_by_model.items()}
    context = VisionContext(config, FakeModels(models),
                            frames or FakeFrames(), output_root=Path(directory))
    return context


def det(name, x1=10, y1=10, x2=60, y2=80, conf=0.9, cls_id=0):
    return Detection(x1, y1, x2, y2, conf, cls_id, name)


# ---------------------------------------------------------------------------
# 基础设施
# ---------------------------------------------------------------------------

class ConfigTests(unittest.TestCase):
    def test_defaults_and_merge(self):
        config = iv_common.load_config()
        self.assertIn("camera", config)
        self.assertTrue(config["timeouts"]["meter"] >= 10.0)
        merged = iv_common._deep_merge(config, {"thresholds": {"light": 0.7}})
        self.assertAlmostEqual(merged["thresholds"]["light"], 0.7)
        self.assertAlmostEqual(merged["thresholds"]["sign"], config["thresholds"]["sign"])

    def test_parse_params(self):
        self.assertEqual(iv_common.parse_params(""), ({}, ""))
        params, error = iv_common.parse_params('{"frames": 3}')
        self.assertFalse(error)
        self.assertEqual(params["frames"], 3)
        _, error = iv_common.parse_params("{bad")
        self.assertTrue(error)

    def test_safe_name_keeps_chinese(self):
        self.assertEqual(iv_common.safe_name("1号停车位"), "1号停车位")
        self.assertNotIn("/", iv_common.safe_name("a/b c"))


class FrameSourceTests(unittest.TestCase):
    def test_image_folder_uses_same_frame_until_advance(self):
        with tempfile.TemporaryDirectory() as directory:
            for index in range(2):
                image = np.full((20, 30, 3), index * 100, np.uint8)
                cv2.imwrite(str(Path(directory) / ("%d.jpg" % index)), image)
            source = FrameSource(directory)
            source.open()
            first = [source.read() for _ in range(3)]
            self.assertTrue(all(np.array_equal(frame, first[0]) for frame in first))
            source.advance()
            second = source.read()
            self.assertFalse(np.array_equal(first[0], second))
            source.close()


class ContextTests(unittest.TestCase):
    def test_deadline_and_logging(self):
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {})
            context.begin_request("crowd", 2, "A街区", {"request_id": "r1"},
                                  "r1", timeout=5.0)
            self.assertGreater(context.remaining(), 0)
            result = context.ok("ok", {"person": 1}, [])
            context.finish(result)
            self.assertTrue(context.jsonl_path.is_file())
            entry = json.loads(context.jsonl_path.read_text(encoding="utf-8").strip())
            self.assertEqual(entry["request_id"], "r1")
            self.assertEqual(entry["task"], "crowd")
            self.assertTrue(entry["ok"])
            self.assertIn("elapsed", entry["extra"])

    def test_params_frames_overrides_config(self):
        with tempfile.TemporaryDirectory() as directory:
            frames = FakeFrames()
            context = make_context(directory, {}, frames=frames)
            context.begin_request("crowd", 2, "", {"frames": 1}, "r", timeout=5.0)
            context.grab()
            self.assertEqual(frames.last_count, 1)
            context.begin_request("crowd", 2, "", {}, "r2", timeout=5.0)
            context.grab()
            self.assertEqual(frames.last_count, 2)

    def test_check_deadline_raises(self):
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {})
            context.begin_request("crowd", 2, "", {}, "r", timeout=0.5)
            context.deadline = 0.0
            with self.assertRaises(VisionTimeout):
                context.check_deadline()


# ---------------------------------------------------------------------------
# task handlers（假模型）
# ---------------------------------------------------------------------------

class LightTests(unittest.TestCase):
    def test_three_red_frames_vote(self):
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {
                "light.pt": [det("红灯", 12, 12, 40, 40, 0.8)],
            })
            context.begin_request("light", 1, "红绿灯1", {}, "r", timeout=5.0)
            result = iv_tasks_detect.handle_light(context, {})
            self.assertTrue(result.ok)
            self.assertEqual(result.result, "红灯")
            self.assertEqual(result.extra["color"], "red")
            self.assertEqual(result.extra["votes"], {"红灯": 3})
            self.assertTrue(result.images and Path(result.images[0]).is_file())

    def test_no_detection_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {"light.pt": []})
            context.begin_request("light", 1, "", {}, "r", timeout=5.0)
            result = iv_tasks_detect.handle_light(context, {})
            self.assertFalse(result.ok)
            self.assertIn("红绿灯", result.extra["error"])


class CrowdTests(unittest.TestCase):
    def test_roles_counted_and_cycle_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {
                "people.pt": [det("people", 0, 0, 20, 40, 0.8, 0),
                              det("people", 30, 0, 50, 40, 0.7, 0),
                              det("bad", 60, 0, 80, 40, 0.6, 1),
                              det("cycle", 90, 0, 120, 40, 0.9, 2)],
            })
            context.begin_request("crowd", 2, "A街区", {}, "r", timeout=5.0)
            result = iv_tasks_detect.handle_crowd(context, {})
            self.assertTrue(result.ok)
            self.assertEqual(result.extra["person"], 2)
            self.assertEqual(result.extra["bad"], 1)
            self.assertNotIn("cycle", result.extra)
            self.assertIn("社区人员 2 人", result.result)


class SignTests(unittest.TestCase):
    def test_side_and_route(self):
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {
                "Signal.pt": [det("A", 10, 10, 80, 80, 0.9)],
                "stop.pt": [det("禁止通行标志", 10, 10, 80, 80, 0.9)],
            })
            context.begin_request("sign", 4, "A街区", {}, "r", timeout=5.0)
            result = iv_tasks_detect.handle_sign(context, {})
            self.assertTrue(result.ok)
            self.assertEqual(result.extra["side"], "A")
            self.assertEqual(result.extra["route"], "left")
            self.assertTrue(result.extra["confirm_hit"])


class TrashTests(unittest.TestCase):
    def setUp(self):
        self.bins = [
            det("可回收垃圾状态为关闭", 100, 100, 150, 220, 0.8, 5),
            det("有害垃圾状态为打开", 155, 100, 200, 220, 0.8, 0),
            det("厨余垃圾状态为关闭", 10, 100, 90, 220, 0.8, 3),
            det("其他垃圾状态为打开", 205, 100, 300, 220, 0.8, 6),
        ]

    def test_24_class_mapping_and_assignment(self):
        items = [det("7", 110, 180, 140, 210, 0.7),    # 塑料瓶 -> 可回收物
                 det("19", 160, 180, 190, 210, 0.6),   # 纽扣电池 -> 有害
                 det("1", 30, 180, 60, 210, 0.5),      # 果核 -> 厨余
                 det("7", 215, 180, 245, 210, 0.4)]    # 塑料瓶 -> 其他（错误投放）
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {
                "trash_brest.pt": self.bins,
                "trash.pt": items,
            })
            context.begin_request("trash", 15, "垃圾桶", {}, "r", timeout=8.0)
            result = iv_tasks_trash.handle(context, {})
            self.assertTrue(result.ok)
            by_type = {entry["type"]: entry for entry in result.extra["bins"]}
            self.assertEqual(result.extra["detected"], 4)
            self.assertEqual(by_type["可回收物"]["item"], "塑料瓶")
            self.assertTrue(by_type["可回收物"]["correct"])
            self.assertEqual(by_type["有害"]["item"], "纽扣电池")
            self.assertTrue(by_type["有害"]["correct"])
            self.assertEqual(by_type["厨余"]["item"], "果核")
            self.assertTrue(by_type["厨余"]["correct"])
            self.assertEqual(by_type["其他"]["item"], "塑料瓶")
            self.assertFalse(by_type["其他"]["correct"])
            self.assertIn("可回收物垃圾桶状态为关闭", result.result)

    def test_bin_parser_and_normalize(self):
        self.assertEqual(iv_tasks_trash._parse_bin(self.bins[0]), ("可回收物", "关闭"))
        self.assertEqual(iv_tasks_trash._parse_bin(self.bins[3]), ("其他", "打开"))
        self.assertEqual(iv_tasks_trash._normalize_type("其它垃圾"), "其他")
        self.assertEqual(iv_tasks_trash._normalize_type("可回收垃圾"), "可回收物")

    def test_catalog_style_item_names(self):
        entry = det("可回收垃圾-塑料瓶", cls_id=13)
        self.assertEqual(iv_tasks_trash._parse_item(entry), ("可回收物", "塑料瓶"))


class EbikeTests(unittest.TestCase):
    def test_street_uses_cycle(self):
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {
                "people.pt": [det("cycle", 10, 10, 90, 90, 0.8, 2)],
            })
            context.begin_request("ebike", 2, "A街区", {"conf": 0.3}, "r", timeout=5.0)
            result = iv_tasks_ebike.handle(context, {})
            self.assertTrue(result.ok)
            self.assertEqual(result.extra["mode"], "street")
            self.assertEqual(result.extra["illegal"], 1)
            self.assertIn("A 街区有 1 辆电动车违停", result.result)

    def test_parking_frame_counts_up_down(self):
        with tempfile.TemporaryDirectory() as directory:
            context = make_context(directory, {
                "Cycle.pt": [det("UP", 0, 0, 40, 60, 0.9), det("UP", 50, 0, 90, 60, 0.9),
                             det("DOWN", 100, 0, 140, 60, 0.9)],
            })
            context.begin_request("ebike", 17, "停车区", {}, "r", timeout=5.0)
            result = iv_tasks_ebike.handle(context, {})
            self.assertTrue(result.ok)
            self.assertEqual(result.extra["normal"], 2)
            self.assertEqual(result.extra["fall"], 1)


class OcrHelperTests(unittest.TestCase):
    def test_building_match(self):
        name, source = iv_tasks_ocr._match_building("A", "A", [])
        self.assertEqual((name, source), ("A", "ocr"))
        name, source = iv_tasks_ocr._match_building("F858573", "A", [])
        self.assertEqual((name, source), ("A", "default"))
        name, source = iv_tasks_ocr._match_building("创业大厦", "A", ["创业大厦", "创新大厦"])
        self.assertEqual((name, source), ("创业大厦", "ocr"))

    def test_parking_number_from_sub_id(self):
        context = types.SimpleNamespace(sub_id="2号停车位", point_id=15)
        self.assertEqual(iv_tasks_ocr._parking_number(context), "2")
        context = types.SimpleNamespace(sub_id="", point_id=16)
        self.assertEqual(iv_tasks_ocr._parking_number(context), "3")


class ThermalTests(unittest.TestCase):
    def test_floor_lookup_and_out_of_band(self):
        bands = (("4F", 0, 5), ("3F", 6, 11), ("2F", 12, 17), ("1F", 18, 23))

        def floor_for(row):
            for name, first, last in bands:
                if first <= row <= last:
                    return name
            return "1F"

        table = types.SimpleNamespace(
            bands=bands, floor_for=floor_for,
            to_text=lambda: "4F:0-5,3F:6-11,2F:12-17,1F:18-23")
        session = types.SimpleNamespace(floor_table=table)
        context = types.SimpleNamespace(config={}, remaining=lambda: 5.0)
        self.assertEqual(iv_tasks_thermal._floor_number(context, session, 8.2),
                         ("3F", 3))
        self.assertEqual(iv_tasks_thermal._floor_number(context, session, 1.5),
                         ("4F", 4))


class MeterTests(unittest.TestCase):
    def test_reading_dict(self):
        dial = types.SimpleNamespace(value=15.0, status="ok",
                                     valid_count=10, inlier_count=9,
                                     as_dict=lambda: {"value": 15.0, "status": "ok"})
        payload = iv_tasks_meter._reading_dict(None, dial)
        self.assertEqual(payload["value"], 15.0)
        self.assertEqual(payload["valid_count"], 10)
        self.assertEqual(payload["inlier_count"], 9)


# ---------------------------------------------------------------------------
# VisionService 分发
# ---------------------------------------------------------------------------

class ServiceTests(unittest.TestCase):
    def make_service(self, directory):
        context = make_context(directory, {})
        service = iv_service.VisionService(context)
        service.handlers["crowd"] = lambda ctx, params: ctx.ok("ok", {"person": 3})
        return service

    def test_dispatch_success_and_request_id(self):
        with tempfile.TemporaryDirectory() as directory:
            service = self.make_service(directory)
            result = service.handle("crowd", 2, "A街区",
                                    '{"request_id": "req-1"}')
            self.assertTrue(result.ok)
            self.assertEqual(result.extra["request_id"], "req-1")
            log = Path(directory) / service.context.run_stamp / "recognitions.jsonl"
            self.assertTrue(log.is_file())

    def test_failure_reason_is_logged(self):
        with tempfile.TemporaryDirectory() as directory:
            service = self.make_service(directory)
            service.handlers["crowd"] = lambda ctx, params: ctx.fail("相机断流")
            result = service.handle("crowd", 2, "", "{}")
            self.assertFalse(result.ok)
            self.assertEqual(result.error, "相机断流")
            entry = json.loads(
                (Path(directory) / service.context.run_stamp
                 / "recognitions.jsonl").read_text(encoding="utf-8").strip())
            self.assertEqual(entry["error"], "相机断流")

    def test_unknown_task(self):
        with tempfile.TemporaryDirectory() as directory:
            service = self.make_service(directory)
            result = service.handle("nope", 1, "", "{}")
            self.assertFalse(result.ok)
            self.assertIn("未知任务", result.extra["error"])

    def test_bad_params(self):
        with tempfile.TemporaryDirectory() as directory:
            service = self.make_service(directory)
            result = service.handle("crowd", 2, "", "{bad")
            self.assertFalse(result.ok)
            self.assertIn("params", result.extra["error"])

    def test_handler_timeout_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            service = self.make_service(directory)

            def timeout_handler(ctx, params):
                raise VisionTimeout("超时了")

            service.handlers["crowd"] = timeout_handler
            result = service.handle("crowd", 2, "", "{}")
            self.assertFalse(result.ok)
            self.assertEqual(result.result, "识别超时")
            self.assertIn("超时", result.extra["error"])

    def test_deadline_override_clamped(self):
        with tempfile.TemporaryDirectory() as directory:
            service = self.make_service(directory)
            short = service.timeout_for("crowd", {"deadline": 0.1})
            self.assertGreaterEqual(short, 0.5)
            long = service.timeout_for("crowd", {"deadline": 10000})
            self.assertLessEqual(long, iv_service.DEFAULT_MAX_TIMEOUT)


if __name__ == "__main__":
    unittest.main()
