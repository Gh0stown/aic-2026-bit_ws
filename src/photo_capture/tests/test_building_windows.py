#!/usr/bin/env python3
"""Facade geometry, safe JSON and pose payloads: no model/device initialization."""
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import building_windows as bw
import yolo_detect


SIZE = (1280, 720)
CORNERS = np.array([[100, 50], [700, 50], [700, 650], [100, 650]], dtype=float)
PLANE_CENTERS = {'1F': (0.5, 0.8), '2F': (0.5, 0.6),
                 '3F': (0.5, 0.4), '4F': (0.5, 0.2)}


def centers(corners=CORNERS, normalized=PLANE_CENTERS):
    return {floor: bw.to_image(corners, *point) for floor, point in normalized.items()}


def template(**kwargs):
    return bw.make_template(CORNERS, centers(), SIZE, **kwargs)


class GeometryTests(unittest.TestCase):
    def test_known_pixel_coordinates_and_roundtrip(self):
        np.testing.assert_allclose(bw.to_plane(CORNERS, 250, 500), (0.25, 0.75))
        np.testing.assert_allclose(bw.to_image(CORNERS, 0.25, 0.75), (250, 500))
        trapezoid = np.array([[100, 60], [750, 100], [680, 600], [180, 640]])
        for s, t in [(0, 0), (1, 0), (1, 1), (0, 1), (0.23, 0.681), (-0.4, 1.2)]:
            uv = bw.to_image(trapezoid, s, t)
            np.testing.assert_allclose(bw.to_plane(trapezoid, *uv), (s, t), atol=2e-7)
        for pixel, point in zip(trapezoid, [(0, 0), (1, 0), (1, 1), (0, 1)]):
            np.testing.assert_allclose(bw.to_plane(trapezoid, *pixel), point, atol=2e-7)

    def test_homography_uses_existing_project_and_cv2(self):
        with patch.object(bw, 'project', wraps=bw.project) as project, patch.object(
                bw.cv2, 'getPerspectiveTransform', wraps=bw.cv2.getPerspectiveTransform) as fit:
            bw.to_plane(CORNERS, 250, 500)
        self.assertTrue(project.called)
        self.assertEqual(fit.call_count, 1)

    def test_template_reprojects_on_new_pose_not_absolute_pixels(self):
        value = template()
        for floor in bw.FLOORS:
            np.testing.assert_allclose(value.centers[floor], PLANE_CENTERS[floor], atol=1e-8)
        self.assertEqual(value.reference_corners, tuple(map(tuple, CORNERS)))
        self.assertEqual(value.reference_centers, centers())
        moved = np.array([[500, 0], [900, 80], [1100, 600], [250, 550]])
        result = bw.template_centers(value, moved)
        for floor in bw.FLOORS:
            np.testing.assert_allclose(bw.to_plane(moved, *result[floor]), value.centers[floor], atol=2e-7)
        self.assertNotEqual(result['1F'], value.reference_centers['1F'])

    def test_nonuniform_and_noncollinear_centers_allowed(self):
        normalized = {'1F': (0.6, 0.9), '2F': (0.48, 0.6),
                      '3F': (0.52, 0.42), '4F': (0.4, 0.1)}
        value = bw.make_template(CORNERS, centers(normalized=normalized), SIZE)
        for floor in bw.FLOORS:
            np.testing.assert_allclose(value.centers[floor], normalized[floor])

    def test_bad_quadrilaterals_rejected_without_reordering(self):
        invalid = [[], CORNERS[:3], CORNERS[:, :1], CORNERS[::-1], CORNERS[[0, 2, 1, 3]],
                   [[0, 0], [2, 0], [1, 0.5], [0, 2]],
                   [[0, 0], [1, 0], [2, 0], [0, 1]],
                   [[0, 0], [0, 0], [2, 2], [0, 2]],
                   [['0', '0'], ['1', '0'], ['1', '1'], ['0', '1']]]
        for bad in (np.nan, np.inf, -np.inf):
            points = CORNERS.copy()
            points[1, 0] = bad
            invalid.append(points)
        for points in invalid:
            with self.subTest(points=points), self.assertRaises(ValueError):
                bw.make_template(points, centers(), SIZE)

    def test_invalid_centers_rejected(self):
        base = centers()
        invalid = [{}, dict(base, **{'5F': (5, 5)}), {k: v for k, v in base.items() if k != '1F'}]
        for point in [(400, 50), (400, 651), (99, 300), (np.nan, 200), (True, 200),
                      ('400', 200), (400,), (400, 200, 5)]:
            invalid.append(dict(base, **{'1F': point}))
        invalid.append(dict(base, **{'1F': base['2F']}))
        invalid.append(dict(base, **{'1F': base['4F'], '4F': base['1F']}))
        invalid.append(dict(base, **{'1F': (450, base['2F'][1])}))
        for points in invalid:
            with self.subTest(points=points), self.assertRaises(ValueError):
                bw.make_template(CORNERS, points, SIZE)

    def test_reference_geometry_must_be_visible_inside_rgb(self):
        for offset in ((700, 0), (-101, 0), (0, 100), (0, -51)):
            facade = CORNERS + offset
            with self.subTest(offset=offset), self.assertRaisesRegex(ValueError, 'rgb_size'):
                bw.make_template(facade, centers(facade), SIZE)

    def test_invalid_metadata_and_projection_inputs(self):
        for kwargs in ({'rgb_mirror': 1}, {'rgb_size': (0, 100)},
                       {'rgb_size': (1280.0, 720)}, {'rgb_size': (True, 720)}):
            args = dict(rgb_size=SIZE, **{})
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                bw.make_template(CORNERS, centers(), **args)
        for point in [(np.nan, 0), (True, 0), ('0.5', 0)]:
            with self.assertRaises(ValueError):
                bw.to_plane(CORNERS, *point)
        np.testing.assert_allclose(bw.to_plane(CORNERS, -500, 800), (-1, 1.25))

    def test_match_only_near_centers_inside_facade(self):
        value = template()
        for floor, point in centers().items():
            self.assertEqual(bw.match_floor(value, CORNERS, *point), floor)
        for point in [(0.5, 0.81), (0.5 + 0.09 - 1e-5, 0.8)]:
            self.assertEqual(bw.match_floor(value, CORNERS, *bw.to_image(CORNERS, *point)), '1F')
        for point in [(0.5 + 0.09 + 1e-5, 0.8), (0.9, 0.8), (0.5, 0.7),
                      (-0.01, 0.8), (0.5, 1.01), (0, 0), (1, 1)]:
            self.assertIsNone(bw.match_floor(value, CORNERS, *bw.to_image(CORNERS, *point)))

    def test_match_normalized_distance_independent_of_aspect_and_perspective(self):
        value = template()
        for facade in [CORNERS, [[1, 1], [1001, 1], [1001, 101], [1, 101]],
                       [[100, 80], [900, 50], [620, 650], [250, 600]]]:
            self.assertEqual(bw.match_floor(value, facade, *bw.to_image(facade, .55, .4)), '3F')
            self.assertIsNone(bw.match_floor(value, facade, *bw.to_image(facade, .65, .4)))

    def test_gate_uses_smallest_consecutive_floor_spacing(self):
        normalized = dict(PLANE_CENTERS, **{'4F': (0.5, 0.39)})
        value = bw.make_template(CORNERS, centers(normalized=normalized), SIZE)
        self.assertEqual(bw.match_floor(value, CORNERS, *bw.to_image(CORNERS, .504, .8)), '1F')
        self.assertIsNone(bw.match_floor(value, CORNERS, *bw.to_image(CORNERS, .506, .8)))


class FileTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.path = self.root / 'window_centers.json'
        self.model = self.root / 'building_d.pt'
        self.model.write_bytes(b'fake-weights-no-model-load')
        self.value = template(model_path=self.model, rgb_mirror=True)
        bw.save_template(self.path, self.value)

    def test_roundtrip_version_orders_and_fingerprint(self):
        loaded = bw.load_template(self.path, SIZE, True, self.model)
        self.assertEqual(loaded, self.value)
        payload = json.loads(self.path.read_text())
        self.assertEqual(payload['version'], 1)
        self.assertEqual(payload['corner_order'], list(bw.CORNER_ORDER))
        self.assertEqual(payload['floor_order'], list(bw.FLOORS))
        self.assertEqual(payload['model_filename'], self.model.name)
        self.assertEqual(len(payload['model_sha256']), 64)
        self.assertFalse((self.root / 'calibration.json').exists())

    def test_empty_model_binding_is_explicit_not_a_load_bypass(self):
        bw.save_template(self.path, template())
        self.assertEqual(bw.load_template(self.path, SIZE).model_sha256, '')
        with self.assertRaisesRegex(ValueError, 'fingerprint'):
            bw.load_template(self.path, SIZE, model_path=self.model)

    def test_incompatible_size_orientation_or_model_rejected(self):
        for args in [((720, 1280), True, self.model), (SIZE, False, self.model), (SIZE, True, '')]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                bw.load_template(self.path, *args)
        renamed = self.root / 'renamed.pt'
        renamed.write_bytes(self.model.read_bytes())
        with self.assertRaisesRegex(ValueError, 'fingerprint'):
            bw.load_template(self.path, SIZE, True, renamed)
        self.model.write_bytes(b'changed-weights')
        with self.assertRaisesRegex(ValueError, 'fingerprint'):
            bw.load_template(self.path, SIZE, True, self.model)

    def test_corrupt_payloads_and_metadata_rejected(self):
        original = json.loads(self.path.read_text())
        changes = [('version', True), ('version', 2), ('corner_order', list(reversed(bw.CORNER_ORDER))),
                   ('floor_order', list(reversed(bw.FLOORS))), ('rgb_mirror', 1), ('rgb_size', [1280.0, 720]),
                   ('created', ''), ('created', 'yesterday'), ('model_sha256', 'z' * 64),
                   ('model_filename', '../building_d.pt'), ('centers', dict(PLANE_CENTERS, **{'1F': (0.5, .7)})),
                   ('reference_corners', CORNERS[::-1].tolist()), ('extra', 1)]
        for key, value in changes:
            payload = dict(original, **{key: value})
            self.path.write_text(json.dumps(payload))
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                bw.load_template(self.path, SIZE, True, self.model)
        for text in ['null', '[]', '{', '{"version":1,"version":1}',
                     '{"version":NaN}', '{"version":Infinity}', '{"version":-Infinity}']:
            self.path.write_text(text)
            with self.subTest(text=text), self.assertRaises(ValueError):
                bw.load_template(self.path, SIZE, True, self.model)
        payload = dict(original)
        del payload['created']
        self.path.write_text(json.dumps(payload))
        with self.assertRaises(ValueError):
            bw.load_template(self.path, SIZE, True, self.model)

    def test_atomic_replace_failure_preserves_old_file_and_cleans_temporary(self):
        original = self.path.read_bytes()
        with patch.object(bw.os, 'replace', side_effect=OSError('simulated replace failure')):
            with self.assertRaises(OSError):
                bw.save_template(self.path, self.value)
        self.assertEqual(self.path.read_bytes(), original)
        self.assertEqual(sorted(p.name for p in self.root.iterdir()), ['building_d.pt', 'window_centers.json'])

    def test_atomic_fsync_failure_preserves_old_file(self):
        original = self.path.read_bytes()
        with patch.object(bw.os, 'fsync', side_effect=OSError('simulated disk failure')):
            with self.assertRaises(OSError):
                bw.save_template(self.path, self.value)
        self.assertEqual(self.path.read_bytes(), original)
        self.assertFalse(list(self.root.glob('.*.tmp')))

    def test_invalid_save_does_not_touch_previous_file(self):
        original = self.path.read_bytes()
        for bad in [None, replace(self.value, created='bad'), replace(self.value, model_sha256=''),
                    replace(self.value, centers=dict(PLANE_CENTERS, **{'4F': (.5, 0)}))]:
            with self.assertRaises(ValueError):
                bw.save_template(self.path, bad)
            self.assertEqual(self.path.read_bytes(), original)


class Tensor:
    def __init__(self, value):
        self.value = np.asarray(value)

    def cpu(self):
        return self

    def numpy(self):
        return self.value


def result(corners=None, box_conf=.8, corner_conf=None, box=None):
    return SimpleNamespace(
        boxes=SimpleNamespace(xyxy=Tensor([box if box is not None else [80, 30, 750, 680]]),
                              conf=Tensor([box_conf])),
        keypoints=SimpleNamespace(xy=Tensor([CORNERS if corners is None else corners]),
                                  conf=Tensor([[.9] * 4 if corner_conf is None else corner_conf])))


class PoseTests(unittest.TestCase):
    def test_pose_payload_types_and_threshold_inclusive(self):
        payload = bw.BuildingPoseDetector._extract_boxes(result(box_conf=.5, corner_conf=[.5] * 4))
        self.assertEqual(len(payload), 1)
        detection = payload[0]
        self.assertIsInstance(detection, bw.BuildingDetection)
        self.assertEqual(detection.confidence, .5)
        self.assertEqual(detection.corner_conf, (.5,) * 4)
        self.assertEqual(detection.box, (80., 30., 750., 680.))
        np.testing.assert_array_equal(detection.corners, CORNERS)

    def test_invalid_pose_detections_filtered(self):
        bad = [result(box_conf=.499), result(box_conf=np.nan), result(box_conf=1.1),
               result(corner_conf=[.9, .9, .49, .9]), result(corner_conf=[np.nan] * 4),
               result(corner_conf=[np.inf] * 4), result(corners=CORNERS[::-1]),
               result(corners=CORNERS[[0, 2, 1, 3]]), result(corners=np.full((4, 2), np.nan)),
               result(box=[80, 30, 20, 680]), result(box=[80, np.inf, 750, 680]),
               SimpleNamespace(), SimpleNamespace(boxes=None, keypoints=None)]
        missing_conf = result()
        missing_conf.keypoints.conf = None
        bad.append(missing_conf)
        wrong_shape = result()
        wrong_shape.keypoints.xy = Tensor([[[1, 2]] * 3])
        bad.append(wrong_shape)
        for item in bad:
            with self.subTest(item=item):
                self.assertEqual(bw.BuildingPoseDetector._extract_boxes(item), [])

    def test_one_bad_detection_does_not_discard_valid_neighbors(self):
        item = result()
        item.boxes.xyxy = Tensor([[80, 30, 750, 680]] * 3)
        item.boxes.conf = Tensor([.9, .2, .8])
        item.keypoints.xy = Tensor([CORNERS] * 3)
        item.keypoints.conf = Tensor([[.9] * 4] * 3)
        self.assertEqual(len(bw.BuildingPoseDetector._extract_boxes(item)), 2)

    def test_constructor_validation_closes_real_parent_worker_on_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / 'model.pt'
            model_path.write_bytes(b'fake')
            for task, shape in [('detect', [4, 3]), ('pose', [4, 2]), ('pose', None), ('pose', [5, 3])]:
                model = SimpleNamespace(task=task, names={0: 'building'},
                                        model=SimpleNamespace(yaml={'kpt_shape': shape}))
                seen = []
                close = bw.BuildingPoseDetector.close

                def track_close(detector):
                    close(detector)
                    seen.append(detector)

                with patch.dict(sys.modules, {'ultralytics': SimpleNamespace(YOLO=Mock(return_value=model))}), \
                        patch.object(yolo_detect, 'resolve_device', return_value='cpu'), \
                        patch.object(bw.BuildingPoseDetector, 'close', track_close):
                    with self.subTest(task=task, shape=shape), self.assertRaises(ValueError):
                        bw.BuildingPoseDetector(model_path)
                self.assertEqual(len(seen), 1)
                self.assertFalse(seen[0]._thread.is_alive())

    def test_constructor_defaults_and_inherited_thread_submit_latest_close(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / 'model.pt'
            model_path.write_bytes(b'fake')
            model = SimpleNamespace(task='pose', names={0: 'building'},
                                    model=SimpleNamespace(kpt_shape=[4, 3]),
                                    predict=Mock(return_value=[result()]))
            with patch.dict(sys.modules, {'ultralytics': SimpleNamespace(YOLO=Mock(return_value=model))}), \
                    patch.object(yolo_detect, 'resolve_device', return_value='cpu'):
                detector = bw.BuildingPoseDetector(model_path)
            try:
                self.assertEqual(detector.imgsz, 960)
                self.assertEqual(detector.conf, .5)
                self.assertIsNone(detector.latest())
                # Event notification avoids sleeping/polling while testing the real worker.
                import threading
                finished = threading.Event()
                original_condition = detector._condition
                original_lock = detector._lock
                # Signal exactly when the worker publishes its result via lock release.
                class PublishingLock:
                    def __enter__(self):
                        original_lock.acquire()
                        return self

                    def __exit__(self, *args):
                        if detector._result is not None:
                            finished.set()
                        original_lock.release()

                detector._lock = PublishingLock()
                detector.submit(np.zeros((10, 10, 3), np.uint8), 42)
                self.assertTrue(finished.wait(2), 'mock inference timed out')
                frame_id, payload, infer_ms = detector.latest()
                self.assertEqual(frame_id, 42)
                self.assertIsInstance(payload[0], bw.BuildingDetection)
                self.assertGreaterEqual(infer_ms, 0)
                self.assertEqual(model.predict.call_args.kwargs['imgsz'], 960)
                self.assertEqual(model.predict.call_args.kwargs['device'], 'cpu')
                self.assertIs(detector._condition, original_condition)
            finally:
                detector.close()
            self.assertFalse(detector._thread.is_alive())

    def test_missing_local_model_rejected_before_parent_no_download(self):
        with patch.object(yolo_detect.YoloDetector, '__init__') as parent:
            with self.assertRaises(ValueError):
                bw.BuildingPoseDetector('/definitely/not/a/local/model.pt')
        parent.assert_not_called()


if __name__ == '__main__':
    unittest.main()
