#!/usr/bin/env python3
"""Pure geometry/JSON regression tests: no Tk, ROS or camera initialization."""
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import thermal_calibration as tc


RGB_SIZE = (1280, 720)
SYNTHETIC_H = np.array([[21.0, 1.5, 110.25], [0.75, 18.0, 60.5],
                        [0.002, 0.001, 1.0]])
IR_POINTS = [(0, 0), (0, 31), (23, 31), (23, 0), (8.25, 9.75), (17.5, 22.25)]


def records(H=SYNTHETIC_H):
    result = []
    for row, col in IR_POINTS:
        q = H @ np.array([col, row, 1.0])
        result.append([row, col, q[0] / q[2], q[1] / q[2]])
    return result


class GeometryTests(unittest.TestCase):
    def test_project_order_and_subpixel_precision(self):
        H = np.array([[2, 3, 10], [5, 7, 20], [0, 0, 1]], dtype=float)
        u, v = tc.project(H, 4.25, 1.5)
        self.assertEqual((u, v), (23.0, 51.75))
        self.assertNotEqual(tc.project(H, 1.5, 4.25), (u, v))

    def test_fit_recovers_synthetic_homography_and_records_row_col(self):
        points = records()
        result = tc.fit_calibration(points, RGB_SIZE, note='同一目标平面')
        np.testing.assert_allclose(result.H, SYNTHETIC_H, rtol=2e-6, atol=2e-5)
        self.assertLess(result.rms_px, 2e-5)
        self.assertEqual(result.ir_size, (32, 24))
        self.assertEqual(result.rgb_size, RGB_SIZE)
        self.assertEqual(result.points[4][:2], (8.25, 9.75))
        self.assertEqual(result.note, '同一目标平面')
        self.assertTrue(result.mirror_x)
        self.assertTrue(result.created)
        u, v = tc.project(result.H, 17.125, 3.25)
        expected = SYNTHETIC_H @ [17.125, 3.25, 1]
        np.testing.assert_allclose((u, v), expected[:2] / expected[2], atol=2e-5)

    def test_find_homography_is_non_ransac_col_row(self):
        original = tc.cv2.findHomography
        with patch.object(tc.cv2, 'findHomography', wraps=original) as fit:
            tc.fit_calibration(records(), RGB_SIZE)
        args = fit.call_args.args
        np.testing.assert_equal(args[0], np.asarray(records())[:, [1, 0]])
        np.testing.assert_equal(args[1], np.asarray(records())[:, [2, 3]])
        self.assertEqual(args[2], 0)

    def test_exactly_four_corners_fit(self):
        result = tc.fit_calibration(records()[:4], RGB_SIZE)
        self.assertEqual(len(result.points), 4)
        self.assertLess(result.rms_px, 2e-5)

    def test_preview_scale_and_mirror_metadata_do_not_change_geometry(self):
        first = tc.fit_calibration(records(), RGB_SIZE, scale=1)
        second = tc.fit_calibration(records(), RGB_SIZE, scale=24, mirror_x=False)
        np.testing.assert_array_equal(first.H, second.H)
        self.assertEqual(first.rms_px, second.rms_px)
        self.assertEqual(second.scale, 24)
        self.assertFalse(second.mirror_x)

    def test_rms_is_mean_squared_euclidean_not_per_component(self):
        self.assertEqual(tc.rms(np.eye(3), [[1, 2, 5, 5], [2, 4, 4, 2]]),
                         np.sqrt(12.5))

    def test_rejects_too_few_wrong_shape_nonfinite_and_outside_points(self):
        cases = [records()[:3], [], [[0, 0, 1, 1, 90]] * 4,
                 [['0', '0', '1', '1']] * 4]
        for column, value in [(0, -0.1), (0, 24), (1, 32), (2, 1280),
                              (3, 720), (2, np.nan), (3, np.inf)]:
            points = records()
            points[0][column] = value
            cases.append(points)
        for points in cases:
            with self.subTest(points=points), self.assertRaises(ValueError):
                tc.fit_calibration(points, RGB_SIZE)

    def test_rejects_duplicate_ir_or_rgb_even_when_other_coords_differ(self):
        for columns in ([0, 1], [2, 3]):
            points = records()
            for column in columns:
                points[1][column] = points[0][column]
            with self.subTest(columns=columns), self.assertRaisesRegex(ValueError, 'duplicate'):
                tc.fit_calibration(points, RGB_SIZE)

    def test_rejects_collinear_ir_or_rgb(self):
        for columns in ([0, 1], [2, 3]):
            points = records()
            for i, point in enumerate(points):
                point[columns[0]], point[columns[1]] = i, 2 * i
            with self.subTest(columns=columns), self.assertRaisesRegex(ValueError, 'collinear'):
                tc.fit_calibration(points, RGB_SIZE)

    def test_invalid_fit_metadata(self):
        for kwargs in ({'scale': 0}, {'scale': -1}, {'scale': np.inf},
                       {'scale': '12'}, {'scale': True}, {'mirror_x': 1},
                       {'note': None}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                tc.fit_calibration(records(), RGB_SIZE, **kwargs)
        for size in ((0, 720), (1280,), (1280.0, 720), (True, 720), None):
            with self.subTest(size=size), self.assertRaises(ValueError):
                tc.fit_calibration(records(), size)

    def test_solver_failure_and_singular_result_become_valueerror(self):
        for H in (None, np.zeros((3, 3)), np.ones((3, 3))):
            with patch.object(tc.cv2, 'findHomography', return_value=(H, None)):
                with self.assertRaises(ValueError):
                    tc.fit_calibration(records(), RGB_SIZE)
        with patch.object(tc.cv2, 'findHomography', side_effect=tc.cv2.error('bad fit')):
            with self.assertRaisesRegex(ValueError, 'fit failed'):
                tc.fit_calibration(records(), RGB_SIZE)

    def test_projection_rejects_invalid_h_and_nonfinite_coordinates(self):
        for H in (np.eye(2), np.zeros((3, 3)), [[1, 0, 0], [0, 1, 0], [0, 0, np.nan]],
                  np.full((3, 3), np.inf), [['1', '0', '0']] * 3):
            with self.subTest(H=H), self.assertRaises(ValueError):
                tc.project(H, 1, 2)
        for col, row in ((np.nan, 2), (1, np.inf), ('1', 2), (True, 2)):
            with self.subTest(col=col, row=row), self.assertRaises(ValueError):
                tc.project(np.eye(3), col, row)

    def test_projection_rejects_zero_denominator_and_nonfinite_result(self):
        horizon = [[1, 0, 0], [0, 1, 0], [1, 0, -2]]
        with self.assertRaisesRegex(ValueError, 'denominator'):
            tc.project(horizon, 2, 3)
        overflow = [[1, 1, 0], [0, 1, 0], [0, 0, 1]]
        with self.assertRaisesRegex(ValueError, 'non-finite'):
            tc.project(overflow, 1.7e308, 1.7e308)

    def test_project_is_invariant_under_homogeneous_scale_and_allows_offscreen(self):
        expected = tc.project(SYNTHETIC_H, 10.5, 4.25)
        for scale in (1e-200, -1e-100, 1e100, -1e200):
            with self.subTest(scale=scale):
                np.testing.assert_allclose(tc.project(SYNTHETIC_H * scale, 10.5, 4.25),
                                           expected, atol=1e-12)
        self.assertEqual(tc.project(np.eye(3), -5.25, 1200.5), (-5.25, 1200.5))


class PersistenceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / '标定.json'
        self.calibration = tc.fit_calibration(records(), RGB_SIZE, note='暖宝宝 6点')
        tc.save_calibration(self.path, self.calibration)
        with self.path.open(encoding='utf-8') as stream:
            self.payload = json.load(stream)

    def write_payload(self, payload):
        with self.path.open('w', encoding='utf-8') as stream:
            json.dump(payload, stream, ensure_ascii=False)

    def test_roundtrip_schema_and_subpixel_coordinates(self):
        self.assertEqual(self.payload['version'], 1)
        self.assertEqual(self.payload['points'], [list(row) for row in self.calibration.points])
        self.assertTrue(all(len(row) == 4 for row in self.payload['points']))
        result = tc.load_calibration(self.path, RGB_SIZE)
        np.testing.assert_allclose(result.H, self.calibration.H)
        self.assertEqual(result.points, self.calibration.points)
        self.assertEqual(result.created, self.calibration.created)
        self.assertEqual(result.note, self.calibration.note)
        self.assertAlmostEqual(result.rms_px, self.calibration.rms_px, places=10)

    def test_scale_roundtrip_independent_of_geometry(self):
        tc.save_calibration(self.path, replace(self.calibration, scale=2.5))
        result = tc.load_calibration(self.path, RGB_SIZE)
        self.assertEqual(result.scale, 2.5)
        np.testing.assert_allclose(result.H, self.calibration.H)

    def test_stored_rms_is_not_trusted_and_high_error_is_allowed(self):
        self.payload['H'][0][2] += 100
        self.payload['rms_px'] = 0
        self.write_payload(self.payload)
        result = tc.load_calibration(self.path, RGB_SIZE)
        self.assertGreater(result.rms_px, 80)
        self.assertAlmostEqual(result.rms_px, tc.rms(result.H, result.points))
        tc.save_calibration(self.path, replace(result, rms_px=0))
        with self.path.open(encoding='utf-8') as stream:
            self.assertGreater(json.load(stream)['rms_px'], 80)

    def test_rejects_current_session_mismatch(self):
        for kwargs, message in [({'rgb_size': (640, 480)}, 'rgb_size mismatch'),
                                ({'rgb_size': RGB_SIZE, 'ir_size': (24, 32)}, 'ir_size mismatch'),
                                ({'rgb_size': RGB_SIZE, 'mirror_x': False}, 'mirror_x mismatch')]:
            with self.subTest(kwargs=kwargs), self.assertRaisesRegex(ValueError, message):
                tc.load_calibration(self.path, **kwargs)

    def test_rejects_invalid_schema_fields(self):
        cases = [('version', 2), ('version', True), ('version', 1.0),
                 ('H', np.zeros((3, 3)).tolist()), ('H', None),
                 ('points', records()[:3]), ('points', [row + [33] for row in records()]),
                 ('ir_size', [24, 32]), ('rgb_size', [640, 480]),
                 ('rgb_size', [1280.0, 720]), ('mirror_x', 1),
                 ('mirror_x', 'true'), ('scale', 0), ('scale', '12'),
                 ('rms_px', -1), ('rms_px', '0'), ('created', 'bad date'), ('note', None)]
        for name, value in cases:
            payload = dict(self.payload, **{name: value})
            self.write_payload(payload)
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                tc.load_calibration(self.path, RGB_SIZE)

    def test_rejects_missing_required_fields(self):
        for name in self.payload:
            payload = dict(self.payload)
            del payload[name]
            self.write_payload(payload)
            with self.subTest(name=name), self.assertRaises(ValueError):
                tc.load_calibration(self.path, RGB_SIZE)

    def test_rejects_invalid_json_nonfinite_constants_and_nonobject(self):
        for text in ('{', '[]', 'null', '{"version": NaN}', '{"version": Infinity}'):
            self.path.write_text(text, encoding='utf-8')
            with self.subTest(text=text), self.assertRaises(ValueError):
                tc.load_calibration(self.path, RGB_SIZE)

    def test_load_revalidates_geometry_and_horizon(self):
        duplicate = records()
        duplicate[1] = duplicate[0]
        collinear = [[i, i, i, 2 * i] for i in range(6)]
        outside = records()
        outside[0][0] = 24
        for points in (duplicate, collinear, outside):
            self.write_payload(dict(self.payload, points=points))
            with self.subTest(points=points), self.assertRaises(ValueError):
                tc.load_calibration(self.path, RGB_SIZE)
        self.payload['H'] = [[1, 0, 0], [0, 1, 0], [1, 0, -31]]
        self.write_payload(self.payload)
        with self.assertRaisesRegex(ValueError, 'denominator'):
            tc.load_calibration(self.path, RGB_SIZE)

    def test_optional_extra_ui_fields_are_ignored(self):
        self.payload['temperatures'] = [30] * len(records())
        self.write_payload(self.payload)
        self.assertEqual(len(tc.load_calibration(self.path, RGB_SIZE).points), 6)

    def test_save_failure_preserves_original_and_cleans_temporary(self):
        original = self.path.read_bytes()
        for operation in ('replace', 'fsync'):
            with patch.object(tc.os, operation, side_effect=OSError('simulated disk failure')):
                with self.subTest(operation=operation), self.assertRaises(OSError):
                    tc.save_calibration(self.path, replace(self.calibration, note='new'))
            self.assertEqual(self.path.read_bytes(), original)
            self.assertEqual(list(self.path.parent.iterdir()), [self.path])

    def test_invalid_save_never_touches_original(self):
        original = self.path.read_bytes()
        for invalid in (replace(self.calibration, H=np.zeros((3, 3))),
                        replace(self.calibration, points=()),
                        replace(self.calibration, scale=-1)):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                tc.save_calibration(self.path, invalid)
            self.assertEqual(self.path.read_bytes(), original)

    def test_missing_file_is_clear_oserror_not_silent_fallback(self):
        with self.assertRaises(FileNotFoundError):
            tc.load_calibration(self.path.parent / 'missing.json', RGB_SIZE)


if __name__ == '__main__':
    unittest.main()
