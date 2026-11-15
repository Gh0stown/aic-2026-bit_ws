#!/usr/bin/env python3
"""Pure math tests for dial_math: no camera, ROS, model, Tk or image files."""
import json
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dial_math as dm


# Synthetic fixture only.  It is deliberately non-equidistant and denser at
# the wet end.  It is NOT a real deli No.0011 calibration and must never be
# used as a production/default table.
SYNTHETIC_DEG = (
    10.0, 25.0, 48.0, 80.0, 120.0, 170.0, 230.0, 295.0, 340.0, 350.0, 355.0,
)
SYNTHETIC_VAL = (
    0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0,
)
SYNTHETIC_VAL_INT = tuple(range(0, 101, 10))
SYNTHETIC_MID_DEG = (
    17.5, 36.5, 64.0, 100.0, 145.0, 200.0, 262.5, 317.5, 345.0, 352.5,
)
SYNTHETIC_MID_VAL = (
    5.0, 15.0, 25.0, 35.0, 45.0, 55.0, 65.0, 75.0, 85.0, 95.0,
)

# A04 legal narrow-interval regressions.  These three tables all pass
# validate_scale; the first/last intervals are narrower than the 1e-9 degree
# endpoint rounding allowance.  They must still return their node values and
# normal interpolation inside the closed interval.
NARROW_HEAD_DEG = (
    0.0, 5e-10, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0,
)
NARROW_TAIL_DEG = (
    0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0,
    270.0 + 5e-10,
)
TINY_DEG = tuple(i * 1e-11 for i in range(11))


class ValidateScaleTests(unittest.TestCase):
    def test_accepts_synthetic_list_tuple_and_integer_values(self):
        self.assertIsNone(dm.validate_scale(list(SYNTHETIC_DEG), list(SYNTHETIC_VAL)))
        self.assertIsNone(dm.validate_scale(SYNTHETIC_DEG, SYNTHETIC_VAL))
        self.assertIsNone(dm.validate_scale(list(SYNTHETIC_DEG), SYNTHETIC_VAL_INT))

    def test_rejects_container_shape_and_nested_elements(self):
        bad_tables = (
            'not-a-table',
            None,
            SYNTHETIC_DEG[:10],
            SYNTHETIC_DEG + (360.0,),
            (list(SYNTHETIC_DEG[:1]),) + SYNTHETIC_DEG[1:],
            (True,) + SYNTHETIC_DEG[1:],
            ('10',) + SYNTHETIC_DEG[1:],
            (None,) + SYNTHETIC_DEG[1:],
            (1 + 2j,) + SYNTHETIC_DEG[1:],
        )
        for bad in bad_tables:
            with self.subTest(table=bad):
                with self.assertRaises(ValueError):
                    dm.validate_scale(bad, SYNTHETIC_VAL)
        with self.assertRaises(ValueError):
            dm.validate_scale(SYNTHETIC_DEG, ('0',) + SYNTHETIC_VAL[1:])

    def test_rejects_nonfinite_and_overflow_values(self):
        nan_deg = SYNTHETIC_DEG[:10] + (float('nan'),)
        inf_deg = SYNTHETIC_DEG[:10] + (float('inf'),)
        huge_deg = SYNTHETIC_DEG[:10] + (10 ** 400,)
        nan_val = SYNTHETIC_VAL[:10] + (float('nan'),)
        inf_val = SYNTHETIC_VAL[:10] + (float('inf'),)
        for deg, val in ((nan_deg, SYNTHETIC_VAL), (inf_deg, SYNTHETIC_VAL),
                         (huge_deg, SYNTHETIC_VAL), (SYNTHETIC_DEG, nan_val),
                         (SYNTHETIC_DEG, inf_val)):
            with self.subTest(deg=deg, val=val):
                with self.assertRaises(ValueError):
                    dm.validate_scale(deg, val)

    def test_rejects_angle_order_duplicates_and_bad_span(self):
        decreasing = tuple(reversed(SYNTHETIC_DEG))
        duplicate = (10.0, 10.0, 48.0, 80.0, 120.0, 170.0, 230.0,
                     295.0, 340.0, 350.0, 355.0)
        span_360 = tuple(float(value) for value in range(0, 361, 36))
        span_400 = tuple(float(value) for value in range(0, 401, 40))
        for deg in (decreasing, duplicate, span_360, span_400):
            with self.subTest(deg=deg):
                with self.assertRaises(ValueError):
                    dm.validate_scale(deg, SYNTHETIC_VAL)

        overflow_span = (
            -1e308, -9e307, -8e307, -7e307, -6e307,
            -5e307, -4e307, -3e307, -2e307, -1e307, 1e308,
        )
        with self.assertRaises(ValueError):
            dm.validate_scale(overflow_span, SYNTHETIC_VAL)

    def test_rejects_wrong_values(self):
        bad_values = (
            (0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 99.0),
            (0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.5),
            (0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 100.0, 90.0),
            (0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, True),
        )
        for values in bad_values:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    dm.validate_scale(SYNTHETIC_DEG, values)

    def test_does_not_mutate_inputs(self):
        deg = list(SYNTHETIC_DEG)
        val = list(SYNTHETIC_VAL)
        dm.validate_scale(deg, val)
        self.assertEqual(deg, list(SYNTHETIC_DEG))
        self.assertEqual(val, list(SYNTHETIC_VAL))


class WrapDegTests(unittest.TestCase):
    def test_wrap_known_values(self):
        cases = (
            (0.0, 0.0), (360.0, 0.0), (-360.0, 0.0),
            (-1.0, 359.0), (359.5, 359.5), (1.0, 1.0),
            (720.0 + 45.0, 45.0), (-720.0 - 45.0, 315.0),
        )
        for angle, expected in cases:
            with self.subTest(angle=angle):
                self.assertAlmostEqual(dm.wrap_deg(angle), expected, places=12)

    def test_wrap_range_and_multi_turn(self):
        for angle in range(-2000, 2001, 7):
            result = dm.wrap_deg(angle)
            self.assertGreaterEqual(result, 0.0)
            self.assertLess(result, 360.0)
        self.assertIsInstance(dm.wrap_deg(10), float)

    def test_wrap_rejects_invalid_values(self):
        for bad in (float('nan'), float('inf'), float('-inf'), True, False,
                    '10', [10], None, 1 + 2j, 10 ** 400):
            with self.subTest(value=bad):
                with self.assertRaises(ValueError):
                    dm.wrap_deg(bad)


class RelativeAngleTests(unittest.TestCase):
    def test_formula_known_values(self):
        self.assertAlmostEqual(dm.relative_angle(140, 10, 100), 130.0, places=12)
        self.assertAlmostEqual(dm.relative_angle(120, -10, 100), 130.0, places=12)
        self.assertAlmostEqual(dm.relative_angle(5, 350, 10), 15.0, places=12)
        self.assertAlmostEqual(dm.relative_angle(355, 5, 350), 350.0, places=12)

    def test_roll_cancels_including_full_turns_and_negative_rotation(self):
        expected = 130.0
        cases = (
            (140.0, 10.0, 100.0),
            (120.0, -10.0, 100.0),
            (500.0, 370.0, 100.0),
            (-220.0, -350.0, 100.0),
            (140.0 + 720.0, 10.0 + 720.0, 100.0),
        )
        for needle, roll, zero in cases:
            with self.subTest(needle=needle, roll=roll):
                self.assertAlmostEqual(
                    dm.relative_angle(needle, roll, zero), expected, places=12)

    def test_relative_rejects_invalid_and_overflow(self):
        for bad in (float('nan'), float('inf'), True, '10', [10], None):
            for position in range(3):
                args = [140.0, 10.0, 100.0]
                args[position] = bad
                with self.subTest(position=position, value=bad):
                    with self.assertRaises(ValueError):
                        dm.relative_angle(*args)
        with self.assertRaises(ValueError):
            dm.relative_angle(1e308, -1e308, 0.0)


class LookupValueTests(unittest.TestCase):
    def test_nodes_and_hand_computed_midpoints(self):
        for degree, expected in zip(SYNTHETIC_DEG, SYNTHETIC_VAL):
            with self.subTest(degree=degree):
                self.assertEqual(
                    dm.lookup_value(degree, SYNTHETIC_DEG, SYNTHETIC_VAL), expected)
        for degree, expected in zip(SYNTHETIC_MID_DEG, SYNTHETIC_MID_VAL):
            with self.subTest(degree=degree):
                self.assertAlmostEqual(
                    dm.lookup_value(degree, SYNTHETIC_DEG, SYNTHETIC_VAL),
                    expected, places=12)

    def test_wet_end_dense_segment_values_are_hand_computed(self):
        self.assertAlmostEqual(
            dm.lookup_value(346.5, SYNTHETIC_DEG, SYNTHETIC_VAL), 86.5, places=12)
        self.assertAlmostEqual(
            dm.lookup_value(354.0, SYNTHETIC_DEG, SYNTHETIC_VAL), 98.0, places=12)

    def test_closed_endpoints_and_outside_rounding_absorption(self):
        first = SYNTHETIC_DEG[0]
        last = SYNTHETIC_DEG[-1]
        cases = (
            (first, 0.0), (last, 100.0),
            (first - 5e-10, 0.0), (last + 5e-10, 100.0),
            (first - 1e-9, 0.0), (last + 1e-9, 100.0),
        )
        for degree, expected in cases:
            with self.subTest(degree=degree):
                self.assertAlmostEqual(
                    dm.lookup_value(degree, SYNTHETIC_DEG, SYNTHETIC_VAL),
                    expected, places=12)

    def test_inside_near_endpoints_is_not_absorbed(self):
        first = SYNTHETIC_DEG[0]
        last = SYNTHETIC_DEG[-1]
        near_first = dm.lookup_value(
            first + 5e-10, SYNTHETIC_DEG, SYNTHETIC_VAL)
        self.assertNotEqual(near_first, 0.0)
        self.assertGreater(near_first, 0.0)
        self.assertLess(near_first, 1.0)
        near_last = dm.lookup_value(
            last - 5e-10, SYNTHETIC_DEG, SYNTHETIC_VAL)
        self.assertNotEqual(near_last, 100.0)
        self.assertGreater(near_last, 99.0)
        self.assertLess(near_last, 100.0)

    def test_true_out_of_range_and_blank_area_return_none(self):
        first = SYNTHETIC_DEG[0]
        last = SYNTHETIC_DEG[-1]
        for degree in (first - 2e-9, last + 2e-9, first - 0.5, last + 0.5,
                       0.0, -350.0, 370.0, 720.0):
            with self.subTest(degree=degree):
                self.assertIsNone(
                    dm.lookup_value(degree, SYNTHETIC_DEG, SYNTHETIC_VAL))
        self.assertIsNone(
            dm.lookup_value(first - 0.5, SYNTHETIC_DEG, SYNTHETIC_VAL))
        self.assertIsNone(
            dm.lookup_value(last + 0.5, SYNTHETIC_DEG, SYNTHETIC_VAL))

    def test_no_automatic_wrap_and_no_endpoint_clamping(self):
        self.assertIsNone(
            dm.lookup_value(10.0 + 360.0, SYNTHETIC_DEG, SYNTHETIC_VAL))
        self.assertIsNone(
            dm.lookup_value(10.0 - 360.0, SYNTHETIC_DEG, SYNTHETIC_VAL))
        self.assertIsNone(
            dm.lookup_value(355.0 + 360.0, SYNTHETIC_DEG, SYNTHETIC_VAL))
        self.assertIsNone(
            dm.lookup_value(355.0 - 360.0, SYNTHETIC_DEG, SYNTHETIC_VAL))
        self.assertIsNotNone(
            dm.lookup_value(10.0, SYNTHETIC_DEG, SYNTHETIC_VAL))

    def test_lookup_rejects_invalid_angle_and_table(self):
        for bad in (float('nan'), float('inf'), float('-inf'), True, '10',
                    [10], None, 10 ** 400):
            with self.subTest(angle=bad):
                with self.assertRaises(ValueError):
                    dm.lookup_value(bad, SYNTHETIC_DEG, SYNTHETIC_VAL)
        with self.assertRaises(ValueError):
            dm.lookup_value(100.0, SYNTHETIC_DEG[:10], SYNTHETIC_VAL)
        with self.assertRaises(ValueError):
            dm.lookup_value(
                100.0, SYNTHETIC_DEG,
                SYNTHETIC_VAL[:10] + (float('nan'),))

    def test_lookup_is_monotone_between_nodes(self):
        rng = random.Random(20261115)
        angles = sorted(
            rng.uniform(SYNTHETIC_DEG[0], SYNTHETIC_DEG[-1]) for _ in range(300))
        values = [
            dm.lookup_value(angle, SYNTHETIC_DEG, SYNTHETIC_VAL)
            for angle in angles
        ]
        self.assertTrue(all(value is not None for value in values))
        for left, right in zip(values, values[1:]):
            self.assertLessEqual(left, right)


class NarrowEndpointRegressionTests(unittest.TestCase):
    def test_all_three_legal_narrow_tables_pass_validate_scale(self):
        for degrees in (NARROW_HEAD_DEG, NARROW_TAIL_DEG, TINY_DEG):
            with self.subTest(degrees=degrees):
                self.assertIsNone(dm.validate_scale(degrees, SYNTHETIC_VAL))

    def test_first_narrow_interval_keeps_node_and_interpolates(self):
        self.assertEqual(
            dm.lookup_value(5e-10, NARROW_HEAD_DEG, SYNTHETIC_VAL), 10.0)
        self.assertAlmostEqual(
            dm.lookup_value(2.5e-10, NARROW_HEAD_DEG, SYNTHETIC_VAL),
            5.0, places=12)
        self.assertEqual(
            dm.lookup_value(30.0, NARROW_HEAD_DEG, SYNTHETIC_VAL), 20.0)
        self.assertEqual(
            dm.lookup_value(0.0, NARROW_HEAD_DEG, SYNTHETIC_VAL), 0.0)
        self.assertEqual(
            dm.lookup_value(270.0, NARROW_HEAD_DEG, SYNTHETIC_VAL), 100.0)

    def test_last_narrow_interval_keeps_node_and_interpolates(self):
        self.assertEqual(
            dm.lookup_value(270.0, NARROW_TAIL_DEG, SYNTHETIC_VAL), 90.0)
        self.assertEqual(
            dm.lookup_value(270.0 + 5e-10, NARROW_TAIL_DEG, SYNTHETIC_VAL),
            100.0)
        self.assertAlmostEqual(
            dm.lookup_value(270.0 + 2.5e-10, NARROW_TAIL_DEG, SYNTHETIC_VAL),
            95.0, places=12)
        self.assertEqual(
            dm.lookup_value(0.0, NARROW_TAIL_DEG, SYNTHETIC_VAL), 0.0)

    def test_tiny_table_keeps_all_nodes_and_interpolates(self):
        for index, degree in enumerate(TINY_DEG):
            with self.subTest(index=index):
                self.assertEqual(
                    dm.lookup_value(degree, TINY_DEG, SYNTHETIC_VAL),
                    float(index * 10))
        self.assertAlmostEqual(
            dm.lookup_value(5.5e-11, TINY_DEG, SYNTHETIC_VAL), 55.0, places=12)
        self.assertEqual(
            dm.lookup_value(0.0, TINY_DEG, SYNTHETIC_VAL), 0.0)
        self.assertEqual(
            dm.lookup_value(1e-10, TINY_DEG, SYNTHETIC_VAL), 100.0)


class SeededPureAngleTests(unittest.TestCase):
    def test_seeded_angle_samples_verify_roll_periodicity_not_image_synthesis(self):
        """Pure angle arithmetic only: not image synthesis or accuracy."""
        for seed in (1, 2, 20261115):
            rng = random.Random(seed)
            for _ in range(100):
                zero = rng.uniform(-720.0, 720.0)
                base = rng.uniform(-720.0, 720.0)
                roll = rng.uniform(-720.0, 720.0)
                needle = base + roll
                expected = zero + ((base - zero) % 360.0)
                self.assertAlmostEqual(
                    dm.relative_angle(needle, roll, zero), expected, places=9)
                turns = rng.randint(-3, 3) * 360.0
                self.assertAlmostEqual(
                    dm.relative_angle(needle + turns, roll + turns, zero),
                    expected, places=9)

    def test_seeded_lookup_monotone_and_nodes_not_image_accuracy(self):
        """Pure table interpolation only: not image synthesis or accuracy."""
        rng = random.Random(20261116)
        angles = sorted(
            rng.uniform(SYNTHETIC_DEG[0], SYNTHETIC_DEG[-1]) for _ in range(300))
        values = [
            dm.lookup_value(angle, SYNTHETIC_DEG, SYNTHETIC_VAL)
            for angle in angles
        ]
        self.assertTrue(all(value is not None for value in values))
        for left, right in zip(values, values[1:]):
            self.assertLessEqual(left, right)
        for degree, expected in zip(SYNTHETIC_DEG, SYNTHETIC_VAL):
            self.assertEqual(
                dm.lookup_value(degree, SYNTHETIC_DEG, SYNTHETIC_VAL), expected)


class ModuleImportTests(unittest.TestCase):
    def test_import_has_no_forbidden_modules_threads_or_files(self):
        scripts = Path(__file__).resolve().parents[1] / 'scripts'
        code = (
            "import json, sys, threading\n"
            "sys.path.insert(0, {path!r})\n"
            "import dial_math\n"
            "forbidden = [name for name in "
            "('gauge', 'yolo_detect', 'ultralytics', 'rospy', 'tkinter', "
            "'numpy', 'cv2', 'PIL') "
            "if name in sys.modules]\n"
            "print(json.dumps({{'forbidden': forbidden, "
            "'threads': threading.active_count()}}))\n"
        ).format(path=str(scripts))
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                [sys.executable, '-B', '-c', code],
                cwd=tmp, capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads(result.stdout.strip())
            self.assertEqual(payload['forbidden'], [])
            self.assertEqual(payload['threads'], 1)
            self.assertEqual(os.listdir(tmp), [])


if __name__ == '__main__':
    unittest.main()
