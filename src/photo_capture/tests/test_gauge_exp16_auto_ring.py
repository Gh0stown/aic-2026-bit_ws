"""Synthetic scale/background checks on the opt-in Exp16 reader."""

import math
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gauge_exp16_vendor import CropReading, UnreadableGauge
from gauge_exp16_vendor.read_temperature_auto_ring import read_temperature_auto_ring


def _dial(radius, *, textured=False, ticks=range(81), color=True, value=18):
    """Keep crop dimensions fixed when the physical scale changes size."""
    height, width = 302, 396
    cx, cy = 192, 182
    ss = 3
    cx *= ss
    cy *= ss
    r = radius * ss
    image = np.full((height * ss, width * ss, 3), 185, dtype=np.uint8)

    def at(relative_radius, angle):
        a = math.radians(angle)
        return (round(cx + r * relative_radius * math.cos(a)),
                round(cy - r * relative_radius * math.sin(a)))

    def stroke(r1, r2, angle, bgr, width):
        cv2.line(image, at(r1, angle), at(r2, angle), bgr, width, cv2.LINE_AA)

    if textured:
        yy, xx = np.ogrid[:height * ss, :width * ss]
        image[:] = np.where(((xx // 18 + yy // 15) % 2)[..., None], (15, 25, 100), (230, 214, 20))
        # Include radial distractors with plausible spacing outside the face.
        for index in range(81):
            stroke(1.12, 1.30, 210 - 3 * index, (14, 14, 14), 2 * ss)
    cv2.circle(image, (cx, cy), round(1.12 * r), (33, 33, 33), -1, cv2.LINE_AA)
    cv2.circle(image, (cx, cy), round(1.095 * r), (215, 220, 218), -1, cv2.LINE_AA)
    if color:
        for begin, end, bgr in [(-30, 37, (40, 63, 205)), (37, 65, (69, 157, 36)),
                                (65, 210, (207, 87, 16))]:
            polygon = [at(.857, a) for a in range(begin, end + 1)]
            polygon += [at(.755, a) for a in range(end, begin - 1, -1)]
            cv2.fillPoly(image, [np.asarray(polygon, dtype=np.int32)], bgr, cv2.LINE_AA)
    for index in ticks:
        stroke(.878, .995 if index % 10 == 0 else .951, 210 - 3 * index,
               (31, 34, 34), max(3, round(r * .009)))
    stroke(-.115, .83, 210 - 3 * (value + 30), (22, 23, 23), max(4, round(r * .024)))
    cv2.circle(image, (cx, cy), round(r * .057), (26, 27, 27), -1, cv2.LINE_AA)
    cv2.circle(image, (cx, cy), round(r * .014), (181, 185, 182), -1, cv2.LINE_AA)
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


class AutoRingTests(unittest.TestCase):
    def test_scale_and_exterior_background_ab(self):
        measured = []
        for radius in (115, 158):
            ab = []
            for textured in (False, True):
                crop = _dial(radius, textured=textured)
                original = crop.copy()
                reading = read_temperature_auto_ring(crop)
                with self.subTest(radius=radius, textured=textured):
                    self.assertIsInstance(reading, CropReading)
                    self.assertEqual(reading.method, "auto-ring")
                    self.assertEqual(reading.value, 18.0)
                    self.assertGreaterEqual(reading.tick_count, 79)
                    self.assertLessEqual(math.dist(reading.center, (192, 182)), 3)
                    self.assertAlmostEqual(reading.radius, .916 * radius, delta=4)
                    np.testing.assert_array_equal(crop, original)
                ab.append(reading.radius)
            self.assertEqual(ab[0], ab[1])
            measured.append(ab[0])
        self.assertGreater(measured[1] - measured[0], 34)

    def test_missing_interior_ticks_do_not_shift_the_scale(self):
        crop = _dial(144, ticks=[i for i in range(81) if i not in (20, 48, 67)])
        reading = read_temperature_auto_ring(crop)
        self.assertEqual(reading.value, 18.0)
        self.assertLessEqual(reading.tick_count, 78)

    def test_arc_with_ticks_only_in_one_angular_sector_is_unreadable(self):
        crop = _dial(144, textured=True, ticks=range(26))
        with self.assertRaisesRegex(UnreadableGauge, "tick ring"):
            read_temperature_auto_ring(crop)

    def test_tick_count_does_not_compensate_for_a_missing_sector(self):
        # A high count on the rest of the arc cannot establish the missing sector.
        crop = _dial(144, ticks=[i for i in range(81) if not 34 <= i < 47])
        with self.assertRaisesRegex(UnreadableGauge, "tick ring"):
            read_temperature_auto_ring(crop)

    def test_strokes_without_a_visible_temperature_arc_are_unreadable(self):
        crop = _dial(144, color=False)
        with self.assertRaisesRegex(UnreadableGauge, "tick ring"):
            read_temperature_auto_ring(crop)

    def test_flat_crop_is_unreadable(self):
        crop = np.full((300, 400, 3), 220, dtype=np.uint8)
        with self.assertRaisesRegex(UnreadableGauge, "hub"):
            read_temperature_auto_ring(crop)


if __name__ == "__main__":
    unittest.main()
