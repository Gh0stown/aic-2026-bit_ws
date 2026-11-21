"""Synthetic evidence and rejection tests for the opt-in Exp16 RH reader."""

import math
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gauge_exp16_vendor import CropReading, UnreadableGauge
from gauge_exp16_vendor.read_humidity_auto_scale import (
    _Arc, _Tick, _occluded_tick, read_humidity_auto_scale,
)


# Deliberately nonlinear even between adjacent ticks; unrelated to camera pose.
_MAJOR = np.array([0, 32, 69, 99, 119, 145, 170, 192, 214, 238, 275])
_TICKS = np.r_[np.concatenate([a + (b - a) * np.array([0, .17, .39, .55, .78])
                               for a, b in zip(_MAJOR, _MAJOR[1:])]), _MAJOR[-1]]


def _dial(*, value=66.8, rotation=288, center=(130, 116), radius=94, sx=1.,
          missing=(), shorten=0, until=50, monochrome=False, dip=False,
          off_scale=None, ambiguous=False, second_bezel=False):
    height, width = 246, 274
    ss = 3
    image = np.full((height * ss, width * ss, 3), (222, 226, 224), dtype=np.uint8)
    bearings = rotation + _TICKS

    def at(r, bearing):
        ang = math.radians(bearing)
        return (round(ss * (center[0] + sx * radius * r * math.sin(ang))),
                round(ss * (center[1] - radius * r * math.cos(ang))))

    if second_bezel:
        for r in (1.15, 1.20):
            cv2.ellipse(image, at(0, 0), (round(radius * ss * r * sx), round(radius * ss * r)),
                        0, 0, 360, (82, 82, 82), 3 * ss, cv2.LINE_AA)
    if not monochrome:
        for lo, hi, color in ((max(shorten, 0), 20, (14, 48, 213)),
                              (20, 35, (32, 156, 21)), (35, min(until, 50), (200, 96, 14))):
            seq = np.linspace(bearings[lo], bearings[hi], 1 + round(bearings[hi] - bearings[lo]))
            points = [at(1.095, b) for b in seq] + [at(1., b) for b in seq[::-1]]
            cv2.fillPoly(image, [np.array(points, dtype=np.int32)], color, cv2.LINE_AA)
    for i, bearing in enumerate(bearings):
        if i in missing or not shorten <= i <= until:
            continue
        cv2.line(image, at(.816 if i % 5 == 0 else .866, bearing), at(.961, bearing),
                 (28, 34, 27), 4, cv2.LINE_AA)
    if dip:
        # A white glyph inside the band, not a new physical endpoint.
        b = (bearings[15] + bearings[16]) / 2
        cv2.line(image, at(.99, b), at(1.10, b), (220, 225, 223), 5, cv2.LINE_AA)
    hand = float(np.interp(value, np.arange(0, 101, 2), bearings))
    if off_scale is not None:
        hand = rotation + off_scale
    cv2.ellipse(image, at(-.25, hand), (6 * ss, 6 * ss), 0, 0, 360, (26, 27, 22), -1, cv2.LINE_AA)
    for bearing in (hand, hand + 180) if ambiguous else (hand,):
        polygon = [at(-.25, bearing), at(.18, bearing - 15), at(.765, bearing),
                   at(.18, bearing + 15)]
        cv2.fillPoly(image, [np.array(polygon, dtype=np.int32)], (27, 30, 26), cv2.LINE_AA)
    cv2.circle(image, at(0, 0), round(.103 * ss * radius), (25, 29, 22), -1, cv2.LINE_AA)
    cv2.circle(image, at(0, 0), round(.030 * ss * radius), (219, 227, 222), -1, cv2.LINE_AA)
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA), hand % 360


class HumidityAutoScaleTests(unittest.TestCase):
    def test_per_frame_geometry_and_nonlinear_tick_spacing(self):
        for rotation, center, radius, sx in [(288, (130, 116), 94, 1.),
                                              (38, (118, 103), 78, .91)]:
            with self.subTest(rotation=rotation, center=center, radius=radius):
                crop, bearing = _dial(rotation=rotation, center=center, radius=radius, sx=sx,
                                      dip=True, second_bezel=True)
                original = crop.copy()
                reading = read_humidity_auto_scale(crop)
                self.assertIsInstance(reading, CropReading)
                self.assertEqual(reading.method, "humidity_auto_scale")
                self.assertAlmostEqual(reading.value, 66.8, delta=.8)
                self.assertGreaterEqual(reading.tick_count, 50)
                self.assertLessEqual(math.dist(reading.center, center), 2)
                projected = math.hypot(radius * sx * math.sin(math.radians(bearing)),
                                       radius * math.cos(math.radians(bearing)))
                self.assertAlmostEqual(reading.radius, .765 * projected, delta=5)
                observed = math.degrees(math.atan2(sx * math.sin(math.radians(bearing)),
                                                    math.cos(math.radians(bearing))))
                self.assertAlmostEqual((reading.angle + observed - 90 + 180) % 360 - 180,
                                       0, delta=2)
                np.testing.assert_array_equal(crop, original)

    def test_both_physical_endpoints(self):
        for truth in (0, 100):
            with self.subTest(truth=truth):
                crop, _ = _dial(value=truth)
                self.assertAlmostEqual(read_humidity_auto_scale(crop).value, truth, delta=.7)

    def test_missing_minor_tick_does_not_shift_next_group(self):
        crop, _ = _dial(missing=(22,))
        reading = read_humidity_auto_scale(crop)
        self.assertAlmostEqual(reading.value, 66.8, delta=.8)
        self.assertEqual(reading.tick_count, 50)

    def test_unrelated_missing_ticks_do_not_block_supported_local_interval(self):
        crop, _ = _dial(missing=(11, 12, 13), value=66.8)
        reading = read_humidity_auto_scale(crop)
        self.assertEqual(reading.method, "humidity_auto_scale_local")
        self.assertEqual(reading.tick_count, 6)
        self.assertAlmostEqual(reading.value, 66.8, delta=.8)

    def test_local_path_does_not_interpolate_through_multiple_missing_ticks(self):
        crop, _ = _dial(missing=(11, 12, 13, 32, 33), value=66.8)
        with self.assertRaises(UnreadableGauge):
            read_humidity_auto_scale(crop)

    def test_pointer_inside_missing_tick_gap_is_rejected(self):
        crop, _ = _dial(missing=(22,), value=44)
        with self.assertRaisesRegex(UnreadableGauge, "between incomplete minor ticks"):
            read_humidity_auto_scale(crop)

    @staticmethod
    def _weak_major_fixture():
        # 80..100 segment from a real failure: pointer masks the 88 stroke;
        # 90 is still visible but is shorter than a normal major candidate.
        strokes = [(274.75, 12, 768), (278.75, 8, 252), (282.25, 8, 304),
                   (286.25, 8, 301), (294.50, 9, 459), (297.50, 6, 125),
                   (300.50, 8, 232), (303.75, 7, 174), (306.50, 6, 122),
                   (309.75, 10, 651)]
        ticks = [_Tick(round(deg * 4), length, strength) for deg, length, strength in strokes]
        arc = _Arc(202, 1239, np.empty(0), 85, (692, 995.5))
        approx = np.interp(np.arange(0, 101, 10), (0, 40, 70, 100),
                           (arc.start, *arc.transitions, arc.stop))
        return ticks, arc, approx

    def test_recover_single_occluded_minor_with_corroborated_weak_major(self):
        ticks, arc, approx = self._weak_major_fixture()
        self.assertEqual(_occluded_tick(ticks, {80: 0, 100: 9}, approx, arc, 290.75 * 4),
                         (88.0, 10))

    def test_occlusion_repair_rejects_off_center_ray_or_uncertain_marks(self):
        ticks, arc, approx = self._weak_major_fixture()
        for label, marks, bearing in (
            ("ray off center", ticks, 287.0),
            ("weak major also missing", ticks[:4] + ticks[5:], 290.75),
            ("another minor missing", ticks[:2] + ticks[3:], 290.75),
            ("major lacks contrast", ticks[:4] + [_Tick(ticks[4].column, 9, 250)] + ticks[5:], 290.75),
        ):
            with self.subTest(label=label):
                with self.assertRaises(UnreadableGauge):
                    _occluded_tick(marks, {80: 0, 100: len(marks) - 1}, approx, arc, bearing * 4)

    def test_truncated_origin_or_missing_major_is_not_renumbered(self):
        for options in ({"shorten": 1}, {"until": 49}, {"missing": (0,)}, {"missing": (30,)}):
            with self.subTest(options=options):
                crop, _ = _dial(**options)
                with self.assertRaisesRegex(UnreadableGauge, "endpoints|colored humidity scale"):
                    read_humidity_auto_scale(crop)

    def test_missing_color_anchors_is_unreadable(self):
        crop, _ = _dial(monochrome=True)
        with self.assertRaisesRegex(UnreadableGauge, "colored humidity scale"):
            read_humidity_auto_scale(crop)

    def test_outside_scale_is_not_clamped(self):
        crop, _ = _dial(off_scale=-8)
        with self.assertRaisesRegex(UnreadableGauge, "pointer outside"):
            read_humidity_auto_scale(crop)

    def test_two_pointer_length_rays_are_unreadable(self):
        crop, _ = _dial(ambiguous=True)
        with self.assertRaisesRegex(UnreadableGauge, "ambiguous humidity pointer"):
            read_humidity_auto_scale(crop)

    def test_invalid_and_blank_crop_rejected(self):
        for crop in (None, np.zeros((100, 100), np.uint8), np.full((100, 100, 3), 200, np.uint8)):
            with self.subTest(shape=getattr(crop, "shape", None)):
                with self.assertRaises(UnreadableGauge):
                    read_humidity_auto_scale(crop)


if __name__ == "__main__":
    unittest.main()
