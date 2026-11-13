#!/usr/bin/env python3
"""Fixed display-range tests; no hardware or ROS master required."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from mlx90640_ros.core import Sample, render_image
from mlx90640_ros.thermal_session import ThermalSession
from mlx90640_ros.viewer import build_parser


class FixedScaleTests(unittest.TestCase):
    def test_same_temperature_keeps_same_color_when_extrema_change(self):
        first = np.full((24, 32), 22.0, dtype=np.float32)
        second = first.copy()
        first[0, :2] = (19, 30)
        second[0, :2] = (10, 60)
        first[8, 8] = second[8, 8] = 25
        before = first.copy(), second.copy()
        images = [render_image(Sample(temps, 26, 0, 0), scale=6)
                  for temps in (first, second)]
        # Away from the peak marker and text, 25 C has exactly the same BGR.
        np.testing.assert_array_equal(images[0][49, 49], images[1][49, 49])
        norm = np.uint8((25 - 20) / (35 - 20) * 255)
        expected = cv2.applyColorMap(np.array([[norm]], np.uint8), cv2.COLORMAP_JET)[0, 0]
        np.testing.assert_array_equal(images[0][49, 49], expected)
        for temps, original in zip((first, second), before):
            np.testing.assert_array_equal(temps, original)

    def test_clips_colors_not_measurements_and_labels_actual_scale(self):
        temps = np.full((24, 32), 25.0, dtype=np.float32)
        temps[8, 8], temps[10, 10] = 15, 50
        original = temps.copy()
        with patch('cv2.putText', wraps=cv2.putText) as put_text:
            image = render_image(Sample(temps, 26, 0, 0), scale=6)
        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertIn('Max 50.00  Min 15.00 C', labels)
        self.assertTrue(any('Scale 20-35' in text for text in labels))
        ends = cv2.applyColorMap(np.array([[0, 255]], np.uint8), cv2.COLORMAP_JET)[0]
        np.testing.assert_array_equal(image[49, 49], ends[0])
        np.testing.assert_array_equal(image[61, 61], ends[1])
        np.testing.assert_array_equal(temps, original)

    def test_gui_viewer_and_launch_defaults_match_and_remain_adjustable(self):
        with patch('mlx90640_ros.core.Decoder'):
            session = ThermalSession(capture='offline')
            self.assertEqual((session.tmin, session.tmax), (20, 35))
            custom = ThermalSession(capture='offline', tmin=18, tmax=32)
            self.assertEqual((custom.tmin, custom.tmax), (18, 32))
        parser = build_parser()
        args = parser.parse_args(['--capture', 'offline'])
        self.assertEqual((args.tmin, args.tmax), (20, 35))
        args = parser.parse_args(['--capture', 'offline', '--tmin', '18', '--tmax', '32'])
        self.assertEqual((args.tmin, args.tmax), (18, 32))
        for launch in ('mlx90640.launch', 'replay.launch'):
            root = ET.parse(str(Path(__file__).parents[1] / 'launch' / launch)).getroot()
            args = {arg.get('name'): arg.get('default') for arg in root.findall('arg')}
            self.assertEqual((args['tmin'], args['tmax']), ('20', '35'))

    def test_explicit_auto_range_still_works_without_changing_defaults(self):
        temps = np.full((24, 32), 22.0, dtype=np.float32)
        temps[0, 0] = 24
        with patch('cv2.putText', wraps=cv2.putText) as put_text:
            render_image(Sample(temps, 26, 0, 0), tmin=None, tmax=None)
        self.assertTrue(any('Scale 22-24' in call.args[1]
                            for call in put_text.call_args_list))


if __name__ == '__main__':
    unittest.main()
