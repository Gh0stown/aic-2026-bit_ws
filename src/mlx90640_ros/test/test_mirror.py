#!/usr/bin/env python3
"""Replay-only orientation regression; never opens a serial port."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from mlx90640_ros.core import Decoder, iter_capture_payloads, load_capture
from mlx90640_ros.hotspot import find_hotspot
from mlx90640_ros.thermal_session import ThermalSession
from mlx90640_ros.viewer import build_parser


class MirrorOptionsTests(unittest.TestCase):
    def test_viewer_default_and_opt_out(self):
        parser = build_parser()
        self.assertTrue(parser.parse_args(['--capture', 'offline']).mirror_x)
        self.assertFalse(parser.parse_args(['--capture', 'offline', '--no-mirror-x']).mirror_x)


@unittest.skipUnless(os.environ.get('MLX90640_LIBRARY'), 'set MLX90640_LIBRARY')
class MirrorReplayTests(unittest.TestCase):
    def test_only_columns_change_and_gui_uses_the_same_coordinates(self):
        import cv2
        eeprom, words = load_capture(Path(__file__).parent / 'data' / 'capture.bin')
        results = []
        # Sequential decoders: the native calibration is process-global.
        for mirror in (False, True):
            decoder = Decoder(os.environ['MLX90640_LIBRARY'], mirror_x=mirror)
            decoder.load_eeprom(eeprom)
            results.append([sample for payload in iter_capture_payloads(words)
                            if (sample := decoder.process_frame(payload)) is not None])
        self.assertEqual([len(samples) for samples in results], [2, 2])
        for raw, mirrored in zip(*results):
            np.testing.assert_array_equal(mirrored.temps, raw.temps[:, ::-1])
            self.assertTrue(mirrored.temps.flags.c_contiguous)
            self.assertEqual(mirrored.ambient, raw.ambient)
            self.assertEqual(mirrored.device_ms, raw.device_ms)
            old = find_hotspot(raw.temps)
            new = find_hotspot(mirrored.temps)
            self.assertEqual(new.row_index, old.row_index)
            self.assertEqual(new.col_index, 31 - old.col_index)
            r0, c0, r1, c1 = old.box
            self.assertEqual(new.box, (r0, 31 - c1, r1, 31 - c0))
        session = ThermalSession(capture='offline', library_path=os.environ['MLX90640_LIBRARY'])
        try:
            self.assertTrue(session._decoder.mirror_x)
            frame = session.process(results[1][0])
            np.testing.assert_array_equal(frame.temps, results[1][0].temps)
            with tempfile.TemporaryDirectory(prefix='mlx-mirror-test-') as folder:
                with patch.object(session, 'latest', return_value=frame):
                    png, _ = session.save_snapshot(folder)
                np.testing.assert_array_equal(cv2.imread(str(png)), frame.bgr)
        finally:
            session.stop()


if __name__ == '__main__':
    unittest.main()
