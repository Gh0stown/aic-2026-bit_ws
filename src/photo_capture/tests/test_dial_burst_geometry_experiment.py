#!/usr/bin/env python3
"""Tests for the isolated joint needle-direction burst experiment.

Synthetic point sets and synthetic images are constructed independently of the
tested joint model.  The only external assets are the frozen R5 module/test, the
candidate calibration and its reference image; real-data tests skip when those
are absent.  A valid synthetic needle is a positive control and is never counted
as a true negative.  Nothing here is a real dial truth or an accuracy result.
"""

import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dial_burst_geometry_experiment as exp
import dial_low_saturation_experiment as r5

REPO_ROOT = Path(__file__).resolve().parents[3]
CANDIDATE = REPO_ROOT / '.gauge_calibration_review' / 'r2-final' / 'candidate_calib.json'
REFERENCE = Path('/home/mowen/.dsh/attachments/v1/objects/be/'
                 'be71a511ebfe297c18c86feebc8d99cdefe9030e507a82e65f50d2aed8d36f30')
REAL_FRAMES = Path('/home/mowen/Pictures/photo_capture/仪表验证')


def _frame(angle_deg, offset=0.0, count=18, noise=0.0, bend=0.0,
           moved=0.0, flip_tip=False, seed=7, roll_u=10.0):
    rng = np.random.RandomState(seed)
    s = np.linspace(5.0, 22.0, count)
    theta = math.radians(angle_deg + moved)
    u = np.array([math.cos(theta), math.sin(theta)])
    n = np.array([-math.sin(theta), math.cos(theta)])
    q = offset * n + s[:, None] * u
    if noise:
        q = q + noise * rng.randn(count, 1) * n
    if bend:
        q = q + bend * (s - s.mean())[:, None] ** 2 * n
    return {'q': q, 'w': np.ones(count), 's': s,
            'tip': -u if flip_tip else u, 'roll_uncertainty_deg': roll_u}


def _frame_set(angle_deg, count=8, offsets=None, **kwargs):
    offsets = offsets or [0.1 * (i - count / 2.0) for i in range(count)]
    return [_frame(angle_deg, offset=offsets[i], seed=10 + i, **kwargs)
            for i in range(count)]


def _line_image(angle=30.0, offset=1.2, thickness=2, size=75, bg=180, ink=40):
    image = np.full((size, size, 3), bg, np.uint8)
    hub = np.array([37.0, 37.0])
    theta = math.radians(angle)
    u = np.array([math.cos(theta), math.sin(theta)])
    n = np.array([-math.sin(theta), math.cos(theta)])
    p1 = np.round(hub + 5.0 * u + offset * n).astype(int)
    p2 = np.round(hub + 23.0 * u + offset * n).astype(int)
    cv2.line(image, tuple(p1), tuple(p2), (ink, ink, ink), thickness, cv2.LINE_AA)
    return image


def _ring():
    return {'center': [37.5, 37.5], 'axes': [60.0, 60.0], 'angle_deg': 0.0}


def _hub(plateau=None):
    hub = {'center': [37.0, 37.0], 'usable': True, 'ambiguous': False}
    if plateau:
        hub['plateau'] = {'span_px': [float(plateau), float(plateau)]}
    return hub


def _perturbation(expected=10, valid=10, complete=True, span=0.3, failures=None):
    return {'expected': expected, 'valid': valid, 'complete': complete,
            'span_deg': span, 'max_abs_delta_deg': 0.2,
            'failures': [] if failures is None else failures}


def _joint_stub(near=1.0, frame_bootstrap=1.0, block_bootstrap=1.0,
                perturbation=None, loo=0.4):
    return {'loo_span_deg': loo, 'sign_conflict': False,
            'near_far': {'difference_deg': near, 'consistent_sign': True},
            'frame_direction': {'max_abs_deviation_deg': 1.5, 'range_deg': 1.0},
            'bootstrap': {'frame': {'span_deg': frame_bootstrap},
                          'block': {'span_deg': block_bootstrap}},
            'perturbation': (_perturbation() if perturbation is None else perturbation)}


def _summary_stub(joint):
    return {'input': {'included_eval_frames': 8, 'body_gate_passed': True},
            'joint': joint}


class BindingTests(unittest.TestCase):
    def test_r5_module_and_test_are_sha_bound(self):
        record = exp.verify_r5_binding()
        self.assertTrue(record['r5_module']['match'])
        self.assertTrue(record['r5_test']['match'])
        self.assertEqual(record['r5_module']['sha256'], exp.R5_SHA)
        self.assertEqual(record['r5_test']['sha256'], exp.R5_TEST_SHA)

    def test_roi_contract_matches_r5(self):
        self.assertEqual(tuple(r5.DECLARED_ROI), exp.DECLARED_ROI)
        self.assertEqual(tuple(r5.EXPERIMENT_ROI), exp.USED_ROI)


class GeometryContractTests(unittest.TestCase):
    def test_reconstruct_pixels_hand_computable(self):
        point = exp.reconstruct_pixels([4.0], [-0.5], 30.0, 1.5, [10.0, 20.0])
        expected = np.array([10.0 + 4.0 * math.cos(math.radians(30.0))
                             - 1.0 * math.sin(math.radians(30.0)),
                             20.0 + 4.0 * math.sin(math.radians(30.0))
                             + 1.0 * math.cos(math.radians(30.0))])
        np.testing.assert_allclose(point[0], expected, atol=1e-9)

    def test_map_to_reference_hand_computable(self):
        matrix = np.diag([1.0 / 10.0, 1.0 / 20.0])
        q = exp.map_to_reference([[10.0, 0.0], [0.0, 20.0]], [0.0, 0.0],
                                 matrix, 90.0)
        np.testing.assert_allclose(q, [[0.0, -1.0], [1.0, 0.0]], atol=1e-12)

    def test_line_refine_coordinate_contract(self):
        image = _line_image()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)
        matrix, _, _, _ = r5._ellipse_matrix((60.0, 60.0), 0.0)
        hub = np.array([37.0, 37.0])
        refine = r5._line_refine(gray, np.array([37.5, 37.5]), matrix, hub,
                                 30.0, 5.0, 30.0, 1.2, max_separation_px=3.5,
                                 cross_half_px=2.5, cross_step_px=0.5)
        self.assertIsNotNone(refine)
        s = np.asarray(refine['samples'])
        t = np.asarray(refine['center'])
        theta = math.radians(refine['coarse_angle_deg'])
        u = np.array([math.cos(theta), math.sin(theta)])
        n = np.array([-math.sin(theta), math.cos(theta)])
        pixel = exp.reconstruct_pixels(s, t, refine['coarse_angle_deg'],
                                       refine['coarse_offset_px'], hub)
        fit = refine['fit']
        expected = (hub + s[:, None] * u
                    + (refine['coarse_offset_px'] + fit['intercept']
                       + fit['slope'] * s)[:, None] * n)
        distance = (pixel - expected) @ n
        self.assertLess(abs(float(np.mean(distance))), 0.15)
        centered = pixel - pixel.mean(axis=0)
        _, vectors = np.linalg.eigh(centered.T @ centered)
        angle = math.degrees(math.atan2(vectors[1, 1], vectors[0, 1])) % 360.0
        delta = abs(exp._signed(angle, refine['angle_deg']))
        self.assertLess(min(delta, abs(180.0 - delta)), 0.2)

    def test_synthetic_line_is_a_positive_control(self):
        result = r5.estimate_needle_line(_line_image(), _ring(), _hub())
        self.assertTrue(result['ok'], result.get('reason'))
        self.assertLess(abs(exp._signed(result['angle_deg'], 30.0)), 2.0)


class ImageNegativeTests(unittest.TestCase):
    def test_pure_noise_has_no_support(self):
        noise = np.random.RandomState(0).randint(0, 255, (75, 75, 3), dtype=np.uint8)
        result = r5.estimate_needle_line(noise, _ring(), _hub())
        self.assertFalse(result['ok'])

    def test_pure_cap_is_not_a_needle(self):
        image = np.full((75, 75, 3), 180, np.uint8)
        cv2.circle(image, (37, 37), 6, (40, 40, 40), -1)
        result = r5.estimate_needle_line(image, _ring(), _hub(plateau=12))
        self.assertFalse(result['ok'])

    def test_text_block_is_not_a_needle(self):
        image = np.full((75, 75, 3), 180, np.uint8)
        cv2.rectangle(image, (28, 34), (50, 40), (40, 40, 40), -1)
        result = r5.estimate_needle_line(image, _ring(), _hub())
        self.assertFalse(result['ok'])

    def test_random_blocks_are_not_a_needle(self):
        image = np.full((75, 75, 3), 180, np.uint8)
        rng = np.random.RandomState(2)
        for _ in range(8):
            x, y = rng.randint(10, 55), rng.randint(10, 55)
            cv2.rectangle(image, (x, y), (x + rng.randint(2, 10),
                                          y + rng.randint(2, 8)), (30, 30, 30), -1)
        result = r5.estimate_needle_line(image, _ring(), _hub())
        self.assertFalse(result['ok'])


class JointFitTests(unittest.TestCase):
    def test_multiple_directions_and_zero_wrap(self):
        for truth in (10.0, 170.0, 181.0, 350.0):
            frames = _frame_set(truth, count=6)
            fit = exp.fit_joint(frames, resolve_tips=True)
            self.assertFalse(fit['conflict'])
            self.assertLess(abs(exp._signed(fit['angle_deg'], truth)), 0.05)

    def test_per_frame_position_offsets_do_not_move_direction(self):
        frames = _frame_set(42.0, count=8, offsets=[-0.2, 0.15, 0.0, 0.3,
                                                    -0.1, 0.25, -0.3, 0.05])
        fit = exp.fit_joint(frames, resolve_tips=True)
        self.assertLess(abs(exp._signed(fit['angle_deg'], 42.0)), 0.05)
        self.assertEqual(len(set(round(v, 3) for v in fit['b'])), len(frames))

    def test_180_conflict_is_rejected(self):
        frames = _frame_set(80.0, count=6)
        frames[2]['tip'] = -frames[2]['tip']
        fit = exp.fit_joint(frames, resolve_tips=True)
        self.assertTrue(fit['conflict'])

    def test_needle_moved_is_reported_as_heterogeneity(self):
        frames = _frame_set(100.0, count=8)
        frames[3] = _frame(105.0, offset=0.0, seed=33)
        fit = exp.fit_joint(frames, resolve_tips=True)
        deviations = [abs(exp._signed(exp.own_angle(f, fit['v']), fit['angle_deg']))
                      for f in frames]
        self.assertGreater(max(deviations), 2.0)

    def test_consistent_near_far_bend_is_not_averaged_away(self):
        frames = _frame_set(50.0, count=8, bend=0.05)
        fit = exp.fit_joint(frames, resolve_tips=True)
        result = exp.near_far(frames, fit)
        self.assertGreater(result['difference_deg'], 2.0)
        self.assertTrue(result['consistent_sign'])

    def test_duplicate_frames_count_once_and_cannot_narrow_bootstrap(self):
        records = [{'index': i, 'crop_sha256': 'same'} for i in range(10)]
        info = exp.dedupe(records)
        self.assertEqual(info['unique'], 1)
        self.assertEqual(info['groups'][0]['frames'], list(range(10)))
        self.assertFalse(records[5]['included'])
        single = [_frame(30.0)]
        fit = exp.fit_joint(single, resolve_tips=True)
        bootstrap = exp.bootstrap_span(single, fit, 30, 3, 1)
        self.assertEqual(bootstrap['frame']['samples'], 0)
        self.assertIsNone(bootstrap['frame']['span_deg'])

    def test_too_few_frames_fail_the_evidence_gate(self):
        summary = {'input': {'included_eval_frames': 5, 'body_gate_passed': True},
                   'joint': None}
        gate = [g for g in exp.gates(summary) if g['name'] == 'unique_eval_frames'][0]
        self.assertFalse(gate['pass'])

    def test_systematic_roll_is_not_divided_by_frame_count(self):
        frames = _frame_set(70.0, count=8, roll_u=10.0)
        fit = exp.fit_joint(frames, resolve_tips=True)
        roll = exp.roll_span(frames, fit)
        self.assertEqual(roll['common_span_deg'], 20.0)
        self.assertTrue(roll['not_reduced_by_n'])
        rotated = [{'q': (exp._rot(10.0) @ f['q'].T).T, 'w': f['w']} for f in frames]
        shifted = exp.fit_joint(rotated, reference_v=fit['v'])
        self.assertLess(abs(exp._signed(shifted['angle_deg'], fit['angle_deg']) - 10.0), 0.01)

    def test_mapping_with_anisotropic_ring_and_roll_round_trips(self):
        theta = math.radians(20.0)
        rotation = np.array([[math.cos(theta), -math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])
        matrix = rotation @ np.diag([1.0 / 20.0, 1.0 / 40.0]) @ rotation.T
        roll = 35.0
        pixel_angle = 60.0
        expected = math.degrees(math.atan2(
            (exp._rot(-roll) @ (matrix @ np.array([math.cos(math.radians(pixel_angle)),
                                                   math.sin(math.radians(pixel_angle))])))[1],
            (exp._rot(-roll) @ (matrix @ np.array([math.cos(math.radians(pixel_angle)),
                                                   math.sin(math.radians(pixel_angle))])))[0])) % 360.0
        s = np.linspace(5.0, 22.0, 18)
        u = np.array([math.cos(math.radians(pixel_angle)),
                      math.sin(math.radians(pixel_angle))])
        n = np.array([-u[1], u[0]])
        pixels = np.array([10.0, 20.0]) + s[:, None] * u + 1.0 * n
        q = exp.map_to_reference(pixels, [10.0, 20.0], matrix, roll)
        fit = exp.fit_joint([{'q': q, 'w': np.ones(len(q)), 's': s}], resolve_tips=False)
        delta = abs(exp._signed(fit['angle_deg'], expected))
        self.assertLess(min(delta, abs(180.0 - delta)), 1e-6)


class SafetyGateTests(unittest.TestCase):
    def test_complete_perturbation_can_be_conditional_green(self):
        gate_rows = exp.gates(_summary_stub(_joint_stub()))
        by_name = {g['name']: g for g in gate_rows}
        self.assertTrue(by_name['hub_perturbation_span_deg']['pass'])
        self.assertTrue(exp.conditional_stability(gate_rows)['accepted'])

    def test_incomplete_perturbation_is_rejected(self):
        cases = [(9, False, []),
                 (10, False, []),
                 (10, True, [{'name': 'hub_x+', 'frame': 4, 'reason': 'rms'}])]
        for valid, complete, failures in cases:
            gate_rows = exp.gates(_summary_stub(_joint_stub(
                perturbation=_perturbation(valid=valid, complete=complete,
                                           failures=failures))))
            by_name = {g['name']: g for g in gate_rows}
            self.assertFalse(by_name['hub_perturbation_span_deg']['pass'],
                             (valid, complete, failures))
            self.assertFalse(exp.conditional_stability(gate_rows)['accepted'],
                             (valid, complete, failures))

    def test_zero_expected_perturbation_cannot_pass(self):
        gate_rows = exp.gates(_summary_stub(_joint_stub(
            perturbation=_perturbation(expected=0, valid=0, complete=True, span=0.0))))
        by_name = {g['name']: g for g in gate_rows}
        self.assertFalse(by_name['hub_perturbation_span_deg']['pass'])
        self.assertFalse(exp.conditional_stability(gate_rows)['accepted'])

    def test_missing_nan_or_wide_bootstrap_breaks_conditional(self):
        cases = (('frame_bootstrap', 'frame_bootstrap_span_deg'),
                 ('block_bootstrap', 'block_bootstrap_span_deg'))
        for field, gate_name in cases:
            for value in (None, float('nan'), 2.5):
                gate_rows = exp.gates(_summary_stub(_joint_stub(**{field: value})))
                by_name = {g['name']: g for g in gate_rows}
                self.assertFalse(by_name[gate_name]['pass'], (field, value))
                self.assertFalse(exp.conditional_stability(gate_rows)['accepted'],
                                 (field, value))

    def test_missing_required_gate_is_not_conditional_green(self):
        gate_rows = [g for g in exp.gates(_summary_stub(_joint_stub()))
                     if g['name'] != 'frame_bootstrap_span_deg']
        result = exp.conditional_stability(gate_rows)
        self.assertFalse(result['accepted'])
        self.assertIn('frame_bootstrap_span_deg', result['failed'])

    def test_near_far_stays_a_common_direction_gate(self):
        gate_rows = exp.gates(_summary_stub(_joint_stub(near=2.4657)))
        by_name = {g['name']: g for g in gate_rows}
        self.assertFalse(by_name['near_far_difference_deg']['pass'])
        self.assertTrue(exp.conditional_stability(gate_rows)['accepted'])


class OutputTests(unittest.TestCase):
    def test_nonempty_output_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'out'
            output.mkdir()
            (output / 'old.txt').write_text('old', encoding='utf-8')
            with self.assertRaises(exp.ExperimentConfigError):
                exp.ensure_output(output)

    def test_empty_output_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'out'
            output.mkdir()
            self.assertEqual(exp.ensure_output(output), output)

    def test_non_camera_size_is_rejected_before_detection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'phone.jpg'
            cv2.imwrite(str(path), np.zeros((1706, 1279, 3), np.uint8))
            record = exp.collect_frame(np.zeros((1706, 1279, 3), np.uint8),
                                       path, 1, 'eval', None, None)
            self.assertFalse(record['included'])
            self.assertEqual(record['exclusion_reason'], 'non_camera_frame_size')


@unittest.skipUnless(CANDIDATE.is_file() and REFERENCE.is_file() and REAL_FRAMES.is_dir(),
                     'local candidate/reference/burst assets are absent')
class RealAssetTests(unittest.TestCase):
    def test_downsampled_phone_image_is_not_a_camera_frame(self):
        from dial_reading import load_calibration
        calibration = load_calibration(str(CANDIDATE), True, exp.CANDIDATE_SHA)
        reference_image = cv2.imread(str(REFERENCE), cv2.IMREAD_COLOR)
        reference = r5.reference_signature(reference_image, calibration)
        downsampled = cv2.resize(reference_image, (1280, 720), interpolation=cv2.INTER_AREA)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'downsampled_phone.jpg'
            cv2.imwrite(str(path), downsampled)
            record = exp.collect_frame(downsampled, path, 1, 'eval',
                                       calibration, reference)
        self.assertFalse(record['included'])
        self.assertTrue(str(record['exclusion_reason']).startswith('ring:'))

    def test_real_burst_numbers_unchanged_and_conditional_rejected(self):
        from dial_reading import load_calibration
        calibration = load_calibration(str(CANDIDATE), True, exp.CANDIDATE_SHA)
        reference_image = cv2.imread(str(REFERENCE), cv2.IMREAD_COLOR)
        reference = r5.reference_signature(reference_image, calibration)
        summary = exp.run_study(REAL_FRAMES, calibration, reference,
                                Path('unused'))['summary']
        joint = summary['joint']
        self.assertAlmostEqual(joint['angle_deg'], 175.945, places=2)
        self.assertAlmostEqual(joint['near_far']['difference_deg'], 2.4657, places=3)
        self.assertAlmostEqual(joint['loo_span_deg'], 0.412, places=3)
        self.assertEqual(joint['perturbation']['valid'], 9)
        self.assertFalse(joint['perturbation']['complete'])
        self.assertEqual(joint['perturbation']['failures'],
                         [{'name': 'hub_x+', 'frame': 4, 'reason': 'rms'}])
        self.assertFalse(summary['conclusion']['accepted_conditional_stability'])
        self.assertFalse(summary['conclusion']['accepted_common_needle_direction'])
        self.assertFalse(summary['conclusion']['accepted_overall'])
        self.assertIn('hub_perturbation_span_deg',
                      summary['conclusion']['failed_gates'])
        self.assertIn('near_far_difference_deg',
                      summary['conclusion']['failed_gates'])
        self.assertIsNone(summary['value'])
        self.assertFalse(summary['trusted'])


if __name__ == '__main__':
    unittest.main()
