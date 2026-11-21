"""Focused, dependency-free tests of Exp16's batch stability policy."""

from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gauge_exp16_stability import BATCH_SIZE, MAX_BATCH_SIZE, MIN_VALID_COUNT, StabilityBatch


class StabilityBatchTests(unittest.TestCase):
    def _complete(self, temperatures, humidities, **collection):
        self.assertEqual(len(temperatures), len(humidities))
        batch = StabilityBatch(**collection)
        for temperature, humidity in zip(temperatures[:-1], humidities[:-1]):
            self.assertIsNone(batch.add(temperature, humidity))
        result = batch.add(temperatures[-1], humidities[-1])
        self.assertIsNotNone(result)
        self.assertEqual(result.frame_count, len(humidities))
        return result

    def assertDial(self, dial, status, valid_count, inlier_count, value=None):
        self.assertEqual((dial.status, dial.valid_count, dial.inlier_count),
                         (status, valid_count, inlier_count))
        if value is None:
            self.assertIsNone(dial.value)
        else:
            self.assertIsInstance(dial.value, float)
            self.assertAlmostEqual(dial.value, value)

    def test_counts_attempts_and_yields_one_immutable_serializable_result(self):
        self.assertEqual((BATCH_SIZE, MAX_BATCH_SIZE, MIN_VALID_COUNT), (10, 20, 6))
        batch = StabilityBatch()
        self.assertEqual(batch.count, 0)
        self.assertEqual(batch.temperature_valid_count, 0)
        self.assertEqual(batch.humidity_valid_count, 0)
        # Both dials need the tenth attempt to reach the valid-reading minimum.
        temperatures = [15.0] * 4 + [None] * 4 + [15.0]
        humidities = [40.0] * 2 + [None] * 3 + [40.0] * 3 + [None]
        for count, (temperature, humidity) in enumerate(zip(temperatures, humidities), 1):
            self.assertIsNone(batch.add(temperature, humidity))
            self.assertEqual(batch.count, count)
            self.assertEqual(batch.temperature_valid_count,
                             sum(value is not None for value in temperatures[:count]))
            self.assertEqual(batch.humidity_valid_count,
                             sum(value is not None for value in humidities[:count]))
        result = batch.add(15.0, 40.0)
        self.assertEqual(batch.count, BATCH_SIZE)
        self.assertEqual(batch.temperature_valid_count, MIN_VALID_COUNT)
        self.assertEqual(batch.humidity_valid_count, MIN_VALID_COUNT)
        self.assertEqual(result.frame_count, BATCH_SIZE)
        self.assertEqual(json.loads(json.dumps(result.as_dict(), allow_nan=False)), {
            "frame_count": 10,
            "temperature": {"value": 15.0, "status": "ok", "valid_count": 6, "inlier_count": 6},
            "humidity": {"value": 40.0, "status": "ok", "valid_count": 6, "inlier_count": 6},
        })
        with self.assertRaises(FrozenInstanceError):
            result.frame_count = 11
        with self.assertRaises(FrozenInstanceError):
            result.temperature.value = 20.0
        with self.assertRaises(RuntimeError):
            batch.add(20.0, 50.0)
        self.assertEqual(batch.count, 10)
        self.assertIsNone(StabilityBatch().add(20.0, 50.0))

    def test_accepts_dominant_medians_from_handoff_replay(self):
        # Ordered temperature/humidity values from the true-weight final replay.
        result = self._complete(
            [16.0, 15.065, 14.16, 15.065, 15.065, 15.065, 15.065, 10.527, 15.065, 15.065],
            [None, None, 40.0, 22.0, 40.0, 40.0, 30.0, 40.0, 40.0, None],
        )
        self.assertDial(result.temperature, "ok", 10, 8, 15.065)
        self.assertDial(result.humidity, "ok", 7, 5, 40.0)

    def test_non_finite_and_missing_values_are_ignored_per_dial(self):
        result = self._complete(
            [15.0, float("nan"), 15.2, None, 15.0, 15.2, float("inf"),
             15.0, float("-inf"), 15.2] + [None] * (MAX_BATCH_SIZE - BATCH_SIZE),
            [40.0, None, float("nan"), 40.0, None, 40.0, float("inf"),
             40.0, float("-inf"), 40.0] + [float("nan")] * (MAX_BATCH_SIZE - BATCH_SIZE),
        )
        self.assertDial(result.temperature, "ok", 6, 6, 15.1)
        self.assertDial(result.humidity, "reference", 5, 5, 40.0)

    def test_requires_five_and_seventy_percent_agreement_for_stability(self):
        tail = [None] * (MAX_BATCH_SIZE - BATCH_SIZE)
        result = self._complete(
            [15.0, 17.0, 15.0, None, None, 15.0, 13.0, 15.0, None, None] + tail,
            [40.0, 20.0, 40.0, 40.0, 60.0, 40.0, 20.0, 40.0, 40.0, None] + tail,
        )
        self.assertDial(result.temperature, "reference", 6, 4, 15.0)
        self.assertDial(result.humidity, "reference", 9, 6, 40.0)
        result = self._complete([15.0, 17.0, 15.0, 15.0, 17.0] + [15.0] * 4 + [17.0],
                                [40.0] * 10)
        self.assertDial(result.temperature, "ok", 10, 7, 15.0)

    def test_initial_split_is_fixed_but_extension_splits_each_valid_span(self):
        # Only one reading agrees in one of the initial attempted halves for each
        # dial, but each independent valid span is balanced on the next attempt.
        result = self._complete([15.0] * 5 + [None, 15.0, None, None, None, None],
                                [None] * 4 + [40.0] * 6 + [None])
        self.assertEqual(result.frame_count, BATCH_SIZE + 1)
        self.assertDial(result.temperature, "ok", 6, 6, 15.0)
        self.assertDial(result.humidity, "ok", 6, 6, 40.0)

    def test_clean_drift_and_step_yield_only_references_at_cap(self):
        # Seven temperatures fit the width: count and split alone are not enough.
        tail = [None] * (MAX_BATCH_SIZE - BATCH_SIZE)
        result = self._complete([15.0 + 0.15 * i for i in range(10)] + tail,
                                [40.0] * 5 + [45.0] * 5 + tail)
        self.assertDial(result.temperature, "reference", 10, 7, 15.9)
        self.assertDial(result.humidity, "reference", 10, 5, 45.0)
        result = self._complete([15.0] * 10 + tail,
                                [45.0 - 0.30 * i for i in range(10)] + tail)
        self.assertDial(result.humidity, "reference", 10, 7, 43.2)

    def test_single_endpoint_outlier_is_not_mistaken_for_drift(self):
        result = self._complete([14.0] + [15.0 + 0.01 * i for i in range(9)], [40.0] * 10)
        self.assertDial(result.temperature, "ok", 10, 9, 15.04)

    def test_width_is_full_span_inclusive_with_only_float_tolerance(self):
        result = self._complete([15.0, 16.0 + 1e-12] * 5, [40.0, 42.0 + 1e-12] * 5)
        self.assertDial(result.temperature, "ok", 10, 10, 15.5)
        self.assertDial(result.humidity, "ok", 10, 10, 41.0)
        tail = [None] * (MAX_BATCH_SIZE - BATCH_SIZE)
        result = self._complete([15.0, 16.000001] * 5 + tail, [40.0, 42.000001] * 5 + tail)
        self.assertDial(result.temperature, "reference", 10, 5, 16.000001)
        self.assertDial(result.humidity, "reference", 10, 5, 42.000001)

    def test_equally_populated_overlapping_clusters_prefer_temporal_support(self):
        result = self._complete([14.0, 14.9, 14.9, 14.9, 10.0, 14.9, 15.8, None, None, None],
                                [40.0] * 10)
        # The lower cluster has only one agreeing frame in the second half;
        # the equally large upper cluster has two.
        self.assertDial(result.temperature, "ok", 7, 5, 14.9)

    def test_remaining_overlap_ties_are_deterministic(self):
        # The two tied clusters have the same split and width but differing medians.
        temperatures = [0.0, 0.5, 0.625, 1.25, None, 0.125, 0.75, 1.25, None, None]
        for order in (temperatures, temperatures[::-1]):
            with self.subTest(order=order):
                result = self._complete(order, [40.0] * 10)
                self.assertDial(result.temperature, "ok", 7, 5, 0.5)

    def test_extends_below_minimum_and_locks_the_first_stable_result_per_dial(self):
        batch = StabilityBatch()
        temperatures = [15.0] * 2 + [None] * 3 + [15.0] * 3 + [None] * 2
        humidities = [40.0, 20.0, None, None, 40.0, None, None, None, 40.0, 60.0]
        for temperature, humidity in zip(temperatures, humidities):
            self.assertIsNone(batch.add(temperature, humidity))
        self.assertEqual(batch.count, BATCH_SIZE)
        self.assertEqual(batch.temperature_valid_count, 5)
        self.assertEqual(batch.humidity_valid_count, 5)

        # At 11 temperature stabilizes; only four of six humidity reads agree.
        self.assertIsNone(batch.add(15.0, 40.0))
        self.assertEqual(batch.count, 11)
        self.assertEqual(batch.temperature_valid_count, 6)
        self.assertEqual(batch.humidity_valid_count, 6)
        result = batch.add(15.0, 40.0)
        self.assertEqual(result.frame_count, 12)
        self.assertEqual(batch.count, 12)
        self.assertEqual(batch.temperature_valid_count, 7)
        self.assertEqual(batch.humidity_valid_count, 7)
        self.assertDial(result.temperature, "ok", 6, 6, 15.0)
        self.assertDial(result.humidity, "ok", 7, 5, 40.0)
        with self.assertRaises(RuntimeError):
            batch.add(15.0, 40.0)
        self.assertEqual(batch.count, 12)
        self.assertEqual(batch.humidity_valid_count, 7)

    def test_extension_accepts_late_humidity_after_leading_dropouts(self):
        humidities = [None] * BATCH_SIZE + [40.0] * MIN_VALID_COUNT
        result = self._complete([15.0] * BATCH_SIZE + [50.0] * MIN_VALID_COUNT, humidities)
        self.assertDial(result.temperature, "ok", 10, 10, 15.0)
        self.assertDial(result.humidity, "ok", 6, 6, 40.0)

    def test_extends_for_inconsistent_temperature_and_preserves_stable_humidity(self):
        # All ten temperatures are valid, but the dominant cluster reaches 70%
        # agreement only on frame 14. Later humidity outliers must not reopen it.
        result = self._complete([15.0] * 6 + [20.0] * 4 + [15.0] * 4,
                                [40.0] * 10 + [90.0] * 4)
        self.assertDial(result.temperature, "ok", 14, 10, 15.0)
        self.assertDial(result.humidity, "ok", 10, 10, 40.0)

    def test_retry_can_skip_either_dial_and_ignores_its_readings(self):
        for skipped, value in (("temperature", 40.0), ("humidity", 15.0)):
            with self.subTest(skipped=skipped):
                batch = StabilityBatch(**{"collect_" + skipped: False})
                for _ in range(BATCH_SIZE - 1):
                    self.assertIsNone(batch.add(15.0, 40.0))
                result = batch.add(15.0, 40.0)
                kept = "temperature" if skipped == "humidity" else "humidity"
                self.assertDial(getattr(result, skipped), "skipped", 0, 0)
                self.assertDial(getattr(result, kept), "ok", BATCH_SIZE, BATCH_SIZE, value)
                self.assertEqual(result.frame_count, BATCH_SIZE)
                self.assertEqual(getattr(batch, skipped + "_valid_count"), 0)

    def test_stable_result_is_preferred_even_on_last_attempt(self):
        temperatures = [None] * (MAX_BATCH_SIZE - MIN_VALID_COUNT) + [15.0] * MIN_VALID_COUNT
        result = self._complete(temperatures, [40.0] * MAX_BATCH_SIZE, collect_humidity=False)
        self.assertDial(result.temperature, "ok", 6, 6, 15.0)
        self.assertDial(result.humidity, "skipped", 0, 0)

    def test_no_data_finishes_insufficient_at_cap(self):
        batch = StabilityBatch()
        for attempt in range(1, MAX_BATCH_SIZE):
            self.assertIsNone(batch.add(None, None))
            self.assertEqual(batch.count, attempt)
            self.assertEqual(batch.temperature_valid_count, 0)
            self.assertEqual(batch.humidity_valid_count, 0)
        result = batch.add(None, None)
        self.assertEqual(batch.count, MAX_BATCH_SIZE)
        self.assertEqual(result.frame_count, MAX_BATCH_SIZE)
        self.assertDial(result.temperature, "insufficient", 0, 0)
        self.assertDial(result.humidity, "insufficient", 0, 0)
        with self.assertRaises(RuntimeError):
            batch.add(None, 40.0)

    def test_extended_humidity_requires_consensus_within_same_width(self):
        humidities = ([40.0, 42.000001] * 2 + [40.0] + [None] * 5
                      + [42.000001, 40.0] * 5)
        result = self._complete([15.0] * MAX_BATCH_SIZE, humidities)
        self.assertDial(result.humidity, "reference", 15, 8, 40.0)

    def test_extended_humidity_keeps_attempted_indices_for_temporal_gate(self):
        # Valid rank would split these six agreeing samples three and three.
        # The midpoint of attempts 1–15 leaves only attempt 1 in the first half.
        humidities = [40.0] + [None] * 9 + [40.0] * 5 + [None] * 5
        result = self._complete([15.0] * MAX_BATCH_SIZE, humidities)
        self.assertDial(result.humidity, "reference", 6, 6, 40.0)

    def test_extended_humidity_still_vetoes_stable_clean_drift(self):
        # Five of the seven readings fit the width and span both halves, but
        # trimming the endpoints still leaves a monotone span over 2 RH points.
        humidities = ([37.8, 39.5, 40.7, 41.0, 41.3] + [None] * 5
                      + [41.6, 42.0] + [None] * 8)
        result = self._complete([15.0] * MAX_BATCH_SIZE, humidities)
        self.assertDial(result.humidity, "reference", 7, 5, 41.3)

    def test_reference_requires_an_agreeing_pair(self):
        result = self._complete([15.0] + [None] * 19,
                                [40.0 + 3.0 * i for i in range(MAX_BATCH_SIZE)])
        self.assertDial(result.temperature, "insufficient", 1, 1)
        self.assertDial(result.humidity, "insufficient", MAX_BATCH_SIZE, 1)

    def test_reference_uses_observed_median_from_most_supported_cluster(self):
        # The older temperature pair beats the newer singleton. The humidity
        # pairs tie in count, so the newer pair wins even though it is wider.
        temperatures = [15.0, 16.0, 12.0] + [None] * 16 + [20.0]
        humidities = [40.0, 40.1, 70.0] + [None] * 15 + [48.0, 46.0]
        result = self._complete(temperatures, humidities)
        self.assertDial(result.temperature, "reference", 4, 2, 16.0)
        self.assertDial(result.humidity, "reference", 5, 2, 46.0)

    def test_new_rounds_can_repeatedly_return_reference_without_promoting_it(self):
        for value in (40.0, 40.0, 56.0):
            with self.subTest(value=value):
                result = self._complete([15.0] * MAX_BATCH_SIZE,
                                        [value] * 2 + [None] * 18,
                                        collect_temperature=False)
                self.assertDial(result.temperature, "skipped", 0, 0)
                self.assertDial(result.humidity, "reference", 2, 2, value)


if __name__ == "__main__":
    unittest.main()
