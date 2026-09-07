#!/usr/bin/env python3

import unittest

import numpy as np

from hazard_perception.tracker import HazardObservation, HazardTracker


def observation(x, timestamp, confidence=0.8, source="rgbd"):
    return HazardObservation(np.asarray([x, 0.0, 0.5]), confidence, timestamp, source)


class TrackerTest(unittest.TestCase):
    def test_diagnostic_events_explain_association_and_jump(self):
        tracker = HazardTracker(merge_distance=2.0, max_position_jump=0.5)
        first = tracker.update(observation(0.0, 0.0))
        self.assertEqual(tracker.last_event["event"], "new_track")
        self.assertIsNone(tracker.last_event["nearest_distance"])
        tracker.update(observation(0.1, 0.1))
        self.assertEqual(tracker.last_event["event"], "associated")
        self.assertEqual(tracker.last_event["track_id"], first.id)
        self.assertIsNone(tracker.update(observation(1.0, 0.2)))
        self.assertEqual(tracker.last_event["event"], "rejected_jump")
        self.assertGreater(tracker.last_event["jump"], 0.5)

    def test_diagnostic_expiration_is_carried_to_next_track(self):
        tracker = HazardTracker(candidate_timeout=1.0)
        old_id = tracker.update(observation(0.0, 0.0)).id
        new_track = tracker.update(observation(2.0, 2.0))
        self.assertNotEqual(old_id, new_track.id)
        self.assertEqual(tracker.last_event["event"], "new_track")
        self.assertEqual(tracker.last_event["expired_track_ids"], [old_id])

    def test_confirms_smooths_and_deduplicates_observations(self):
        tracker = HazardTracker(
            confirmation_count=3,
            merge_distance=0.6,
            smoothing_alpha=0.5,
            candidate_timeout=5.0,
            max_position_jump=0.7,
        )
        first = tracker.update(observation(1.0, 0.0))
        tracker.update(observation(1.2, 0.1, source="fused"))
        track = tracker.update(observation(1.0, 0.2, source="fused"))
        self.assertEqual(first.id, track.id)
        self.assertTrue(track.confirmed)
        self.assertEqual(track.observation_count, 3)
        self.assertAlmostEqual(track.position[0], 1.05)
        self.assertEqual(track.localization_source, "fused")

    def test_candidate_timeout_and_finalize_stops_new_tracks(self):
        tracker = HazardTracker(confirmation_count=2, candidate_timeout=1.0)
        old_id = tracker.update(observation(0.0, 0.0)).id
        new_track = tracker.update(observation(2.0, 2.0))
        self.assertNotEqual(old_id, new_track.id)
        self.assertNotIn(old_id, [track.id for track in tracker.tracks])
        tracker.finalize()
        self.assertIsNone(tracker.update(observation(5.0, 3.0)))

    def test_rejects_abnormal_jump(self):
        tracker = HazardTracker(
            merge_distance=2.0,
            max_position_jump=0.5,
            candidate_timeout=5.0,
        )
        track = tracker.update(observation(0.0, 0.0))
        self.assertIsNone(tracker.update(observation(1.0, 0.1)))
        self.assertEqual(track.observation_count, 1)

    def test_final_deduplicate_merges_confirmed_tracks(self):
        tracker = HazardTracker(
            confirmation_count=1,
            merge_distance=0.1,
            final_merge_distance=0.5,
        )
        tracker.update(observation(0.0, 0.0))
        tracker.update(observation(0.3, 0.1))
        finalized = tracker.finalize()
        self.assertEqual(len(finalized), 1)
        self.assertEqual(finalized[0].observation_count, 2)
        self.assertAlmostEqual(finalized[0].position[0], 0.15)

    def test_unvalidated_fallback_never_confirms(self):
        tracker = HazardTracker(confirmation_count=3, candidate_timeout=5.0)
        track = None
        for index in range(10):
            track = tracker.update(HazardObservation(
                np.asarray([1.0, 0.0, 0.5]), 0.3, index * 0.1, "monocular",
                geometry_validated=False,
            ))
        self.assertFalse(track.confirmed)
        self.assertEqual(track.validated_observation_count, 0)

    def test_edge_target_uses_higher_confirmation_count(self):
        tracker = HazardTracker(confirmation_count=3, candidate_timeout=5.0)
        track = None
        for index in range(4):
            track = tracker.update(HazardObservation(
                np.asarray([1.0, 0.0, 0.5]), 0.7, index * 0.1, "rgbd",
                geometry_validated=True, required_confirmation_count=5,
            ))
        self.assertFalse(track.confirmed)
        track = tracker.update(HazardObservation(
            np.asarray([1.0, 0.0, 0.5]), 0.7, 0.5, "rgbd",
            geometry_validated=True, required_confirmation_count=5,
        ))
        self.assertTrue(track.confirmed)


if __name__ == "__main__":
    unittest.main()
