"""Spatial multi-frame hazard tracking without ROS dependencies."""

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

import numpy as np


@dataclass(frozen=True)
class HazardObservation:
    position: np.ndarray
    confidence: float
    timestamp: float
    localization_source: str
    geometry_validated: bool = True
    required_confirmation_count: int = 0

    def __post_init__(self):
        position = np.asarray(self.position, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("Observation position must be a finite XYZ vector")
        if not np.isfinite(self.confidence):
            raise ValueError("Observation confidence must be finite")
        if self.localization_source not in ("rgbd", "livox", "fused", "monocular"):
            raise ValueError("Unknown localization source")
        if self.required_confirmation_count < 0:
            raise ValueError("required_confirmation_count cannot be negative")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "confidence", float(np.clip(self.confidence, 0.0, 1.0)))


@dataclass
class HazardTrack:
    id: int
    position: np.ndarray
    observation_count: int
    confidence_sum: float
    first_observation: float
    last_observation: float
    confirmed: bool
    localization_source: str
    source_counts: Dict[str, int] = field(default_factory=dict)
    validated_observation_count: int = 0
    required_confirmation_count: int = 1

    @property
    def average_confidence(self) -> float:
        return self.confidence_sum / max(1, self.observation_count)


class HazardTracker:
    def __init__(
        self,
        confirmation_count: int = 3,
        merge_distance: float = 0.6,
        smoothing_alpha: float = 0.35,
        candidate_timeout: float = 3.0,
        max_position_jump: float = 1.0,
        final_merge_distance: Optional[float] = None,
    ):
        if confirmation_count < 1:
            raise ValueError("confirmation_count must be positive")
        if merge_distance <= 0.0 or max_position_jump <= 0.0:
            raise ValueError("Distances must be positive")
        if not 0.0 < smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if candidate_timeout < 0.0:
            raise ValueError("candidate_timeout cannot be negative")
        self.confirmation_count = int(confirmation_count)
        self.merge_distance = float(merge_distance)
        self.smoothing_alpha = float(smoothing_alpha)
        self.candidate_timeout = float(candidate_timeout)
        self.max_position_jump = float(max_position_jump)
        self.final_merge_distance = float(final_merge_distance or merge_distance)
        self._tracks: Dict[int, HazardTrack] = {}
        self._next_id = 1
        self.finalized = False
        # One compact event is intentionally exposed for the ROS diagnostics
        # writer.  Keeping it on the tracker avoids changing the public
        # update() return type used by the existing perception tests.
        self.last_event = {"event": "none"}
        self._expired_ids = []

    @property
    def tracks(self) -> List[HazardTrack]:
        return sorted(self._tracks.values(), key=lambda item: item.id)

    def expire_candidates(self, now: float) -> None:
        self._expired_ids = []
        expired = [
            track_id
            for track_id, track in self._tracks.items()
            if not track.confirmed and now - track.last_observation > self.candidate_timeout
        ]
        for track_id in expired:
            track = self._tracks[track_id]
            del self._tracks[track_id]
            self._expired_ids.append(int(track_id))
            self.last_event = {
                "event": "expired", "track_id": int(track_id),
                "last_observation": float(track.last_observation),
                "age": float(now - track.last_observation),
            }

    def _nearest_track(self, position: np.ndarray) -> Optional[HazardTrack]:
        if not self._tracks:
            return None
        candidates = [
            (float(np.linalg.norm(track.position - position)), track)
            for track in self._tracks.values()
        ]
        distance, track = min(candidates, key=lambda item: item[0])
        return track if distance <= self.merge_distance else None

    def update(self, observation: HazardObservation) -> Optional[HazardTrack]:
        self.expire_candidates(observation.timestamp)
        self.last_event = ({"event": "expired", "track_ids": list(self._expired_ids)}
                           if self._expired_ids else {"event": "none"})
        track = self._nearest_track(observation.position)
        nearest_distance = None
        if self._tracks:
            nearest_distance = min(
                float(np.linalg.norm(item.position - observation.position))
                for item in self._tracks.values()
            )
        if track is None:
            if self.finalized:
                self.last_event = {
                    "event": "finalized_reject",
                    "nearest_distance": nearest_distance,
                }
                return None
            required_count = int(
                observation.required_confirmation_count or self.confirmation_count
            )
            validated_count = 1 if observation.geometry_validated else 0
            track = HazardTrack(
                id=self._next_id,
                position=observation.position.copy(),
                observation_count=1,
                confidence_sum=observation.confidence,
                first_observation=observation.timestamp,
                last_observation=observation.timestamp,
                confirmed=(required_count <= 1 and validated_count >= required_count),
                localization_source=observation.localization_source,
                source_counts={observation.localization_source: 1},
                validated_observation_count=validated_count,
                required_confirmation_count=required_count,
            )
            self._tracks[track.id] = track
            self._next_id += 1
            self.last_event = {
                "event": "new_track", "track_id": int(track.id),
                "nearest_distance": nearest_distance,
                "observation_count": 1,
                "required_confirmation_count": int(required_count),
            }
            if self._expired_ids:
                self.last_event["expired_track_ids"] = list(self._expired_ids)
            return track

        jump = float(np.linalg.norm(track.position - observation.position))
        if jump > self.max_position_jump:
            self.last_event = {
                "event": "rejected_jump", "track_id": int(track.id),
                "jump": jump, "max_position_jump": self.max_position_jump,
                "observation_count": int(track.observation_count),
            }
            if self._expired_ids:
                self.last_event["expired_track_ids"] = list(self._expired_ids)
            return None
        alpha = self.smoothing_alpha
        track.position = (1.0 - alpha) * track.position + alpha * observation.position
        track.observation_count += 1
        track.confidence_sum += observation.confidence
        track.last_observation = observation.timestamp
        if observation.geometry_validated:
            track.validated_observation_count += 1
        requested = int(observation.required_confirmation_count or self.confirmation_count)
        track.required_confirmation_count = max(track.required_confirmation_count, requested)
        track.confirmed = bool(
            track.observation_count >= track.required_confirmation_count
            and track.validated_observation_count >= track.required_confirmation_count
        )
        track.source_counts[observation.localization_source] = (
            track.source_counts.get(observation.localization_source, 0) + 1
        )
        track.localization_source = max(
            track.source_counts,
            key=lambda source: (track.source_counts[source], source == "fused"),
        )
        self.last_event = {
            "event": "associated", "track_id": int(track.id),
            "distance": jump, "observation_count": int(track.observation_count),
            "validated_observation_count": int(track.validated_observation_count),
            "required_confirmation_count": int(track.required_confirmation_count),
            "confirmed": bool(track.confirmed),
        }
        if self._expired_ids:
            self.last_event["expired_track_ids"] = list(self._expired_ids)
        return track

    @staticmethod
    def _merge_pair(primary: HazardTrack, secondary: HazardTrack) -> None:
        total = primary.observation_count + secondary.observation_count
        primary.position = (
            primary.position * primary.observation_count
            + secondary.position * secondary.observation_count
        ) / total
        primary.observation_count = total
        primary.confidence_sum += secondary.confidence_sum
        primary.first_observation = min(primary.first_observation, secondary.first_observation)
        primary.last_observation = max(primary.last_observation, secondary.last_observation)
        primary.confirmed = primary.confirmed or secondary.confirmed
        primary.validated_observation_count += secondary.validated_observation_count
        primary.required_confirmation_count = max(
            primary.required_confirmation_count, secondary.required_confirmation_count
        )
        for source, count in secondary.source_counts.items():
            primary.source_counts[source] = primary.source_counts.get(source, 0) + count
        primary.localization_source = max(
            primary.source_counts,
            key=lambda source: (primary.source_counts[source], source == "fused"),
        )

    def final_deduplicate(self) -> None:
        changed = True
        while changed:
            changed = False
            tracks = self.tracks
            for index, primary in enumerate(tracks):
                for secondary in tracks[index + 1 :]:
                    if np.linalg.norm(primary.position - secondary.position) <= self.final_merge_distance:
                        self._merge_pair(primary, secondary)
                        del self._tracks[secondary.id]
                        changed = True
                        break
                if changed:
                    break

    def finalize(self) -> List[HazardTrack]:
        self.finalized = True
        self.final_deduplicate()
        return [track for track in self.tracks if track.confirmed]

    def visible_tracks(self, publish_candidates: bool) -> Iterable[HazardTrack]:
        return self.tracks if publish_candidates else [t for t in self.tracks if t.confirmed]
