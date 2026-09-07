"""Online point-cloud projection and safe candidate scoring for room NBP."""

from __future__ import print_function

import json
import math
import os
import select
import struct
import subprocess
import threading
import time
from collections import OrderedDict

import numpy as np


class _InferenceProcess(object):
    def __init__(self, python_path, server_path, weights_path, device, timeout):
        self.python_path = python_path
        self.server_path = server_path
        self.weights_path = weights_path
        self.device = device
        self.timeout = float(timeout)
        self.process = None
        self.ready = False
        self.reason = "starting"
        self.lock = threading.Lock()
        self._starter = threading.Thread(target=self._start, daemon=True)
        self._starter.start()

    def _start(self):
        try:
            if not os.path.isfile(self.weights_path):
                raise IOError("weights not found: %s" % self.weights_path)
            if not os.path.isfile(self.server_path):
                raise IOError("server not found: %s" % self.server_path)
            self.process = subprocess.Popen(
                [self.python_path, self.server_path,
                 "--weights", self.weights_path, "--device", self.device],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=None,
                bufsize=0,
            )
            header = self._read_exact(4, max(30.0, self.timeout))
            if header != b"NBPR":
                raise RuntimeError("inference server did not become ready")
            self.ready = True
            self.reason = "ready"
        except Exception as exc:
            self.reason = "%s: %s" % (type(exc).__name__, exc)
            self.ready = False

    def _read_exact(self, size, timeout):
        if self.process is None or self.process.stdout is None:
            return None
        deadline = time.monotonic() + timeout
        chunks = []
        remaining = size
        fd = self.process.stdout.fileno()
        while remaining:
            wait = deadline - time.monotonic()
            if wait <= 0.0:
                raise TimeoutError("NBP response timed out")
            readable, _, _ = select.select([fd], [], [], wait)
            if not readable:
                raise TimeoutError("NBP response timed out")
            chunk = os.read(fd, remaining)
            if not chunk:
                raise EOFError("NBP server exited")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def infer(self, model_input):
        if not self.ready:
            raise RuntimeError(self.reason)
        array = np.ascontiguousarray(model_input, dtype=np.float32)
        if array.shape != (1, 5, 256, 256):
            raise ValueError("unexpected NBP input shape %r" % (array.shape,))
        with self.lock:
            try:
                self.process.stdin.write(b"NBP1")
                self.process.stdin.write(array.tobytes(order="C"))
                self.process.stdin.flush()
                header = self._read_exact(struct.calcsize("<4sddI"), self.timeout)
                magic, total_ms, model_ms, count = struct.unpack("<4sddI", header)
                if magic != b"NBPO" or count != 8 * 64 * 64:
                    raise RuntimeError("invalid NBP response")
                payload = self._read_exact(count * 4, self.timeout)
                values = np.frombuffer(payload, dtype=np.float32).reshape(8, 64, 64).copy()
                if not np.all(np.isfinite(values)):
                    raise RuntimeError("NBP value map contains NaN/Inf")
                return values, float(total_ms), float(model_ms)
            except Exception as exc:
                self.ready = False
                self.reason = "%s: %s" % (type(exc).__name__, exc)
                raise

    def close(self):
        process = self.process
        self.ready = False
        if process is None:
            return
        try:
            if process.stdin:
                process.stdin.close()
            process.terminate()
            process.wait(timeout=2.0)
        except Exception:
            try:
                process.kill()
            except Exception:
                pass


class RoomNBPPlanner(object):
    """Keeps room observations and ranks map-validated candidates by NBP value."""

    INPUT_SIZE = 256
    VALUE_SIZE = 64
    YAW_COUNT = 8

    def __init__(self, enabled, python_path, server_path, weights_path,
                 device="cuda", view_range=12.8, inference_timeout=12.0,
                 max_points=60000, voxel_size=0.08, log_path=""):
        self.enabled = bool(enabled)
        self.view_range = max(2.0, float(view_range))
        self.max_points = max(1000, int(max_points))
        self.voxel_size = max(0.02, float(voxel_size))
        self.log_path = os.path.expanduser(log_path) if log_path else ""
        self.lock = threading.RLock()
        self.room_id = None
        self.room_floor = None
        self.bounds = None
        self.floor_z = 0.0
        self.started_wall = None
        self.points = OrderedDict()
        self.trajectory = []
        self.last_pose = None
        self.target_count = 0
        self.fallback_count = 0
        self.inference_count = 0
        self._room_target_base = 0
        self._room_fallback_base = 0
        self._room_inference_base = 0
        self.backend = None
        if self.enabled:
            self.backend = _InferenceProcess(
                os.path.expanduser(python_path), os.path.expanduser(server_path),
                os.path.expanduser(weights_path), device, inference_timeout,
            )

    @property
    def ready(self):
        return bool(self.backend is not None and self.backend.ready)

    @property
    def reason(self):
        if not self.enabled:
            return "disabled"
        return self.backend.reason if self.backend is not None else "unavailable"

    def _log(self, event, **fields):
        if not self.log_path:
            return
        record = {
            "event": event, "wall_time": time.time(),
            "room_id": self.room_id, "floor": self.room_floor,
        }
        record.update(fields)
        try:
            directory = os.path.dirname(self.log_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, sort_keys=True) + "\n")
        except Exception:
            pass

    def begin_room(self, room_id, bounds, floor_z, pose, floor_index=None):
        with self.lock:
            if self.room_id == room_id:
                return
            self.room_id = int(room_id)
            self.room_floor = (int(floor_index)
                               if floor_index is not None else None)
            self.bounds = dict(bounds)
            self.floor_z = float(floor_z)
            self.started_wall = time.monotonic()
            self.points = OrderedDict()
            self.trajectory = []
            self.last_pose = None
            self.target_count = 0
            self.fallback_count = 0
            self.observe_pose(pose)
            self._room_target_base = self.target_count
            self._room_fallback_base = self.fallback_count
            self._room_inference_base = self.inference_count
            self._log("room_start", bounds=self.bounds, pose=list(pose), backend=self.reason)

    def log_event(self, event, **fields):
        """Write a navigation-state event into the room JSONL trace."""
        with self.lock:
            # Entry failures happen before begin_room(). Permit an explicit
            # discovered room_id so offline evaluation can distinguish
            # entry_status from room_search_status.
            if self.room_id is not None or fields.get("room_id") is not None:
                self._log(event, **fields)

    def finish_room(self, outcome, coverage, pose, camera_seen_ratio=None,
                    camera_seen_geometric_ratio=None,
                    hazard_visibility_ratio=None):
        with self.lock:
            if self.room_id is None:
                return
            room_id = self.room_id
            duration = (time.monotonic() - self.started_wall
                        if self.started_wall is not None else 0.0)
            finish_fields = {
                "outcome": outcome, "coverage": float(coverage),
                "pose": list(pose), "room_duration_s": duration,
                "targets": self.target_count - self._room_target_base,
                "fallbacks": self.fallback_count - self._room_fallback_base,
                "inferences": self.inference_count - self._room_inference_base,
            }
            if camera_seen_ratio is not None:
                finish_fields["camera_seen_ratio"] = float(camera_seen_ratio)
            if camera_seen_geometric_ratio is not None:
                finish_fields["camera_seen_geometric_ratio"] = float(
                    camera_seen_geometric_ratio
                )
            if hazard_visibility_ratio is not None:
                finish_fields["hazard_visibility_ratio"] = float(
                    hazard_visibility_ratio
                )
            self._log("room_finish", **finish_fields)
            self.room_id = None
            self.room_floor = None
            self.bounds = None
            self.points = OrderedDict()
            self.trajectory = []
            self.last_pose = None

    def observe_pose(self, pose):
        if self.room_id is None or pose is None:
            return
        point = (float(pose[0]), float(pose[1]), float(pose[2]))
        if (self.last_pose is None or
                math.hypot(point[0] - self.last_pose[0],
                           point[1] - self.last_pose[1]) >= 0.15):
            self.trajectory.append(point)
            self.last_pose = point

    def observe_scan(self, points, pose):
        with self.lock:
            if self.room_id is None or self.bounds is None:
                return
            self.observe_pose(pose)
            margin = 0.25
            for point in points:
                x, y, z = map(float, point)
                if not all(math.isfinite(value) for value in (x, y, z)):
                    continue
                if not (self.bounds["x_min"] - margin <= x <= self.bounds["x_max"] + margin and
                        self.bounds["y_min"] - margin <= y <= self.bounds["y_max"] + margin and
                        self.floor_z - 0.20 <= z <= self.floor_z + 2.60):
                    continue
                key = (int(round(x / self.voxel_size)),
                       int(round(y / self.voxel_size)),
                       int(round(z / self.voxel_size)))
                if key not in self.points:
                    self.points[key] = (x, y, z)
                    if len(self.points) > self.max_points:
                        self.points.popitem(last=False)

    def _project(self, pose):
        with self.lock:
            points = np.asarray(list(self.points.values()), dtype=np.float32)
            trajectory = np.asarray(self.trajectory, dtype=np.float32)
        result = np.zeros((1, 5, self.INPUT_SIZE, self.INPUT_SIZE), dtype=np.float32)
        if points.size:
            relative_z = points[:, 2] - self.floor_z
            bins = np.floor((relative_z + 0.20) / (2.80 / 4.0)).astype(np.int32)
            bins = np.clip(bins, 0, 3)
            self._map_points(result[0], points[:, :2], pose, bins)
        if trajectory.size:
            self._map_points(result[0], trajectory[:, :2], pose, None, channel=4)
        return result, int(len(points)), int(len(trajectory))

    def _map_points(self, target, xy, pose, bins=None, channel=None):
        # Matches official no_rotation=True projection semantics: a current-pose
        # centered, world-axis-aligned top view. ROS (x,y) replaces its (x,z).
        u = -(xy[:, 1] - float(pose[1]))
        v = -(xy[:, 0] - float(pose[0]))
        scale = self.INPUT_SIZE / (2.0 * self.view_range)
        rows = np.rint((u + self.view_range) * scale).astype(np.int32)
        cols = np.rint((v + self.view_range) * scale).astype(np.int32)
        valid = ((rows >= 0) & (rows < self.INPUT_SIZE) &
                 (cols >= 0) & (cols < self.INPUT_SIZE))
        rows, cols = rows[valid], cols[valid]
        if channel is not None:
            np.add.at(target[channel], (rows, cols), 1.0)
        else:
            valid_bins = bins[valid]
            np.add.at(target, (valid_bins, rows, cols), 1.0)

    def _value_pixel(self, world, pose):
        u = -(float(world[1]) - float(pose[1]))
        v = -(float(world[0]) - float(pose[0]))
        scale = self.VALUE_SIZE / (2.0 * self.view_range)
        row = int(round((u + self.view_range) * scale))
        col = int(round((v + self.view_range) * scale))
        if not (0 <= row < self.VALUE_SIZE and 0 <= col < self.VALUE_SIZE):
            return None
        return row, col

    def select(self, pose, candidates, coverage=None,
               camera_weight=0.0, path_weight=0.0, nbp_weight=1.0,
               hazard_weight=0.0):
        """Return the highest-value prevalidated candidate or None.

        Each candidate is a dictionary containing world=(x,y), cell, path and
        path_m. FREE/room/reachability validation belongs to the caller.
        """
        if not candidates:
            self.fallback_count += 1
            self._log("fallback", reason="no_legal_candidate")
            return None
        if not self.ready:
            self.fallback_count += 1
            self._log("fallback", reason=self.reason)
            return None
        model_input, point_count, trajectory_count = self._project(pose)
        started = time.perf_counter()
        try:
            value_map, server_ms, model_ms = self.backend.infer(model_input)
        except Exception as exc:
            self.fallback_count += 1
            self._log("fallback", reason="inference: %s" % exc)
            return None
        client_ms = (time.perf_counter() - started) * 1000.0
        self.inference_count += 1
        scored = []
        for candidate in candidates:
            pixel = self._value_pixel(candidate["world"], pose)
            if pixel is None:
                continue
            row, col = pixel
            visibility = candidate.get("camera_visibility", ())
            hazard_visibility = candidate.get("hazard_visibility", ())
            if not isinstance(visibility, (list, tuple, np.ndarray)):
                visibility = [visibility] * self.YAW_COUNT
            if not isinstance(hazard_visibility, (list, tuple, np.ndarray)):
                hazard_visibility = [hazard_visibility] * self.YAW_COUNT
            for yaw_channel in range(self.YAW_COUNT):
                value = float(value_map[yaw_channel, row, col])
                if not math.isfinite(value):
                    continue
                record = dict(candidate)
                record.update({
                    "value": value,
                    "yaw_channel": yaw_channel,
                    "yaw": yaw_channel * (2.0 * math.pi / self.YAW_COUNT),
                    "value_pixel": (row, col),
                    "camera_new_visibility": float(
                        visibility[yaw_channel]
                        if yaw_channel < len(visibility) else 0.0
                    ),
                    "hazard_new_visibility": float(
                        hazard_visibility[yaw_channel]
                        if yaw_channel < len(hazard_visibility) else 0.0
                    ),
                    "client_ms": client_ms,
                    "model_ms": model_ms,
                })
                scored.append(record)
        best = None
        if scored:
            # Preserve the exact historical argmax behavior when the new
            # task-layer terms are disabled (Baseline A).
            if (abs(float(camera_weight)) <= 1e-12 and
                    abs(float(path_weight)) <= 1e-12 and
                    abs(float(hazard_weight)) <= 1e-12):
                best = max(scored, key=lambda item: (item["value"], -item["path_m"]))
                best["score"] = float(best["value"])
                best["nbp_value_norm"] = 1.0
                best["camera_visibility_norm"] = 0.0
                best["path_norm"] = 0.0
            else:
                values = np.asarray([item["value"] for item in scored], dtype=np.float64)
                views = np.asarray(
                    [item["camera_new_visibility"] for item in scored], dtype=np.float64
                )
                hazards = np.asarray(
                    [item["hazard_new_visibility"] for item in scored], dtype=np.float64
                )
                paths = np.asarray([item["path_m"] for item in scored], dtype=np.float64)
                def _normalize(array):
                    low, high = float(np.min(array)), float(np.max(array))
                    return ((array - low) / (high - low)) if high > low else np.zeros_like(array)
                value_norm = _normalize(values)
                view_norm = _normalize(views)
                hazard_norm = _normalize(hazards)
                path_norm = _normalize(paths)
                scores = (float(nbp_weight) * value_norm +
                          float(camera_weight) * view_norm +
                          float(hazard_weight) * hazard_norm -
                          float(path_weight) * path_norm)
                best_index = int(np.argmax(scores))
                for index, record in enumerate(scored):
                    record["nbp_value_norm"] = float(value_norm[index])
                    record["camera_visibility_norm"] = float(view_norm[index])
                    record["hazard_visibility_norm"] = float(hazard_norm[index])
                    record["path_norm"] = float(path_norm[index])
                    record["score"] = float(scores[index])
                best = scored[best_index]
        inference_fields = {
            "client_ms": client_ms, "server_ms": server_ms,
            "model_ms": model_ms, "point_count": point_count,
            "trajectory_count": trajectory_count,
            "candidate_count": len(candidates),
        }
        if coverage is not None:
            inference_fields["coverage"] = float(coverage)
        self._log("inference", **inference_fields)
        if best is None:
            self.fallback_count += 1
            self._log("fallback", reason="no_candidate_in_value_map")
            return None
        self.target_count += 1
        target_fields = {
            "x": best["world"][0], "y": best["world"][1],
            "yaw": best["yaw"], "yaw_channel": best["yaw_channel"],
            "value": best["value"], "cell": list(best["cell"]),
            "path_m": best["path_m"],
            "path": [list(item) for item in best["path"]],
            "gain_cells": int(best.get("gain_cells", 0)),
            "client_ms": client_ms, "model_ms": model_ms,
            "score": float(best.get("score", best["value"])),
            "nbp_value_norm": float(best.get("nbp_value_norm", 1.0)),
            "camera_visibility_norm": float(best.get("camera_visibility_norm", 0.0)),
            "path_norm": float(best.get("path_norm", 0.0)),
            "camera_new_visibility": float(best.get("camera_new_visibility", 0.0)),
            "hazard_new_visibility": float(best.get("hazard_new_visibility", 0.0)),
            "hazard_visibility_norm": float(best.get("hazard_visibility_norm", 0.0)),
        }
        if coverage is not None:
            target_fields["coverage"] = float(coverage)
        self._log("target", **target_fields)
        return best

    def close(self):
        if self.backend is not None:
            self.backend.close()
