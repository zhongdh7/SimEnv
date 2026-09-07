import importlib.util
import json
import os
import tempfile
import unittest


def _load_tool():
    path = os.path.join(os.path.dirname(__file__), "..", "tools", "evaluate_floor0_visibility.py")
    spec = importlib.util.spec_from_file_location("floor0_visibility", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class VisibilityAuditTest(unittest.TestCase):
    def test_projects_gt_and_classifies_geometry_rejection(self):
        tool = _load_tool()
        with tempfile.TemporaryDirectory() as directory:
            diagnostics = os.path.join(directory, "diagnostics.jsonl")
            truth = os.path.join(directory, "truth.json")
            with open(diagnostics, "w") as stream:
                stream.write(json.dumps({
                    "stamp": 1.0, "frame_id": "camera",
                    "camera_info": {"fx": 100.0, "fy": 100.0, "cx": 50.0, "cy": 50.0,
                                    "width": 100, "height": 100},
                    "T_odom_camera": {"translation_xyz": [0, 0, 0],
                                      "quaternion_xyzw": [0, 0, 0, 1]},
                    "candidates": [{"bbox": [42, 42, 16, 16], "center": [50, 50],
                                    "depth_valid_pixels": 40,
                                    "geometry_status": "rejected",
                                    "geometry_reason": "sphere_residual"}],
                }) + "\n")
            with open(truth, "w") as stream:
                json.dump({"danger_sources": [{"id": 7, "position": [0, 0, 2],
                                                "floor_index": 0, "radius": 0.15}]}, stream)
            result = tool.audit(diagnostics, truth)
            hazard = result["hazards"][0]
            self.assertEqual(hazard["visible_frame_count"], 1)
            self.assertEqual(hazard["candidate_frame_count"], 1)
            self.assertEqual(hazard["depth_valid_frame_count"], 1)
            self.assertEqual(hazard["classification"], "depth_but_geometry_rejected")
            self.assertEqual(hazard["geometry_reject_reasons"]["sphere_residual"], 1)


if __name__ == "__main__":
    unittest.main()
