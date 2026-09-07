"""Hazard perception algorithms and dependency-light integration helpers.

Detector symbols remain available from the package root, but are loaded only
when requested.  This keeps offline evaluation and pose adapters independent
of OpenCV while preserving the existing public import API.
"""

__all__ = ["Detection", "DetectorConfig", "RedSphereDetector"]


def __getattr__(name):
    if name in __all__:
        from .detector import Detection, DetectorConfig, RedSphereDetector
        return {
            "Detection": Detection,
            "DetectorConfig": DetectorConfig,
            "RedSphereDetector": RedSphereDetector,
        }[name]
    raise AttributeError("module {!r} has no attribute {!r}".format(__name__, name))
