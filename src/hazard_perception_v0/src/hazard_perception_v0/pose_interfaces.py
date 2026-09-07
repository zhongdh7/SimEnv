"""ROS-independent validation and quaternion helpers for pose adapters."""

import math


def finite_vector(values, expected_length):
    result = tuple(float(value) for value in values)
    return len(result) == expected_length and all(math.isfinite(value) for value in result)


def normalize_quaternion(quaternion):
    values = tuple(float(value) for value in quaternion)
    if not finite_vector(values, 4):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in values)


def quaternion_inverse(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return -x, -y, -z, w


def quaternion_multiply(left, right):
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def relative_orientation(initial, current):
    """Return inverse(q0) * q_current, both in xyzw convention."""
    return quaternion_multiply(quaternion_inverse(initial), current)


def orientation_only_status(valid=True, detail=""):
    """Describe the IMU adapter contract without implying a translated pose."""
    status = {
        "orientation_valid": bool(valid),
        "translation_valid": False,
        "global_position_valid": False,
        "mode": "orientation_only",
    }
    if detail:
        status["detail"] = str(detail)
    return status


def normalize_odometry_data(position, quaternion, parent_frame, child_frame,
                            parent_override="auto", base_override="base"):
    """Validate and standardize the pose portion of nav_msgs/Odometry."""
    position = tuple(float(value) for value in position)
    if not finite_vector(position, 3):
        raise ValueError("odometry position must contain three finite values")
    quaternion = normalize_quaternion(quaternion)
    parent = str(parent_frame).strip()
    child = str(child_frame).strip()
    if str(parent_override).strip().lower() != "auto":
        parent = str(parent_override).strip()
    if str(base_override).strip().lower() not in ("", "auto"):
        child = str(base_override).strip()
    if not parent or not child:
        raise ValueError("odometry parent and child frames must be non-empty")
    return {
        "position": position,
        "quaternion_xyzw": quaternion,
        "parent_frame": parent,
        "child_frame": child,
    }
