"""Geometry helpers used by the per-camera AprilTag detector.

The detector (race_auv_camera_pkg/apriltag_detector_node.py) needs only:

* ``sanitize_rotation`` -- SVD projection of a near-rotation 3x3 onto SO(3).
  pupil_apriltags occasionally returns a slightly non-right-handed
  rotation for noisy / degenerate detections; without SVD projection
  ``scipy.Rotation.from_matrix`` raises ``ValueError: Non-positive
  determinant`` and the node crashes.
* ``is_bad_rotation`` -- cheap heuristic (non-finite / det <= 0 / far
  from orthogonal) used to drop a detection's pose from the published
  ``Detection3DArray`` without spending the SVD cost first.
* ``matrix_to_pose_msg`` -- 4x4 homogeneous -> ``geometry_msgs/Pose``,
  with the SVD sanitize step inside so a degenerate input cannot raise.
* ``load_yaml_config`` -- read the apriltag.yaml the launch file points
  this node at and return the inner ``apriltag:`` block.

TF helpers (``matrix_to_transform_stamped``), the Umeyama / SVD joint
SE(3) solver (``solve_cam_to_base``), and the URDF path resolver
(``resolve_urdf_path``) live in the fuser's separate copy of this module
at ``race_auv_sim_pkg/race_auv_sim_pkg/apriltag_geom.py`` -- they are
not used here because this package does not broadcast TF and does not
fit a multi-tag base pose.
"""

from __future__ import annotations

from pathlib import Path
from typing import Tuple

import numpy as np
import yaml
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R


# 8-tuple return shape of ``_yaml_intrinsics`` in the detector node.
# Documented here for cross-reference; not used at runtime.
#   (fx, fy, cx, cy, width, height, distortion[<=5], fisheye)
Intrinsics8 = Tuple[float, float, float, float, int, int, list, bool]


def sanitize_rotation(M: np.ndarray) -> np.ndarray:
    """Project a near-rotation 3x3 matrix onto SO(3) via SVD.

    Returns the closest proper rotation. If the SVD solution has a
    negative determinant, the last singular vector is flipped so the
    result is right-handed. Used to convert pupil_apriltags' output to
    a rotation that ``Rotation.from_matrix`` accepts without raising.
    """
    M = np.asarray(M, dtype=np.float64)
    U, _, Vt = np.linalg.svd(M)
    R_fixed = U @ Vt
    if np.linalg.det(R_fixed) < 0.0:
        Vt[-1, :] *= -1.0
        R_fixed = U @ Vt
    return R_fixed


def is_bad_rotation(M: np.ndarray) -> bool:
    """Heuristic: this 3x3 is not a usable proper rotation.

    True if the matrix is non-finite, has det <= 0, or is far from
    orthogonal. Used to drop a detection's pose before the more
    expensive SVD sanitize step.
    """
    M = np.asarray(M, dtype=np.float64)
    if not np.all(np.isfinite(M)):
        return True
    try:
        if np.linalg.det(M) <= 1e-6:
            return True
    except Exception:
        return True
    if not np.allclose(M @ M.T, np.eye(3), atol=1e-3):
        return True
    return False


def matrix_to_pose_msg(T: np.ndarray) -> Pose:
    """Homogeneous 4x4 -> ``geometry_msgs/Pose``. Robust to noisy rotations."""
    p = T[:3, 3]
    R_clean = sanitize_rotation(T[:3, :3])
    q = R.from_matrix(R_clean).as_quat()  # xyzw
    msg = Pose()
    msg.position.x = float(p[0])
    msg.position.y = float(p[1])
    msg.position.z = float(p[2])
    msg.orientation.x = float(q[0])
    msg.orientation.y = float(q[1])
    msg.orientation.z = float(q[2])
    msg.orientation.w = float(q[3])
    return msg


def load_yaml_config(path: str) -> dict:
    """Load the YAML config and return the inner ``apriltag:`` block.

    Falls back to a top-level dict if no ``apriltag:`` key is present.
    Returns an empty dict if ``path`` is empty or the file is missing.
    """
    if not path:
        return {}
    p = Path(str(path)).expanduser()
    if not p.is_file():
        return {}
    with open(p, "r") as f:
        cfg = yaml.safe_load(f) or {}
    return cfg.get("apriltag", cfg) or {}