"""Geometry / URDF helpers shared by the AprilTag pipeline nodes.

Both ``apriltag_detector_node`` (per-camera detector) and
``apriltag_fuser_node`` (multi-camera fuser) need the same primitives:

* SVD-based rotation sanitization (pupil_apriltags occasionally returns
  slightly non-right-handed rotation matrices for noisy / degenerate
  detections, which crashes ``scipy.Rotation.from_matrix``).
* Bad-rotation heuristic to filter out untrustworthy detections.
* 4x4 homogeneous <-> ROS message conversion (Pose / TransformStamped).
* The Umeyama / SVD joint SE(3) solver used to fit one rigid
  ``T_camera_to_base`` to a set of (observed, known) tag-pose pairs.
* The YAML-driven URDF path resolution used by the bridge launches.

Centralizing these keeps the detector and the fuser byte-for-byte aligned
on the math (and the small numerical edge cases), so a fix in one place
applies to both.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml
from geometry_msgs.msg import Pose, TransformStamped
from scipy.spatial.transform import Rotation as R


def sanitize_rotation(M: np.ndarray) -> np.ndarray:
    """Project a near-rotation 3x3 matrix onto SO(3) via SVD.

    pupil_apriltags can return a slightly non-orthogonal rotation when
    the tag is noisy or its pose is degenerate; the matrix's determinant
    may end up negative or zero, which makes ``scipy.Rotation.from_matrix``
    raise ``ValueError: Non-positive determinant``. This helper returns
    the closest proper rotation so downstream quaternion conversion
    never crashes.
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
    orthogonal. Used to decide whether a detection's pose is trustworthy
    enough to be included in the joint base-pose solve.
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
    """Homogeneous 4x4 -> geometry_msgs/Pose. Robust to noisy rotations."""
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


def matrix_to_transform_stamped(
    T: np.ndarray, parent: str, child: str, stamp,
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    p = T[:3, 3]
    R_clean = sanitize_rotation(T[:3, :3])
    q = R.from_matrix(R_clean).as_quat()
    msg.transform.translation.x = float(p[0])
    msg.transform.translation.y = float(p[1])
    msg.transform.translation.z = float(p[2])
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])
    return msg


def solve_cam_to_base(
    pairs: List[Tuple[np.ndarray, np.ndarray]],
) -> Optional[np.ndarray]:
    """Joint SE(3) solve: one rigid transform explaining all tag observations.

    Parameters
    ----------
    pairs
        List of ``(T_cam_to_tag_observed, T_base_to_tag_known)`` 4x4
        transforms. ``T_base_to_tag_known`` comes from the URDF.

    Returns
    -------
    np.ndarray or None
        The single ``T_cam_to_base`` that best satisfies
        ``T_cam_to_tag_i ~= T_cam_to_base @ T_base_to_tag_i`` for every
        pair, in the least-squares sense. ``None`` if fewer than 3 pairs
        are supplied (3 non-collinear points are needed for a unique
        SE(3) solution).

    Notes
    -----
    This is the classic Umeyama / ArUco ``estimatePoseBoard``
    formulation: point correspondences are the tag centers in the base
    and camera frames, and we solve for the rigid transform that aligns
    them. Doing it jointly over all detected tags is essential when the
    tags sit at different orientations on the base link, because
    averaging per-tag inverse solutions would mix inconsistent
    orientation estimates.
    """
    if len(pairs) < 3:
        return None

    P = np.stack([T_b[:3, 3] for _, T_b in pairs], axis=0)  # base-frame points
    Q = np.stack([T_c[:3, 3] for T_c, _ in pairs], axis=0)  # cam-frame points
    p_bar = P.mean(axis=0)
    q_bar = Q.mean(axis=0)
    Pc = P - p_bar
    Qc = Q - q_bar
    # Covariance H: minimizes ||Q_c - R P_c||_F^2 over R in SO(3).
    H = Qc.T @ Pc
    U, _, Vt = np.linalg.svd(H)
    R_sol = U @ Vt
    if np.linalg.det(R_sol) < 0.0:
        Vt[-1, :] *= -1.0
        R_sol = U @ Vt
    t_sol = q_bar - R_sol @ p_bar

    T = np.eye(4)
    T[:3, :3] = R_sol
    T[:3, 3] = t_sol
    return T


def resolve_urdf_path(obj_cfg: dict) -> str:
    """Resolve a URDF path from ``object.urdf_package`` + ``object.urdf_filename``.

    Equivalent to ``description.launch.py``'s ``get_package_share_directory``
    pattern. Returns an empty string if the package name is missing.
    """
    pkg = obj_cfg.get("urdf_package") or obj_cfg.get("urdf_path_pkg")
    rel = obj_cfg.get("urdf_filename") or obj_cfg.get("urdf_path_in_pkg")
    if not pkg:
        return ""
    if not rel:
        rel = "urdf/base.urdf"
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(str(pkg))
    except Exception as e:
        raise RuntimeError(
            f"Could not resolve ROS package '{pkg}' for URDF lookup: {e}"
        )
    return str(Path(share) / rel)


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