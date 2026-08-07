"""Per-(family, size) AprilTag detection backed by ``pupil_apriltags``.

The detector owns a single ``pupil_apriltags.Detector`` instance configured
for one tag family (e.g. ``tag36h11``) and one physical tag size, but it is
responsible for a *set* of numeric ids within that family. Because the
underlying library only supports a single ``tag_size`` per ``detect()``
call, tags of different sizes within the same family must be handled by
separate detector instances (see ``apriltag_detector_node.py``).

Detection results are filtered against ``tag_ids`` so a detector that was
built for the 12.5 cm group does not report ids belonging to the 4 cm
group, even though the underlying library would happily decode them all.
"""

from __future__ import annotations

from typing import Iterable, Sequence

import cv2
import numpy as np
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R


class AprilTagDetector:
    """Detect AprilTags from one family at one physical size.

    The detector draws bounding boxes, axes, and id labels on the input
    image in-place (returning the same array for convenience) and returns
    a list of structured detections so callers can publish them.

    Parameters
    ----------
    family
        The pupil_apriltags family name, e.g. ``"tag36h11"``.
    tag_size
        Physical side length of every tag in this group, in meters. All
        tags handled by one ``AprilTagDetector`` must share the same size.
    tag_ids
        Iterable of integer ids this detector is allowed to report. Any
        detection whose ``tag_id`` is not in this set is dropped, even
        though pupil_apriltags can decode it.
    camera_intrinsics
        Dict with keys ``fx``, ``fy``, ``cx``, ``cy`` (pixels) for the
        *rectified* image the detector will receive.
    camera_distortion
        Distortion coefficients for the rectified image. pupil_apriltags
        does not use these for its internal pose solve; they are kept
        here only so axes can be drawn with ``cv2.drawFrameAxes``.
    image_size
        Dict with ``img_width`` and ``img_height`` of the rectified image.
    logger
        A ``rclpy``-compatible logger.
    detector_params
        Dict of tuning parameters forwarded to ``pupil_apriltags.Detector``
        (``nthreads``, ``quad_decimate``, ``quad_sigma``, ``refine_edges``,
        ``decode_sharpening``).
    """

    def __init__(
        self,
        family: str,
        tag_size: float,
        tag_ids: Iterable[int],
        camera_intrinsics: dict,
        camera_distortion: Sequence[float],
        image_size: dict,
        logger,
        detector_params: dict,
    ):
        self.logger = logger
        self.family = str(family)
        self.tag_size = float(tag_size)
        self.tag_ids: set[int] = {int(t) for t in tag_ids}

        # pupil-apriltags takes camera params as a simple list/tuple: [fx, fy, cx, cy]
        self.camera_params = (
            float(camera_intrinsics["fx"]),
            float(camera_intrinsics["fy"]),
            float(camera_intrinsics["cx"]),
            float(camera_intrinsics["cy"]),
        )

        # NOTE: pupil-apriltags does not use distortion coefficients for its
        # internal pose estimation. The user should provide an undistorted
        # image if pose accuracy is critical. The provided coefficients are
        # only used for drawing the axes, so we build the matrix here.
        self.dist_coeffs = np.array(camera_distortion, dtype=np.float32)
        self.camera_matrix = np.array(
            [
                [self.camera_params[0], 0, self.camera_params[2]],
                [0, self.camera_params[1], self.camera_params[3]],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )

        self.img_width = int(image_size["img_width"])
        self.img_height = int(image_size["img_height"])

        # --- Create pupil-apriltags Detector ---
        try:
            self.detector = Detector(families=self.family, **detector_params)
            self.logger.info(
                f"pupil_apriltags detector ready: family='{self.family}' "
                f"tag_size={self.tag_size:.4f} m ids={sorted(self.tag_ids)} "
                f"image={self.img_width}x{self.img_height} "
                f"params={dict(detector_params)}"
            )
        except Exception as e:
            self.logger.error(f"Failed to create pupil_apriltags detector: {e}")
            self.detector = None

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _rotation_matrix_to_euler_angles(R_matrix: np.ndarray):
        """Convert a 3x3 rotation matrix to roll/pitch/yaw Euler angles in degrees."""
        return R.from_matrix(R_matrix).as_euler("xyz", degrees=True)

    @staticmethod
    def _family_short(family: str) -> str:
        """``"tag36h11"`` -> ``"36h11"``. Used to build the TF frame name
        (which mirrors the URDF link name, e.g. ``apriltag36h11_5``)."""
        return family[3:] if family.startswith("tag") else family

    @staticmethod
    def _normalize_family(raw) -> str:
        """Decode ``tag.tag_family`` (a ``bytes`` blob from
        ``ctypes.string_at``) into a plain ``str`` for comparison.

        Earlier versions used ``str(raw)``, which on Python 3 produces
        ``"b'tag36h11'"`` (with the ``b''`` wrapper) and silently
        rejected every detection. Pupil_apriltags consistently returns
        the canonical lowercase family name, so a UTF-8 decode is safe.
        """
        if isinstance(raw, (bytes, bytearray)):
            return raw.decode("utf-8")
        return str(raw)

    # ----------------------------------------------------------------- detect
    def detect(self, gray_image: np.ndarray) -> list[dict]:
        """Run detection on a grayscale image and return structured results.

        Returns a list of dicts with keys ``family``, ``tag_id``, ``T`` (4x4
        camera->tag), ``corners`` (Nx2 float), ``bad_pose`` (bool). All
        returned ids are guaranteed to be in ``self.tag_ids`` and to
        belong to ``self.family``.
        """
        if self.detector is None:
            self.logger.error("AprilTag detector is not initialized; skipping.")
            return []
        if gray_image is None:
            self.logger.warn("Received a null image for AprilTag detection.")
            return []

        raw = self.detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )

        out: list[dict] = []
        for tag in raw:
            # Defensive: pupil_apriltags shouldn't return another family,
            # but guard against cross-family false positives anyway. The
            # detector was built for one family only, so anything else
            # here would be a bug or a false positive.
            tag_family = self._normalize_family(tag.tag_family)
            if tag_family != self.family:
                continue
            try:
                tag_id = int(tag.tag_id)
            except (TypeError, ValueError):
                continue
            if tag_id not in self.tag_ids:
                continue

            corners = np.asarray(tag.corners, dtype=np.float32)
            T = np.eye(4)
            try:
                T[:3, :3] = tag.pose_R
                T[:3, 3] = tag.pose_t.flatten()
                bad_pose = False
            except (AttributeError, TypeError, ValueError):
                bad_pose = True

            out.append(
                {
                    "family": self.family,
                    "tag_id": tag_id,
                    "T": T,
                    "corners": corners,
                    "bad_pose": bad_pose,
                }
            )
        return out

    # ---------------------------------------------------------------- drawing
    def annotate(self, image: np.ndarray, detections: list[dict]) -> np.ndarray:
        """Draw bounding boxes, axes, and id labels for ``detections`` in-place.

        Green boxes indicate a detection with a usable pose; red boxes
        indicate a detection whose pose was rejected (very rare).
        On-image labels use the full pupil_apriltags family name
        (e.g. ``tag36h11:5``) so they match ``Detection3D.id`` and the
        URDF link naming convention. Returns the same image array for
        chaining convenience.
        """
        if image is None:
            return image
        # Full family name (e.g. "tag36h11") for on-image labels; the TF
        # frame name still uses the shortened form "apriltag36h11_<id>".
        for det in detections:
            tag_id = det["tag_id"]
            corners = det["corners"].astype(int)
            T = det["T"]
            bad = det["bad_pose"]

            if bad:
                cv2.polylines(
                    image, [corners], isClosed=True, color=(0, 0, 255), thickness=2
                )
                anchor = tuple(int(v) for v in corners[0])
                cv2.putText(
                    image,
                    f"ID: {self.family}:{tag_id} (Bad Pose)",
                    (anchor[0], anchor[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                continue

            try:
                tvec = T[:3, 3]
                Rmat = T[:3, :3]
                rvec, _ = cv2.Rodrigues(Rmat)
                roll, pitch, yaw = self._rotation_matrix_to_euler_angles(Rmat)
            except (ValueError, cv2.error) as e:
                self.logger.warn(
                    f"Could not process pose for tag {self.family}:{tag_id}. "
                    f"Marking as bad pose. Error: {e}"
                )
                cv2.polylines(
                    image, [corners], isClosed=True, color=(0, 0, 255), thickness=2
                )
                anchor = tuple(int(v) for v in corners[0])
                cv2.putText(
                    image,
                    f"ID: {self.family}:{tag_id} (Bad Pose)",
                    (anchor[0], anchor[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                continue

            cv2.polylines(
                image, [corners], isClosed=True, color=(0, 255, 0), thickness=2
            )
            # cv2.drawFrameAxes(
            #     image,
            #     self.camera_matrix,
            #     self.dist_coeffs,
            #     rvec,
            #     tvec,
            #     self.tag_size * 0.5,
            # )

            center = tuple(int(v) for v in det["corners"].mean(axis=0))
            # cv2.putText(
            #     image,
            #     f"ID: {self.family}:{tag_id}",
            #     (center[0] - 125, center[1] + 20),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     0.7,
            #     (255, 0, 255),
            #     2,
            # )
            cv2.putText(
                image,
                f"ID: {tag_id}",
                (center[0] - 50, center[1] + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 255),
                1,
            )
            cv2.putText(
                image,
                f"x:{tvec[0]:.1f},y:{tvec[1]:.1f}",
                (center[0] - 50, center[1] - 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )
            cv2.putText(
                image,
                f"z: {tvec[2]:.2f}",
                (center[0] - 50, center[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
            # cv2.putText(
            #     image,
            #     f"({tvec[0]:.2f},{tvec[1]:.2f},{tvec[2]:.2f})",
            #     (center[0] - 50, center[1] - 20),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     0.6,
            #     (0, 255, 255),
            #     2,
            # )
            # cv2.putText(
            #     image,
            #     f"rpy: ({roll:.0f}, {pitch:.0f}, {yaw:.0f})",
            #     (center[0] - 125, center[1] - 25),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     0.85,
            #     (255, 255, 0),
            #     2,
            # )
        return image
