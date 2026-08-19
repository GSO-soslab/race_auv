"""Per-(family, size) AprilTag detection backed by ``pupil_apriltags``.

A single ``AprilTagDetector`` owns one ``pupil_apriltags.Detector``
configured for one tag family (e.g. ``tag36h11``) at one physical size,
and is responsible for a *set* of numeric ids within that family.
Because pupil_apriltags only supports a single ``tag_size`` per
``detect()`` call, tags of different sizes within the same family must
be handled by separate ``AprilTagDetector`` instances -- the parent node
groups tags by ``(family, size)`` and creates one instance per group.

Detections are filtered against ``tag_ids`` so a detector built for the
12.5 cm group never reports ids belonging to the 4 cm group, even
though the underlying library will happily decode them all.

On every detection the detector also draws:

* a colored bounding box (green = good pose, red = bad pose),
* the tag's axes via ``cv2.drawFrameAxes``,
* a label block stacked above the box: ``ID``, ``xyz``, ``rpy``.

All drawing happens on the caller-provided image in-place; ``detect``
returns structured data for the caller to publish as
``vision_msgs/Detection3DArray``.
"""

from __future__ import annotations

from typing import Iterable, Sequence

import cv2
import numpy as np
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R


# --- On-image drawing constants -------------------------------------------------
# Box / label colors (BGR).
_BOX_OK = (0, 255, 0)
_BOX_BAD = (0, 0, 255)
_TEXT_PRIMARY = (255, 0, 255)   # ID
_TEXT_SECONDARY = (0, 255, 255) # xyz / rpy

# Font and stroke sizing.
_FONT = cv2.FONT_HERSHEY_SIMPLEX
_FONT_SCALE_BIG = 0.55
_FONT_SCALE_SMALL = 0.5
_TEXT_THICKNESS = 1
_LINE_THICKNESS = 2

# Per-text vertical stride (pixels) so the stacked label block is evenly spaced.
_LABEL_STRIDE = 18


def _rotation_matrix_to_euler_xyz(Rmat: np.ndarray) -> tuple[float, float, float]:
    """Roll / pitch / yaw (degrees) of ``Rmat`` in scipy's ``'xyz'`` order.

    scipy is already imported in this module (used elsewhere); the cost
    is negligible for the few detections per frame. The closed-form
    shortcut is tempting but the sign conventions are easy to get
    wrong -- scipy's reference implementation is the source of truth
    for on-image labels.
    """
    e = R.from_matrix(Rmat).as_euler("xyz", degrees=True)
    return float(e[0]), float(e[1]), float(e[2])


def _normalize_family(raw) -> str:
    """Decode pupil_apriltags' ``tag.tag_family`` (a ``bytes`` blob from
    ``ctypes.string_at``) into a plain ``str`` for comparison.

    Earlier code used ``str(raw)``, which on Python 3 produces
    ``"b'tag36h11'"`` (with the ``b''`` wrapper) and silently rejected
    every detection. Pupil_apriltags consistently returns the canonical
    lowercase family name, so a UTF-8 decode is safe.
    """
    if isinstance(raw, (bytes, bytearray)):
        return raw.decode("utf-8")
    return str(raw)


class AprilTagDetector:
    """Detect AprilTags from one family at one physical size.

    Parameters
    ----------
    family
        The pupil_apriltags family name, e.g. ``"tag36h11"``.
    tag_size
        Physical side length of every tag in this group, in meters. All
        tags handled by one ``AprilTagDetector`` must share the same
        size.
    tag_ids
        Iterable of integer ids this detector is allowed to report.
        Any detection whose ``tag_id`` is not in this set is dropped,
        even though pupil_apriltags can decode it.
    camera_intrinsics
        Dict with keys ``fx``, ``fy``, ``cx``, ``cy`` (pixels) for the
        *rectified* image the detector will receive.
    camera_distortion
        Distortion coefficients for the rectified image. pupil_apriltags
        does not use these for its internal pose solve; they are kept
        here so ``cv2.drawFrameAxes`` can label its axes.
    image_size
        Dict with ``img_width`` and ``img_height`` of the rectified
        image.
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

        # pupil-apriltags takes camera params as a simple list/tuple: [fx, fy, cx, cy].
        self.camera_params = (
            float(camera_intrinsics["fx"]),
            float(camera_intrinsics["fy"]),
            float(camera_intrinsics["cx"]),
            float(camera_intrinsics["cy"]),
        )

        # NOTE: pupil_apriltags does not use distortion coefficients for its
        # internal pose estimation. The caller must feed an undistorted
        # image for pose accuracy. The provided coefficients are only used
        # for drawing the axes.
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

        try:
            self.detector = Detector(families=self.family, **detector_params)
            self.logger.info(
                f"pupil_apriltags ready: family='{self.family}' "
                f"tag_size={self.tag_size:.4f} m ids={sorted(self.tag_ids)} "
                f"image={self.img_width}x{self.img_height} "
                f"params={dict(detector_params)}"
            )
        except Exception as e:
            self.logger.error(f"Failed to create pupil_apriltags detector: {e}")
            self.detector = None

    # --------------------------------------------------------------------- detect
    def detect(self, gray_image: np.ndarray) -> list[dict]:
        """Run detection on a grayscale image and return structured results.

        Each result is a dict with keys ``family``, ``tag_id``, ``T``
        (4x4 camera->tag), ``corners`` (Nx2 float), ``bad_pose`` (bool).
        All returned ids are guaranteed to be in ``self.tag_ids`` and to
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
            tag_family = _normalize_family(tag.tag_family)
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

            out.append({
                "family": self.family,
                "tag_id": tag_id,
                "T": T,
                "corners": corners,
                "bad_pose": bad_pose,
            })
        return out

    # ------------------------------------------------------------------ annotate
    def annotate(self, image: np.ndarray, detections: list[dict]) -> np.ndarray:
        """Draw bounding boxes, axes, and id labels for ``detections`` in-place.

        Green boxes indicate a usable pose; red boxes indicate a pose
        that was rejected (very rare). The label block above each box
        shows ``ID: <family>:<id>``, ``xyz``, and ``rpy``. ``rpy`` and
        axes are only drawn for usable poses.

        Returns ``image`` for chaining convenience.
        """
        if image is None:
            return image

        for det in detections:
            tag_id = det["tag_id"]
            corners = det["corners"].astype(int)
            T = det["T"]
            bad = det["bad_pose"]

            if bad:
                self._draw_bad_pose(image, corners, tag_id)
                continue

            tvec = T[:3, 3]
            Rmat = T[:3, :3]
            try:
                rvec, _ = cv2.Rodrigues(Rmat)
            except (ValueError, cv2.error) as e:
                self.logger.warn(
                    f"Could not convert rotation for {self.family}:{tag_id}; "
                    f"marking as bad pose. Error: {e}"
                )
                self._draw_bad_pose(image, corners, tag_id)
                continue

            roll, pitch, yaw = _rotation_matrix_to_euler_xyz(Rmat)

            # box + axes
            cv2.polylines(
                image, [corners], isClosed=True, color=_BOX_OK, thickness=_LINE_THICKNESS,
            )
            # cv2.drawFrameAxes(
            #     image,
            #     self.camera_matrix,
            #     self.dist_coeffs,
            #     rvec,
            #     tvec,
            #     self.tag_size * 0.5,
            # )

            # stacked label block above the box
            anchor_x = int(corners[:, 0].min())
            anchor_y = int(corners[:, 1].min()) - _LABEL_STRIDE
            lines = [
                (f"ID: {self.family}:{tag_id}",  _TEXT_PRIMARY,   _FONT_SCALE_BIG,  _TEXT_THICKNESS),
                (f"xyz: ({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f})",
                                                   _TEXT_SECONDARY, _FONT_SCALE_SMALL, _TEXT_THICKNESS),
                (f"rpy: ({roll:.0f}, {pitch:.0f}, {yaw:.0f})",
                                                   _TEXT_SECONDARY, _FONT_SCALE_SMALL, _TEXT_THICKNESS),
            ]
            self._draw_label_block(image, anchor_x, anchor_y, lines)

        return image

    # --------------------------------------------------------------------- helpers
    def _draw_bad_pose(
        self,
        image: np.ndarray,
        corners: np.ndarray,
        tag_id: int,
    ) -> None:
        """Draw the red box + id label used when a detection's pose is rejected."""
        cv2.polylines(
            image, [corners], isClosed=True, color=_BOX_BAD, thickness=_LINE_THICKNESS,
        )
        anchor = tuple(int(v) for v in corners[0])
        cv2.putText(
            image,
            f"ID: {self.family}:{tag_id} (Bad Pose)",
            (anchor[0], anchor[1] - 5),
            _FONT, 0.7, _BOX_BAD, 2,
        )

    @staticmethod
    def _draw_label_block(
        image: np.ndarray,
        x: int,
        start_y: int,
        lines: list[tuple[str, tuple[int, int, int], float, int]],
    ) -> None:
        """Stack ``lines`` vertically starting at ``(x, start_y)``.

        Each line is ``(text, color_bgr, font_scale, thickness)``. Each
        subsequent line is placed ``_LABEL_STRIDE`` pixels below the
        previous one.
        """
        y = start_y
        for text, color, scale, thick in lines:
            cv2.putText(image, text, (x, y), _FONT, scale, color, thick, cv2.LINE_AA)
            y += _LABEL_STRIDE