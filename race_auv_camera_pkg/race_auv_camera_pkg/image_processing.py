"""Image rectification for the AprilTag pipeline.

Two backend classes share the same API:

* ``ImageRectifier`` -- CPU. Builds pre-computed OpenCV remap maps once
  and rectifies every frame with ``cv2.remap``. Universally available.
* ``CUDAImageRectifier`` -- GPU via ``cv2.cuda``. Uploads the remap
  maps once and runs ``cv2.cuda.remap`` per frame. Used when the
  ``apriltag.yaml`` ``detector_defaults.use_cuda`` is true and a
  CUDA-enabled OpenCV is present. Falls back to ``ImageRectifier`` if
  construction fails.

Both produce:

* the rectified, ROI-cropped BGR image (numpy array, CPU),
* the rectified image's intrinsics (fx, fy, cx, cy) and size,
* zero distortion coefficients (a rectified image is, by construction,
  distortion-free).

The Jetson / Orin build of OpenCV exposes ``cv2.cuda`` and the
``cv2.cuda.remap`` / ``cv2.cuda.cvtColor`` primitives that make this
backend ~3-5x faster than the CPU path on 1600x1200 frames.
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple

import cv2
import numpy as np


_logger = logging.getLogger(__name__)


# --- Public type returned by both rectifiers ------------------------------------
class RectifiedIntrinsics(dict):
    """Plain dict subclass so callers can treat the result like a dict.

    Keys: ``fx``, ``fy``, ``cx``, ``cy``, ``img_width``, ``img_height``,
    ``distortion`` (always an array of zeros).
    """


# =============================================================================
# CPU backend
# =============================================================================
class ImageRectifier:
    """CPU image rectifier.

    Pre-computes remap maps for either the fisheye or standard
    (plumb-bob) distortion model and exposes ``rectify(image)`` that
    undistorts and ROI-crops an image in one call.

    Parameters
    ----------
    logger
        Logger used for init messages.
    camera_matrix
        3x3 intrinsic matrix K.
    dist_coeffs
        1D distortion coefficients (4 for fisheye, 5 for plumb-bob).
    image_size
        ``(width, height)`` of the *original* (distorted) image.
    is_fisheye
        ``True`` for the fisheye model (``cv2.fisheye.*``),
        ``False`` for the standard plumb-bob model (``cv2.undistort``).
    crop_to_valid_pixels
        ``True`` to crop to the largest bounding rectangle of valid
        (non-black) pixels, removing the borders introduced by
        rectification.
    """

    def __init__(
        self,
        logger,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        image_size: Tuple[int, int],
        is_fisheye: bool,
        crop_to_valid_pixels: bool,
    ) -> None:
        self._logger = logger
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs
        self._image_size = image_size
        self._is_fisheye = bool(is_fisheye)
        self._crop = bool(crop_to_valid_pixels)
        self.map1: Optional[np.ndarray] = None
        self.map2: Optional[np.ndarray] = None

        self._logger.info(
            f"ImageRectifier init: backend=cpu fisheye={self._is_fisheye} "
            f"crop={self._crop} size={self._image_size[0]}x{self._image_size[1]}"
        )

        if self._is_fisheye:
            self._init_fisheye()
        else:
            self._init_standard()

        # ROI -> final image dims + adjusted principal point.
        self._new_width = self.roi[2]
        self._new_height = self.roi[3]
        self._final_camera_matrix = self.new_camera_matrix.copy()
        self._final_camera_matrix[0, 2] -= self.roi[0]  # cx adjustment
        self._final_camera_matrix[1, 2] -= self.roi[1]  # cy adjustment

        self._logger.info(
            f"Rectifier ready: {self._new_width}x{self._new_height} "
            f"fx={self._final_camera_matrix[0, 0]:.2f} "
            f"fy={self._final_camera_matrix[1, 1]:.2f} "
            f"cx={self._final_camera_matrix[0, 2]:.2f} "
            f"cy={self._final_camera_matrix[1, 2]:.2f}"
        )

    def _init_fisheye(self) -> None:
        d = np.asarray(self._dist_coeffs, dtype=np.float64).ravel()[:4]
        if d.size != 4:
            raise ValueError(
                f"Fisheye model requires exactly 4 distortion coefficients "
                f"(k1, k2, k3, k4); got {self._dist_coeffs.size}."
            )
        if self._dist_coeffs.size > 4:
            self._logger.warn(
                f"Fisheye model received {self._dist_coeffs.size} distortion "
                "coefficients; using the first 4 (k1, k2, k3, k4) and ignoring "
                "the rest."
            )

        # balance=0.0 crops to valid pixels, balance=1.0 shows all pixels.
        balance = 0.0 if self._crop else 1.0
        self.new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self._camera_matrix, d, self._image_size, np.eye(3), balance=balance,
        )
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self._camera_matrix, d, np.eye(3), self.new_camera_matrix,
            self._image_size, cv2.CV_16SC2,
        )

        if self._crop:
            mask = np.ones(self._image_size[::-1], dtype=np.uint8) * 255  # H, W
            undistorted_mask = cv2.remap(mask, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            contours, _ = cv2.findContours(
                undistorted_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
            )
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                self.roi = cv2.boundingRect(largest_contour)
            else:
                self._logger.warn(
                    "No contour in fisheye undistorted mask; not cropping."
                )
                self.roi = (0, 0, self._image_size[0], self._image_size[1])
        else:
            self.roi = (0, 0, self._image_size[0], self._image_size[1])

    def _init_standard(self) -> None:
        if len(self._dist_coeffs) < 4:
            self._logger.warn(
                f"Standard model with only {len(self._dist_coeffs)} distortion "
                "coefficients; expected at least 4 (k1, k2, p1, p2)."
            )
        alpha = 0.0 if self._crop else 1.0
        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self._camera_matrix, self._dist_coeffs, self._image_size, alpha, self._image_size,
        )

    def rectify(self, image: np.ndarray) -> np.ndarray:
        """Rectify and ROI-crop ``image``. Returns a new BGR array."""
        if self._is_fisheye:
            rect_img = cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        else:
            rect_img = cv2.undistort(
                image, self._camera_matrix, self._dist_coeffs,
                None, self.new_camera_matrix,
            )
        x, y, w, h = self.roi
        return rect_img[y:y + h, x:x + w]

    def get_intrinsics(self) -> RectifiedIntrinsics:
        """Intrinsics for the rectified (and ROI-cropped) image."""
        return RectifiedIntrinsics(
            fx=float(self._final_camera_matrix[0, 0]),
            fy=float(self._final_camera_matrix[1, 1]),
            cx=float(self._final_camera_matrix[0, 2]),
            cy=float(self._final_camera_matrix[1, 2]),
            img_width=int(self._new_width),
            img_height=int(self._new_height),
            distortion=np.zeros(5, dtype=np.float32),
        )


# =============================================================================
# CUDA backend
# =============================================================================
class CUDAImageRectifier:
    """GPU image rectifier using ``cv2.cuda``.

    Mirrors the CPU ``ImageRectifier`` API so callers can swap between
    them. Per-frame cost on Jetson Orin (~2-4 ms for 1600x1200) is
    dominated by the GPU remap; BGR->gray is done on the GPU too if
    ``rectify_bgr_gray`` is used.

    Construction will raise ``RuntimeError`` if CUDA is not available;
    callers should treat that as a signal to fall back to ``ImageRectifier``.
    """

    def __init__(
        self,
        logger,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        image_size: Tuple[int, int],
        is_fisheye: bool,
        crop_to_valid_pixels: bool,
    ) -> None:
        if cv2.cuda.getCudaEnabledDeviceCount() <= 0:
            raise RuntimeError("cv2.cuda reports no CUDA-enabled devices.")

        self._logger = logger
        self._image_size = image_size
        self._is_fisheye = bool(is_fisheye)
        self._crop = bool(crop_to_valid_pixels)

        # Build the CPU remap maps first using the same logic as the CPU
        # backend, then upload them to the GPU.
        cpu = ImageRectifier(
            logger=logger,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_size=image_size,
            is_fisheye=is_fisheye,
            crop_to_valid_pixels=crop_to_valid_pixels,
        )
        self._roi = cpu.roi
        self._new_width = cpu._new_width
        self._new_height = cpu._new_height
        self._final_camera_matrix = cpu._final_camera_matrix
        self._gpu_map1 = cv2.cuda_GpuMat()
        self._gpu_map2 = cv2.cuda_GpuMat()
        self._gpu_map1.upload(cpu.map1)
        self._gpu_map2.upload(cpu.map2)
        self._stream = cv2.cuda_Stream()

        self._logger.info(
            f"ImageRectifier init: backend=cuda fisheye={self._is_fisheye} "
            f"crop={self._crop} size={self._image_size[0]}x{self._image_size[1]}"
        )
        self._logger.info(
            f"Rectifier ready: {self._new_width}x{self._new_height} "
            f"fx={self._final_camera_matrix[0, 0]:.2f} "
            f"fy={self._final_camera_matrix[1, 1]:.2f} "
            f"cx={self._final_camera_matrix[0, 2]:.2f} "
            f"cy={self._final_camera_matrix[1, 2]:.2f}"
        )

    # --------------------------------------------------------------------- API
    def rectify(self, image: np.ndarray) -> np.ndarray:
        """Rectify ``image`` on the GPU and download the BGR result."""
        gpu_in = cv2.cuda_GpuMat()
        gpu_in.upload(image, self._stream)
        gpu_rect = cv2.cuda.remap(
            gpu_in, self._gpu_map1, self._gpu_map2,
            interpolation=cv2.INTER_LINEAR, stream=self._stream,
        )
        rect_img = gpu_rect.download()
        x, y, w, h = self._roi
        return rect_img[y:y + h, x:x + w]

    def rectify_bgr_and_gray(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Rectify ``image`` on the GPU and return ``(bgr, gray)`` on the CPU.

        The grayscale is produced on the GPU by ``cv2.cuda.cvtColor``
        before download, saving one CPU pass through the rectified BGR.
        """
        gpu_in = cv2.cuda_GpuMat()
        gpu_in.upload(image, self._stream)
        gpu_rect = cv2.cuda.remap(
            gpu_in, self._gpu_map1, self._gpu_map2,
            interpolation=cv2.INTER_LINEAR, stream=self._stream,
        )
        gpu_gray = cv2.cuda.cvtColor(gpu_rect, cv2.COLOR_BGR2GRAY, stream=self._stream)
        bgr = gpu_rect.download()
        gray = gpu_gray.download()
        x, y, w, h = self._roi
        return bgr[y:y + h, x:x + w], gray[y:y + h, x:x + w]

    def get_intrinsics(self) -> RectifiedIntrinsics:
        """Intrinsics for the rectified (and ROI-cropped) image."""
        return RectifiedIntrinsics(
            fx=float(self._final_camera_matrix[0, 0]),
            fy=float(self._final_camera_matrix[1, 1]),
            cx=float(self._final_camera_matrix[0, 2]),
            cy=float(self._final_camera_matrix[1, 2]),
            img_width=int(self._new_width),
            img_height=int(self._new_height),
            distortion=np.zeros(5, dtype=np.float32),
        )


# =============================================================================
# Selection helper
# =============================================================================
def build_rectifier(
    logger,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    image_size: Tuple[int, int],
    is_fisheye: bool,
    crop_to_valid_pixels: bool,
    use_cuda: bool,
) -> Tuple[object, str]:
    """Build a rectifier using CUDA when ``use_cuda`` is true.

    Returns ``(rectifier, backend_name)`` where ``backend_name`` is one
    of ``"cuda"`` or ``"cpu"``. The detector logs this at startup so
    operators can confirm HW acceleration is engaged.
    """
    if use_cuda:
        try:
            r = CUDAImageRectifier(
                logger=logger,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                image_size=image_size,
                is_fisheye=is_fisheye,
                crop_to_valid_pixels=crop_to_valid_pixels,
            )
            return r, "cuda"
        except Exception as e:
            logger.warn(
                f"CUDA rectifier unavailable ({e!r}); falling back to CPU."
            )
    r = ImageRectifier(
        logger=logger,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=image_size,
        is_fisheye=is_fisheye,
        crop_to_valid_pixels=crop_to_valid_pixels,
    )
    return r, "cpu"