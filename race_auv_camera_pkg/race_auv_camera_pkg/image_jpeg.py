"""JPEG encoder selection for the annotated CompressedImage output.

The annotated frame is published on a ``sensor_msgs/CompressedImage``
topic and relayed to topside Foxglove via ``foxglove_bridge``. On a
1600x1200 frame at 5 Hz the encode cost is significant; on a Jetson
Orin the hardware JPEG encoder (NVIDIA nvjpeg) is ~3-5x faster than
the OpenCV CPU encoder.

Backend selection (in order):

1. **pyNvJPEG** -- NVIDIA's Python wrapper for the nvjpeg hardware
   encoder. Preferred when importable. ``pip install pyNvJPEG``.
2. **cv2.cuda.encodeJpeg** -- OpenCV CUDA backend, only present if the
   installed OpenCV was built with ``WITH_NVCOMPRESS=ON``.
3. **cv2.imencode** -- CPU fallback. Always available.

Both backends take a BGR numpy array and a quality (1-100) and return
JPEG bytes. The detector node wraps the encoded bytes in a
``sensor_msgs/CompressedImage`` directly -- we do NOT use
``cv_bridge.cv2_to_compressed_imgmsg`` for this so the JPEG params
travel unchanged through the HW path.

Activation is controlled by the environment variable
``APRILTAG_USE_CUDA=1``. When unset the encoder always resolves to the
CPU backend, so the same code works on dev machines and CI.
"""

from __future__ import annotations

import os
from typing import Optional, Tuple

import cv2
import numpy as np


_BGR2JPEG_QUALITY = int(cv2.IMWRITE_JPEG_QUALITY)


# =============================================================================
# Backend implementations
# =============================================================================
class _PyNvJpegBackend:
    """Hardware JPEG encoder via NVIDIA ``pyNvJPEG``.

    Falls back silently to ``None`` if the module is not importable.
    """

    def __init__(self, quality: int) -> None:
        import pyNvJPEG as nvjpeg  # type: ignore
        self._nvjpeg = nvjpeg
        self._encoder = nvjpeg.JpegEncoder(device_id=0)
        self._quality = int(quality)
        self._bgr_buf: Optional[np.ndarray] = None

    def encode(self, bgr: np.ndarray) -> bytes:
        h, w = bgr.shape[:2]
        if self._bgr_buf is None or self._bgr_buf.shape != bgr.shape:
            # nvjpeg wants a contiguous BGR uint8 array; reuse the buffer
            # across calls when the frame size is stable.
            self._bgr_buf = np.ascontiguousarray(bgr)
        else:
            np.copyto(self._bgr_buf, bgr)
        return bytes(self._encoder.encode(
            self._bgr_buf, quality=self._quality, format="bgr",
        ))


class _CudaEncodeBackend:
    """GPU JPEG encoder via ``cv2.cuda``.

    Available only when OpenCV was built with ``WITH_NVCOMPRESS=ON``.
    Falls back silently to ``None`` if the encoder is not present.
    """

    def __init__(self, quality: int) -> None:
        # Force the symbol to bind; raise AttributeError at init time if absent.
        cv2.cuda.encodeJpeg  # noqa: B018
        self._quality = int(quality)
        self._stream = cv2.cuda_Stream()

    def encode(self, bgr: np.ndarray) -> bytes:
        gpu = cv2.cuda_GpuMat()
        gpu.upload(bgr, self._stream)
        ok, buf = cv2.cuda.encodeJpeg(
            gpu, stream=self._stream,
            params=[_BGR2JPEG_QUALITY, self._quality],
        )
        if not ok:
            raise RuntimeError("cv2.cuda.encodeJpeg returned False")
        return bytes(buf.tobytes() if hasattr(buf, "tobytes") else buf)


class _CpuEncodeBackend:
    """CPU JPEG encoder via ``cv2.imencode``. Always available."""

    def __init__(self, quality: int) -> None:
        self._quality = int(quality)
        # Reuse the same params list across calls.
        self._params = [_BGR2JPEG_QUALITY, self._quality]

    def encode(self, bgr: np.ndarray) -> bytes:
        ok, buf = cv2.imencode(".jpg", bgr, self._params)
        if not ok:
            raise RuntimeError("cv2.imencode returned False")
        return bytes(buf.tobytes() if hasattr(buf, "tobytes") else buf)


# =============================================================================
# Selection helper
# =============================================================================
def build_jpeg_encoder(quality: int, prefer_cuda: bool) -> Tuple[object, str]:
    """Pick the best available JPEG encoder.

    Returns ``(encoder, backend_name)`` where ``backend_name`` is one
    of ``"nvjpeg"``, ``"cuda"``, or ``"cpu"``. The detector logs this
    at startup so operators can confirm the HW path is engaged.

    ``prefer_cuda`` is the environment-derived gate
    (``os.environ.get("APRILTAG_USE_CUDA") == "1"``). When false the
    function returns the CPU encoder immediately.
    """
    if prefer_cuda:
        try:
            return _PyNvJpegBackend(quality), "nvjpeg"
        except Exception:
            pass
        try:
            return _CudaEncodeBackend(quality), "cuda"
        except Exception:
            pass
    return _CpuEncodeBackend(quality), "cpu"


def cuda_acceleration_requested() -> bool:
    """True iff ``APRILTAG_USE_CUDA=1`` is set in the environment."""
    return os.environ.get("APRILTAG_USE_CUDA", "").strip().lower() in ("1", "true", "yes", "on")