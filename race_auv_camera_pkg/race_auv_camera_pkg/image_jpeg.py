"""JPEG encoder selection for the annotated CompressedImage output.

The annotated frame is published on a ``sensor_msgs/CompressedImage``
topic and relayed to topside Foxglove via ``foxglove_bridge``. On a
1600x1200 frame at 5 Hz the encode cost is significant; on a Jetson
Orin the hardware JPEG encoder (NVIDIA nvjpeg) is ~3-5x faster than
the OpenCV CPU encoder.

Backend selection is controlled entirely by the ``apriltag.yaml``
``detector_defaults:`` block (no environment variables):

    detector_defaults:
      use_cuda: false          # master switch
      jpeg_backend: "auto"     # "auto" | "nvjpeg" | "cuda" | "cpu"

Resolution:

* ``"auto"`` with ``use_cuda=true`` -> try ``pyNvJPEG`` first, then
  ``cv2.cuda.encodeJpeg``, then CPU. The first one that imports /
  initializes wins.
* ``"auto"`` with ``use_cuda=false`` -> CPU.
* ``"nvjpeg"`` -> require ``pyNvJPEG``. If not importable, fall back to
  CPU with a warning (avoids silently producing wrong output if the
  operator's expectation was that nvjpeg would be used).
* ``"cuda"`` -> require ``cv2.cuda.encodeJpeg``. Falls back to CPU on
  failure.
* ``"cpu"`` -> ``cv2.imencode``.

All backends take a BGR numpy array and a quality (1-100) and return
JPEG bytes. The detector node wraps the encoded bytes in a
``sensor_msgs/CompressedImage`` directly -- we do NOT use
``cv_bridge.cv2_to_compressed_imgmsg`` for this so the JPEG params
travel unchanged through the HW path.
"""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np


_BGR2JPEG_QUALITY = int(cv2.IMWRITE_JPEG_QUALITY)
_VALID_BACKENDS = ("auto", "nvjpeg", "cuda", "cpu")


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
        self._bgr_buf: np.ndarray | None = None

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
        self._params = [_BGR2JPEG_QUALITY, self._quality]

    def encode(self, bgr: np.ndarray) -> bytes:
        ok, buf = cv2.imencode(".jpg", bgr, self._params)
        if not ok:
            raise RuntimeError("cv2.imencode returned False")
        return bytes(buf.tobytes() if hasattr(buf, "tobytes") else buf)


# =============================================================================
# Selection helper
# =============================================================================
def _try_pynvjpeg(quality: int):
    """Try to construct a ``pyNvJPEG`` backend. Returns ``None`` on any failure."""
    try:
        return _PyNvJpegBackend(quality)
    except Exception:
        return None


def _try_cuda_encode(quality: int):
    """Try to construct a ``cv2.cuda.encodeJpeg`` backend. Returns ``None`` on any failure."""
    try:
        return _CudaEncodeBackend(quality)
    except Exception:
        return None


def _nvjpeg_hint() -> str:
    """Actionable hint when pyNvJPEG can't be imported."""
    return "hint: install NVIDIA's hardware JPEG encoder: `pip install pyNvJPEG`."


def _cuda_encode_hint() -> str:
    """Actionable hint when cv2.cuda.encodeJpeg is unavailable."""
    return (
        "hint: cv2.cuda.encodeJpeg missing -- OpenCV was not built with "
        "NVCOMPRESS. Set `jpeg_backend: \"cpu\"` (Jetson Orin Nano "
        "does not support nvjpeg; on Orin NX / AGX you can also "
        "`pip install pyNvJPEG` and use `jpeg_backend: \"nvjpeg\"`)."
    )


def build_jpeg_encoder(
    quality: int,
    use_cuda: bool,
    requested_backend: str = "auto",
    logger=None,
) -> Tuple[object, str, list]:
    """Pick the JPEG encoder per the YAML config.

    Parameters
    ----------
    quality
        JPEG quality 1-100.
    use_cuda
        Master GPU switch (``detector_defaults.use_cuda``). When false,
        the GPU paths are skipped regardless of ``requested_backend``.
    requested_backend
        ``"auto"`` | ``"nvjpeg"`` | ``"cuda"`` | ``"cpu"``. Invalid
        values are coerced to ``"auto"`` with a warning logged by the
        caller (this function logs nothing).
    logger
        Optional rclpy-compatible logger. When provided, hints are
        logged for any silent fallback so the operator sees *why* HW
        wasn't engaged.

    Returns
    -------
    (encoder, backend_name, hints)
        ``backend_name`` is one of ``"nvjpeg"``, ``"cuda"``, ``"cpu"``
        -- what was actually selected, after any fallbacks. ``hints``
        is a list of human-readable strings the caller can log
        alongside the HW banner.
    """
    requested_backend = requested_backend.lower()
    if requested_backend not in _VALID_BACKENDS:
        requested_backend = "auto"

    hints: list[str] = []

    def _log(msg: str) -> None:
        if logger is not None:
            logger.warn(msg)

    if requested_backend == "auto":
        if use_cuda:
            enc = _try_pynvjpeg(quality)
            if enc is not None:
                return enc, "nvjpeg", hints
            hints.append(_nvjpeg_hint())
            enc = _try_cuda_encode(quality)
            if enc is not None:
                return enc, "cuda", hints
            hints.append(_cuda_encode_hint())
        return _CpuEncodeBackend(quality), "cpu", hints

    if requested_backend == "cpu":
        return _CpuEncodeBackend(quality), "cpu", hints

    if requested_backend == "nvjpeg":
        enc = _try_pynvjpeg(quality)
        if enc is not None:
            return enc, "nvjpeg", hints
        hints.append(_nvjpeg_hint())
        # Explicitly requested -- fall back to CPU with the hint.
        for h in hints:
            _log(h)
        return _CpuEncodeBackend(quality), "cpu", hints

    # requested_backend == "cuda"
    if not use_cuda:
        hints.append(
            "hint: use_cuda is false, so jpeg_backend='cuda' was skipped."
        )
        return _CpuEncodeBackend(quality), "cpu", hints
    enc = _try_cuda_encode(quality)
    if enc is not None:
        return enc, "cuda", hints
    hints.append(_cuda_encode_hint())
    for h in hints:
        _log(h)
    return _CpuEncodeBackend(quality), "cpu", hints