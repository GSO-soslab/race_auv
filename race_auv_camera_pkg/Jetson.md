# Jetson Orin deployment notes

`race_auv_camera_pkg/apriltag_detector_node` is built to run in real
time on a Jetson Orin (Nano / AGX / Orin NX) with two camera streams.
This file is the operator's quick reference for getting the most out of
the HW paths.

## 1. Enable HW acceleration (YAML only -- no env vars)

All hardware-acceleration knobs live in `apriltag.yaml` under
`detector_defaults:`:

```yaml
detector_defaults:
  use_cuda: false          # master GPU switch (true to engage GPU paths)
  jpeg_backend: "auto"     # "auto" | "nvjpeg" | "cuda" | "cpu"
```

Resolution rules:

* `use_cuda: false` -> everything runs on the CPU regardless of
  `jpeg_backend`.
* `use_cuda: true` + `jpeg_backend: "auto"` -> the JPEG encoder tries
  `pyNvJPEG` first, then `cv2.cuda.encodeJpeg`, then CPU. The rectifier
  tries `cv2.cuda.remap` and falls back to CPU on failure.
* `use_cuda: true` + `jpeg_backend: "nvjpeg"` -> require pyNvJPEG; if
  the import fails the node logs a warning and falls back to CPU.
* `use_cuda: true` + `jpeg_backend: "cuda"` -> require
  `cv2.cuda.encodeJpeg`; falls back to CPU on failure.
* `jpeg_backend: "cpu"` -> force CPU regardless of `use_cuda`.

To run on a Jetson:

```yaml
detector_defaults:
  use_cuda: true
  jpeg_backend: "auto"     # or "nvjpeg" to require hardware encode
```

## 2. Recommended packages

```bash
sudo apt install nvidia-jetpack      # provides cv2 with CUDA
pip install pyNvJPEG                 # hardware JPEG encoder (preferred)
```

If `pyNvJPEG` is missing, the detector still gets GPU acceleration
through `cv2.cuda.encodeJpeg` if the OpenCV build was compiled with
`WITH_NVCOMPRESS=ON` (uncommon on L4T, but possible). Otherwise the
CPU fallback engages and the operator sees `jpeg backend : cpu` in the
banner.

## 3. Jetson power / clocks

Before launching:

```bash
sudo nvpmodel -m 0                  # MAXN (or -m 2 for 25W)
sudo jetson_clocks                   # lock CPU/GPU to max frequencies
```

Verify with `jtop` (recommended) or `tegrastats`.

## 4. Expected per-stage timings (1600x1200 @ 5 Hz)

| Stage                   | CPU only    | `use_cuda: true` + nvjpeg |
|-------------------------|-------------|--------------------------|
| JPEG decode (compressed)|  5-10 ms    |  5-10 ms                 |
| Rectify + BGR->gray     |  8-15 ms    |  2-4 ms                  |
| Detect (process_scale=1)| 15-30 ms    | 15-30 ms                 |
| Detect (process_scale=0.5) |  5-10 ms |  5-10 ms                 |
| Annotate                |  3-5 ms     |  3-5 ms                  |
| JPEG encode (annotated) |  8-15 ms    |  1-2 ms (nvjpeg)         |
| **Total (process_scale=0.5, full HW)** | ~35-50 ms | **~12-22 ms** |

`process_scale: 0.5` is the single biggest knob: it cuts detection
time by ~3-5x on the CPU cores. Combined with `use_cuda: true` and a
working `pyNvJPEG`, the per-frame budget fits comfortably inside 200
ms at 5 Hz with headroom for two cameras in parallel.

## 5. Confirming HW paths at runtime

Every detector logs a banner like this at startup (look for `HW
acceleration`):

```
[INFO] [...]: === HW acceleration ===
  use_cuda (yaml) : true
  rectify backend : cuda
  jpeg   backend  : nvjpeg (requested: auto)
  process_scale   : 0.5
  jpeg_quality    : 75
```

If you see `cpu` for either backend, either `use_cuda` is false or the
optional libraries (`pyNvJPEG`, CUDA-enabled OpenCV) are not installed.

## 6. Topside / Foxglove

`apriltag_detection/image` is already published as
`sensor_msgs/CompressedImage` (the standard "image/compressed"
transport). `foxglove_bridge` relays it byte-for-byte -- no extra
config on the topside side. Set the Foxglove panel's "Compression
quality" to match `jpeg_quality` for best results.