# Jetson Orin deployment notes

`race_auv_camera_pkg/apriltag_detector_node` is built to run in real
time on a Jetson Orin (Nano / AGX / Orin NX) with two camera streams.
This file is the operator's quick reference for getting the most out of
the HW paths.

## 1. Enable HW acceleration (env var)

Set in the launch environment (or via `ros2 run ... --env`):

```bash
export APRILTAG_USE_CUDA=1
```

This engages two backends inside the detector:

* **Rectification** via `cv2.cuda.remap` + `cv2.cuda.cvtColor` (when
  OpenCV is built with CUDA -- standard on NVIDIA L4T images).
* **JPEG encode** of the annotated `CompressedImage` published to
  Foxglove. Tries `pyNvJPEG` first (NVIDIA nvjpeg hardware), then
  `cv2.cuda.encodeJpeg`, then the OpenCV CPU encoder as the last
  resort. The selected backend is logged at startup under
  `=== HW acceleration ===`.

When the env var is unset (default) the node is fully CPU; safe for dev
machines and CI.

## 2. Pin the detector to perf cores

`apriltag.yaml` per-camera entry:

```yaml
process_scale: 0.5         # detect on 800x600; annotate stays 1600x1200
jpeg_quality: 75           # Foxglove / topside bandwidth knob
thread_priority: 10        # SCHED_FIFO; needs CAP_SYS_NICE
cpu_affinity: [4, 5, 6, 7] # Orin perf cores (4-7); LITTLE is 0-3
```

For `thread_priority` / `cpu_affinity` to take effect the launch user
needs `CAP_SYS_NICE` (root has it). Without it the node logs `DENIED
-- needs CAP_SYS_NICE` and falls back to normal scheduling.

To grant a non-root user `CAP_SYS_NICE`:

```bash
sudo setcap cap_sys_nice+ep $(which python3)
```

## 3. Jetson power / clocks

Before launching:

```bash
sudo nvpmodel -m 0                  # MAXN (or -m 2 for 25W)
sudo jetson_clocks                   # lock CPU/GPU to max frequencies
```

Verify with `jtop` (recommended) or `tegrastats`.

## 4. Recommended packages

```bash
sudo apt install nvidia-jetpack      # provides cv2 with CUDA + pyNvJPEG-compatible libs
pip install pyNvJPEG                 # hardware JPEG encoder (preferred)
```

If `pyNvJPEG` is missing, the detector still gets GPU acceleration
through `cv2.cuda.encodeJpeg` if the OpenCV build was compiled with
`WITH_NVCOMPRESS=ON` (uncommon on L4T, but possible). Otherwise the
CPU fallback engages.

## 5. Expected per-stage timings (1600x1200 @ 5 Hz)

| Stage                   | CPU only    | `APRILTAG_USE_CUDA=1` |
|-------------------------|-------------|----------------------|
| JPEG decode (compressed)|  5-10 ms    |  5-10 ms             |
| Rectify + BGR->gray     |  8-15 ms    |  2-4 ms              |
| Detect (process_scale=1)| 15-30 ms    | 15-30 ms             |
| Detect (process_scale=0.5) |  5-10 ms |  5-10 ms             |
| Annotate                |  3-5 ms     |  3-5 ms              |
| JPEG encode (annotated) |  8-15 ms    |  1-2 ms (nvjpeg)     |
| **Total (process_scale=0.5, CUDA)** | ~35-50 ms | **~12-22 ms** |

`process_scale=0.5` is the single biggest knob: it cuts detection
time by ~3-5x on the CPU cores. Combined with `APRILTAG_USE_CUDA=1`
and `pyNvJPEG`, the per-frame budget fits comfortably inside 200 ms
at 5 Hz with headroom for two cameras in parallel.

## 6. Confirming HW paths at runtime

Every detector logs a banner like this at startup (look for `HW
acceleration`):

```
[INFO] [<timestamp>] [apriltag_detector_cam_front]: === HW acceleration ===
  rectify backend : cuda
  jpeg   backend  : nvjpeg
  process_scale   : 0.5
  jpeg_quality    : 75
  APRILTAG_USE_CUDA env: on
```

If you see `cpu` for either backend, the env var was not picked up or
the optional libraries are not installed.

## 7. Topside / Foxglove

`apriltag_detection/image` is already published as
`sensor_msgs/CompressedImage` (the standard "image/compressed"
transport). `foxglove_bridge` relays it byte-for-byte -- no extra
config on the topside side. Set the Foxglove panel's "Compression
quality" to match `jpeg_quality` for best results.