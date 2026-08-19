# Jetson Orin deployment notes

`race_auv_camera_pkg/apriltag_detector_node` is built to run in real
time on a Jetson Orin (Nano / AGX / Orin NX) with two camera streams.
This file is the operator's quick reference.

---

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

---

## 2. First-time install on the Jetson (CUDA + nvjpeg)

**This is the step most people miss.** The detector logs
`CUDA rectifier unavailable ... cv2.cuda reports no CUDA-enabled devices`
and falls back to CPU when the wrong OpenCV is installed.

`pip`'s `opencv-python` / `opencv-contrib-python` wheels do **not**
include CUDA. The CUDA-enabled OpenCV ships with NVIDIA's JetPack as a
Debian package and lives under `/usr/lib/python3/dist-packages/cv2/`.

```bash
# 1. Remove any pip-installed opencv (no CUDA in the wheels)
pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless

# 2. Install NVIDIA's CUDA-enabled system opencv (JetPack)
sudo apt update
sudo apt install python3-opencv

# 3. Hardware JPEG encoder
pip install pyNvJPEG

# 4. Sanity check -- this is what the detector does internally.
python3 - <<'PY'
import cv2
print("cv2 path :", cv2.__file__)
print("cuda devs:", cv2.cuda.getCudaEnabledDeviceCount())
PY
# Expect:
#   cv2 path : /usr/lib/python3/dist-packages/cv2/__init__.py
#   cuda devs: 1
```

If `cuda devs: 0`, the system OpenCV did not get installed in front of
a pip wheel -- usually means a stray `~/.local/lib/python3.12/site-packages/cv2/`
exists. `pip uninstall` and reinstall, or move the user-site out of the
way for that shell.

---

## 3. Reading the detector's HW banner

Every detector logs a banner at startup (look for `HW acceleration`):

```
[INFO] [...]: === HW acceleration ===
  use_cuda (yaml) : True
  rectify backend : cuda
  jpeg   backend  : nvjpeg (requested: auto)
  process_scale   : 0.5
  jpeg_quality    : 75
```

If a backend silently fell back, you'll also see a hint (WARN level):

```
[WARN] [...]: HW fallback hint: hint: pip's opencv-python has no CUDA support.
   Uninstall it and install NVIDIA's JetPack system opencv: ...
[WARN] [...]: HW fallback hint: hint: install NVIDIA's hardware JPEG encoder:
   `pip install pyNvJPEG`.
```

What each line means:

| Line | Meaning |
|------|---------|
| `use_cuda (yaml) : True` | Read straight from `apriltag.yaml`. |
| `rectify backend : cuda` | Rectification is running on the GPU (`cv2.cuda.remap`). |
| `rectify backend : cpu` | Rectification is running on the CPU. With `use_cuda: True` this means the CUDA path failed -- check the WARN hint. |
| `jpeg backend : nvjpeg` | Annotated frames are encoded by `pyNvJPEG` (NVIDIA hardware). |
| `jpeg backend : cuda` | Encoded by `cv2.cuda.encodeJpeg` (rare; needs OpenCV built with NVCOMPRESS). |
| `jpeg backend : cpu` | Encoded by `cv2.imencode`. Fine for dev / low-rate; on Jetson you'll want HW encode. |
| `(requested: auto)` | The value of `jpeg_backend` in the YAML. |

---

## 4. Jetson power / clocks

```bash
sudo nvpmodel -m 0                  # MAXN (or -m 2 for 25W)
sudo jetson_clocks                   # lock CPU/GPU to max frequencies
```

Verify with `jtop` (recommended) or `tegrastats`.

---

## 5. Expected per-stage timings (1600x1200 @ 5 Hz)

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
time by ~3-5x on the CPU cores.

---

## 6. Topside / Foxglove

`apriltag_detection/image` is already published as
`sensor_msgs/CompressedImage` (the standard "image/compressed"
transport). `foxglove_bridge` relays it byte-for-byte -- no extra
config on the topside side. Set the Foxglove panel's "Compression
quality" to match `jpeg_quality` for best results.

---

## 7. Known upstream issues (not in this package)

These come from `dwe_camera_driver` and the rclpy/v4l2 stack. They are
**not** bugs in `race_auv_camera_pkg`; they're listed here so you
don't waste time blaming the wrong layer.

### 7.1. `/dev/video2` "can't open camera by index"

Symptom:

```
[camera_node-N] Found camera 'usb-3610000.usb-2.X' at /dev/video2
[camera_node-N] [ WARN:0@0.626] open VIDEOIO(V4L2:/dev/video2): can't open camera by index
[camera_node-N] RuntimeError: Failed to open video device /dev/video2
```

What it actually means: `dwe_camera_driver` uses `v4l2-ctl` to
locate the camera by product name (returns `/dev/videoN`) and then
asks OpenCV's `cv2.VideoCapture(N)` to open it. On Linux, USB UVC
cameras expose two `/dev/videoN` nodes per physical camera:

| Node                | What it is             |
|---------------------|------------------------|
| `/dev/video0`       | cam0 **capture**       |
| `/dev/video1`       | cam0 metadata          |
| `/dev/video2`       | cam1 **capture**       |
| `/dev/video3`       | cam1 metadata          |

`v4l2-ctl --list-devices` returns the capture node as soon as it
exists, but the UVC driver hasn't necessarily finished probing the
capture endpoint yet. `cv2.VideoCapture(N)` returns `ENODEV` /
`EBUSY` when called before that probe completes.

**Why `multi_camera.launch.py` works and `camera_apriltag.launch.py`
didn't:** launch order.

* `multi_camera.launch.py` lists `stellar_camera_node_1` first
  (maps to `usb-3610000.usb-2.1` → `/dev/video0`). `/dev/video0`
  opens successfully because the UVC probe for the lower-indexed
  device finishes first. Then `stellar_camera_node_2` opens
  `/dev/video2` and the kernel has had time to finish its probe.

* `camera_apriltag.launch.py` originally iterated the YAML cameras
  list (`cam_front` first, which mapped to `usb-3610000.usb-2.3` →
  `/dev/video2`) and interleaved each detector with its driver.
  Result: `/dev/video2` was opened first -- before the UVC probe
  finished -- and failed; `/dev/video0` opened second and won.

**The fix (in our launch file):** strictly sequential spawn with a
2-second delay between each of the four nodes:

```
t = 0 s   camera_node_1
t = 2 s   camera_node_2
t = 4 s   detector_1   (paired with camera_node_1)
t = 6 s   detector_2   (paired with camera_node_2)
```

Drivers are sorted by `driver.node_name` so the kernel-lowest
`/dev/videoN` is spawned first. The detector for camera 1 is spawned
*after* camera 2, so the camera has a full ~4 s head start before its
detector starts subscribing -- enough to clear any remaining UVC /
OpenCV race. See the launch file docstring for the full rationale.

If you can't wait for an upstream fix in `dwe_camera_driver`, the
underlying issue (no retry on `ENODEV`) is still there -- the
workaround just keeps the failure window off the critical path. The
real fix is in `dwe_camera_driver`:

1. Add a small `time.sleep(0.2)` or retry loop between the
   `v4l2-ctl` lookup and the `cv2.VideoCapture` open in
   `dwe_camera_driver/camera_node.py::setup_camera_device`.
2. Or pin the video device path explicitly and open it with
   `cv2.VideoCapture("/dev/videoN", cv2.CAP_V4L2)` instead of by
   index, which sidesteps the OpenCV enumeration race entirely.

### 7.2. rclpy `exc_info` logging crash

```
TypeError: parameter "exc_info" is not one of the recognized logging
options "['throttle_duration_sec', 'throttle_time_source_type',
'skip_first', 'once']"
```

`dwe_camera_driver/camera_node.py` calls
`self.get_logger().fatal(..., exc_info=True)`. rclpy in Jazzy rejects
`exc_info` on its logger. The crash is benign (the node has already
exited), but it pollutes the launch log. Fix is to remove
`exc_info=True` in `dwe_camera_driver`. Out of scope for this package.

### 7.3. `camera.power_line_frequency` not supported

```
[WARN] power_line_frequency: Failed to set control 'power_line_frequency' to 0
[WARN] Control 'power_line_frequency' not supported. Setting parameter
       'camera.power_line_frequency' to read-only.
```

The Stellar cam driver reports this as a warning and the parameter is
marked read-only -- harmless. The `hardware_controls.yaml` value of 0
(Disabled) was correct for desktop testing but is not supported on the
Stellar Air underwater variant. Set it to 2 (60 Hz disabled) or
remove the line in `race_auv_bringup/config/camera/hardware_controls.yaml`
if you want a clean log.