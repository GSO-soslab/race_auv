# Jetson Orin deployment notes

`race_auv_camera_pkg/apriltag_detector_node` runs in real time on a
Jetson Orin (Nano / AGX / Orin NX) with one or more camera streams.
This file is the operator's quick reference for the Jetson-specific
deployment pieces.

The detector pipeline runs entirely on the CPU. Earlier versions of
this package supported GPU-accelerated rectification (`cv2.cuda.remap`)
and JPEG encoding (`cv2.cuda.encodeJpeg` / NVIDIA `nvjpeg` via
`pyNvJPEG`); those paths were removed because the OpenCV build
bundled with JetPack in the version we target does not include
`NVCOMPRESS`, and `pyNvJPEG`'s wheel is not available for the Python
version we ship. See the top-of-file comment in `image_processing.py`
for the full rationale; the JPEG encode is now done inline with
`cv2.imencode` in `apriltag_detector_node._publish_annotated`.

---

## 0. Install `apriltag3`

The detector uses the upstream `AprilRobotics/apriltag` Python wrapper
(the C extension built via CMake). It is **not** in the Debian apt
repos and is **not** available via `pip install` from a wheel --
the upstream repo does not ship a `setup.py` / `pyproject.toml`, so
the canonical install is a CMake build + install:

```bash
sudo apt install -y cmake build-essential python3-dev python3-numpy

git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
sudo ldconfig
```

`cmake` defaults the install prefix to `/usr/local`, which puts
`libapriltag.so.3` in `/usr/local/lib` and the
`apriltag.cpython-*.so` Python wrapper in
`/usr/local/lib/python3.12/site-packages/`. `ldconfig` then makes the
shared library discoverable to any process on the system with no
`LD_LIBRARY_PATH` gymnastics.

After the install, symlink the shared library into the same directory
as the Python wrapper. Upstream's CMakeLists installs the wrapper one
directory deeper than `libapriltag.so.3` (it's at
`${prefix}/lib/python3.12/site-packages/`), so the loader's default
search path won't find the shared library when the wrapper is
imported. The symlink fixes that and survives across rebuilds:

```bash
PY_SITE=/usr/local/lib/python3.12/site-packages
sudo ln -sf ../../libapriltag.so.3 $PY_SITE/libapriltag.so.3
```

(If you can't `sudo`, install to your home instead with
`cmake -B build -DCMAKE_INSTALL_PREFIX=$HOME/.local` and adjust the
paths above. The symlink trick is identical.)

`ninja` works in place of the default Makefiles if installed
(`sudo apt install ninja-build`):

```bash
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
sudo ldconfig
```

The `cv2.cuda` / `pyNvJPEG` GPU paths remain off (see top of file);
only the detector library itself changed.

---

## 1. Power / clocks

```bash
sudo nvpmodel -m 0                  # MAXN (or -m 2 for 25 W)
sudo jetson_clocks                  # lock CPU/GPU to max frequencies
```

Verify with `jtop` (recommended) or `tegrastats`.

---

## 2. Expected per-stage timings (1600x1200 @ 5 Hz, CPU)

| Stage                            | process_scale=1.0 | process_scale=0.5 |
|----------------------------------|-------------------|-------------------|
| JPEG decode (compressed)         |  5-10 ms          |  5-10 ms          |
| Rectify + ROI crop               | 10-15 ms          | 10-15 ms          |
| Detect (full image)              | 15-30 ms          | --                |
| Detect (process_scale=0.5)       | --                |  5-10 ms          |
| Annotate (boxes + axes + labels) |  3-5 ms           |  3-5 ms           |
| JPEG encode (annotated)          |  8-15 ms          |  8-15 ms          |
| **Total (process_scale=1.0)**    | **~45-80 ms**     | --                |
| **Total (process_scale=0.5)**    | --                | **~35-55 ms**     |

`process_scale: 0.5` is the single biggest knob: it cuts detection
time by ~3-5x on the CPU cores with negligible accuracy loss for
dock-sized tags at typical working distances. Even at full resolution
the pipeline keeps up with 5 Hz on the Orin; at 0.5 it has ~2x headroom.

---

## 3. Topside / Foxglove

`apriltag_detection/image` is published as
`sensor_msgs/CompressedImage` (the standard `image/compressed`
transport). `foxglove_bridge` relays it byte-for-byte -- no extra
config on the topside side. Set the Foxglove panel's "Compression
quality" to match `jpeg_quality` for best results.

---

## 4. Known upstream issues (not in this package)

These come from `dwe_camera_driver` and the rclpy / v4l2 stack. They
are **not** bugs in `race_auv_camera_pkg`; they're listed here so you
don't waste time blaming the wrong layer.

### 4.1. `/dev/video2` "can't open camera by index"

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

| Node           | What it is        |
|----------------|-------------------|
| `/dev/video0`  | cam0 **capture**  |
| `/dev/video1`  | cam0 metadata     |
| `/dev/video2`  | cam1 **capture**  |
| `/dev/video3`  | cam1 metadata     |

`v4l2-ctl --list-devices` returns the capture node as soon as it
exists, but the UVC driver hasn't necessarily finished probing the
capture endpoint yet. `cv2.VideoCapture(N)` returns `ENODEV` /
`EBUSY` when called before that probe completes.

**Why `multi_camera.launch.py` works and `camera_apriltag.launch.py`
didn't:** launch order.

* `multi_camera.launch.py` lists `stellar_camera_node_1` first
  (maps to `usb-3610000.usb-2.1` -> `/dev/video0`). `/dev/video0`
  opens successfully because the UVC probe for the lower-indexed
  device finishes first. Then `stellar_camera_node_2` opens
  `/dev/video2` and the kernel has had time to finish its probe.
* `camera_apriltag.launch.py` originally iterated the YAML cameras
  list (`cam_front` first, which mapped to `usb-3610000.usb-2.3` ->
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

### 4.2. rclpy `exc_info` logging crash

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

### 4.3. `camera.power_line_frequency` not supported

```
[WARN] power_line_frequency: Failed to set control 'power_line_frequency' to 0
[WARN] Control 'power_line_frequency' not supported. Setting parameter
       'camera.power_line_frequency' to read-only.
```

The Stellar cam driver reports this as a warning and the parameter is
marked read-only -- harmless. The `hardware_controls.yaml` value of 0
(Disabled) was correct for desktop testing but is not supported on the
Stellar Air underwater variant. Set it to 2 (60 Hz disabled) or
remove the line in
`race_auv_bringup/config/camera/hardware_controls.yaml` if you want a
clean log.