# AprilTag Pipeline Refactor Plan

**Package:** `race_auv/race_auv_sim_pkg`  
**Goal:** Split the package into a *single-camera detector node per camera* and a *multi-camera TF-fuser node*, driven entirely from `config/apriltag.yaml`, with one zero-argument launch file that works for both simulation and real DWE hardware.

---

## 1. Current state

| File | Responsibility | Problem |
|------|----------------|---------|
| `stonefish_apriltag_node.py` | Per-camera detection **+** per-tag TF **+** per-camera `object_base` TF | Mixes detector and base-pose publisher; Stonefish-specific defaults; expects raw `Image` + `CameraInfo` only |
| `apriltag_fuser_node.py` | Subscribes to many `Detection3DArray` topics, publishes one fused `object_base` | Already well-scoped, but is launched separately with a long argument list |
| `apriltag_stonefish.launch.py` | Launches one detector with many launch args | Needs per-camera overrides from `bringup_simulation.launch.py` |
| `apriltag_fuser.launch.py` | Launches fuser with many launch args | Same issue |
| `config/apriltag.yaml` | Tag list, URDF, detector defaults | Only models one camera; no hardware camera list |

The result is that `bringup_simulation.launch.py` has to know every camera topic and pass them as launch arguments.

---

## 2. Target architecture

```
apriltag.yaml  (single source of truth)
      │
      ├─ cameras[]  ─────►  apriltag_detector_node (one per camera)
      │                        sub: image (+ optional camera_info)
      │                        pub: <ns>/apriltag_detection/detections3d
      │                        pub: <ns>/apriltag_detection/image
      │                        tf : <prefix><tag_id>  (optional, no object_base)
      │
      └─ fuser  ─────────►  apriltag_fuser_node (exactly one)
                               sub: all detector detection topics
                               tf : <reference_frame> → <output_frame>
```

### 2.1 Design principles

1. **One detector per camera.** Each camera gets its own node. Detectors only publish per-tag poses (and an annotated debug image); they never publish `object_base`.
2. **One fuser for all cameras.** The fuser owns the single fused `object_base` TF. It works unchanged for one camera or many.
3. **YAML is the single source of truth.** All topics, intrinsics, frame IDs, namespaces, URDF info, detector settings, tag list, and fuser settings live in `apriltag.yaml`.
4. **Zero-argument launch file.** `apriltag.launch.py` reads `apriltag.yaml` and starts every detector + the fuser. `bringup_simulation.launch.py` only includes this launch file.
5. **Simulation and real hardware are data-source differences only.** The detector node accepts raw `sensor_msgs/Image`, `sensor_msgs/CompressedImage`, and optionally `sensor_msgs/CameraInfo`. Camera intrinsics can come from `CameraInfo` or from the YAML.

---

## 3. Proposed `config/apriltag.yaml`

```yaml
apriltag:
  # URDF that describes the tag layout in the object's base frame.
  object:
    urdf_package:  "race_station_description"
    urdf_filename: "urdf/base.urdf"
    base_link_name: ""          # empty -> URDF root link
    tag_link_prefix: "apriltag"

  # Shared detector tuning.
  detector_defaults:
    nthreads:          2
    quad_decimate:     1.0
    quad_sigma:        0.0
    refine_edges:      true
    decode_sharpening: 0.25
    publish_rate:      5.0

  # Tag family / id / physical size.
  tags:
    - { id: 0, family: "tag25h9", size: 0.21 }
    - { id: 1, family: "tag25h9", size: 0.21 }
    - { id: 2, family: "tag25h9", size: 0.21 }

  # One entry per physical camera. Add/remove/enable cameras here.
  cameras:
    - name: "cam1"
      enabled: true
      namespace: "cam1"
      # Topic type: "raw" -> sensor_msgs/Image
      #             "compressed" -> sensor_msgs/CompressedImage
      image_transport: "raw"
      image_topic: "/race_auv/camera1/stonefish/data/image_color"
      info_topic:  "/race_auv/camera1/stonefish/data/camera_info"
      # Optional: override the camera optical frame. Empty -> use CameraInfo.header.frame_id.
      camera_frame: ""
      # Per-camera TF prefix so multiple cameras do not fight over "apriltag0".
      tag_frame_prefix: "apriltag"
      output_image_topic: "apriltag_detection/image"
      output_detections_topic: "apriltag_detection/detections3d"
      # Intrinsics override (only used when info_topic is empty).
      # If both are present, CameraInfo wins.
      intrinsics:
        fx: 0.0
        fy: 0.0
        cx: 0.0
        cy: 0.0
        width: 0
        height: 0
        distortion: [0.0, 0.0, 0.0, 0.0, 0.0]
        fisheye: false

    - name: "cam2"
      enabled: true
      namespace: "cam2"
      image_transport: "raw"
      image_topic: "/race_auv/camera2/stonefish/data/image_color"
      info_topic:  "/race_auv/camera2/stonefish/data/camera_info"
      camera_frame: ""
      tag_frame_prefix: "apriltag"
      output_image_topic: "apriltag_detection/image"
      output_detections_topic: "apriltag_detection/detections3d"
      intrinsics:
        fx: 0.0
        fy: 0.0
        cx: 0.0
        cy: 0.0
        width: 0
        height: 0
        distortion: [0.0, 0.0, 0.0, 0.0, 0.0]
        fisheye: false

  # Multi-camera fuser settings.
  fuser:
    enabled: true
    reference_frame: "base_link"   # frame in which object_base is published
    output_frame:    "object_base" # the fused object pose
    publish_rate:    5.0
    min_pairs:       3
    detection_max_age: 0.5
    tf_timeout:      0.1
```

### 3.1 How the launch file uses this file

1. Load the YAML once in the launch file.
2. For every `cameras[]` entry where `enabled == true`, generate an `apriltag_detector_node` with parameters taken from that block (merged with `detector_defaults`).
3. If `fuser.enabled == true`, generate an `apriltag_fuser_node`. Its `detections_topics` list is auto-built from each enabled camera's `output_detections_topic` (respecting `namespace`).

---

## 4. Node changes

### 4.1 New / renamed detector: `apriltag_detector_node`

**Source file:** `race_auv_sim_pkg/apriltag_detector_node.py`  
**Entry point:** `apriltag_detector_node`

Responsibilities:

- Subscribe to one image topic per camera.
  - `sensor_msgs/Image` when `image_transport == "raw"`.
  - `sensor_msgs/CompressedImage` when `image_transport == "compressed"` (decode with `cv_bridge`).
- Optionally subscribe to `sensor_msgs/CameraInfo`.
  - If `info_topic` is non-empty and messages arrive, use `K`, `D`, width, height, and frame ID.
  - If no `CameraInfo` is configured, use `intrinsics` from the YAML.
- Run the same `AprilTagDetector` logic currently in `stonefish_apriltag_node.py`.
- Publish:
  - `vision_msgs/Detection3DArray` on `output_detections_topic`.
  - Annotated debug image on `output_image_topic`.
- Publish per-tag TF frames using `tag_frame_prefix` + tag id.
- **Do NOT publish `object_base`.**

Parameter interface (populated by the launch file from YAML):

```yaml
config_yaml:        ""  # kept for URDF loading
namespace:          ""
image_transport:    "raw"
image_topic:        ""
info_topic:         ""
camera_frame:       ""
tag_frame_prefix:   "apriltag"
output_image_topic: "apriltag_detection/image"
output_detections_topic: "apriltag_detection/detections3d"
publish_rate:       5.0
# intrinsics block (used if CameraInfo is absent)
# detector_defaults merged from YAML
```

Notes:

- Re-use `apriltag_geom.py` and `urdf_tag_parser.py` unchanged.
- Move the camera-info-to-detector-build logic into a helper so raw vs. compressed is the only input difference.

### 4.2 Fuser: `apriltag_fuser_node`

**Changes from today:**

- No per-camera `object_base` fallback: detectors never publish it, so the fuser is the only publisher.
- `detections_topics` is read from YAML instead of a launch argument. The launch file constructs it automatically.
- Keep all existing math (Umeyama solve + single-tag fallback).
- Parameter interface shrinks to the `fuser:` block plus `config_yaml` for URDF loading.

---

## 5. Launch file changes

### 5.1 New launch file: `launch/apriltag.launch.py`

This is the only launch file the package needs.

```python
def generate_launch_description():
    pkg_share = get_package_share_directory('race_auv_sim_pkg')
    default_config = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to apriltag.yaml.',
    )

    # The OpaqueFunction reads the YAML and returns per-camera Nodes + the fuser Node.
    def build_nodes(context, *args, **kwargs):
        config_path = LaunchConfiguration('config').perform(context)
        cfg = load_yaml(config_path)
        apriltag_cfg = cfg.get('apriltag', cfg)

        actions = []
        detection_topics = []

        for cam in apriltag_cfg.get('cameras', []):
            if not cam.get('enabled', True):
                continue
            ns = cam.get('namespace', '')
            det_topic = _join(ns, cam.get('output_detections_topic'))
            detection_topics.append(det_topic)
            actions.append(Node(
                package='race_auv_sim_pkg',
                executable='apriltag_detector_node',
                name=f"apriltag_detector_{cam['name']}",
                namespace=ns,
                output='screen',
                parameters=[{
                    'config_yaml': config_path,
                    'image_transport': cam.get('image_transport', 'raw'),
                    'image_topic': cam.get('image_topic'),
                    'info_topic': cam.get('info_topic', ''),
                    'camera_frame': cam.get('camera_frame', ''),
                    'tag_frame_prefix': cam.get('tag_frame_prefix', 'apriltag'),
                    'output_image_topic': cam.get('output_image_topic'),
                    'output_detections_topic': cam.get('output_detections_topic'),
                    'publish_rate': cam.get('publish_rate',
                                            apriltag_cfg.get('detector_defaults', {}).get('publish_rate', 5.0)),
                    'intrinsics': cam.get('intrinsics', {}),
                }],
            ))

        fuser_cfg = apriltag_cfg.get('fuser', {})
        if fuser_cfg.get('enabled', True):
            actions.append(Node(
                package='race_auv_sim_pkg',
                executable='apriltag_fuser_node',
                name='apriltag_fuser',
                output='screen',
                parameters=[{
                    'config_yaml': config_path,
                    'reference_frame': fuser_cfg.get('reference_frame', 'base_link'),
                    'output_frame': fuser_cfg.get('output_frame', 'object_base'),
                    'publish_rate': fuser_cfg.get('publish_rate', 5.0),
                    'min_pairs': fuser_cfg.get('min_pairs', 3),
                    'detection_max_age': fuser_cfg.get('detection_max_age', 0.5),
                    'tf_timeout': fuser_cfg.get('tf_timeout', 0.1),
                    'detections_topics': detection_topics,
                }],
            ))

        return actions

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=build_nodes),
    ])
```

### 5.2 `bringup_simulation.launch.py`

Replace the current three include blocks (`sim_tagdet_cam1`, `sim_tagdet_cam2`, `sim_tagdet_fuser`) with a single include:

```python
sim_tagdet = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory(sim_tagdet_bringup),
                     'launch', 'apriltag.launch.py')
    ]),
    # No launch_arguments needed; everything is in apriltag.yaml.
)
```

Also remove the now-unused `arg_enable_fuser` DeclareLaunchArgument.

### 5.3 Legacy launch files

`apriltag_stonefish.launch.py` and `apriltag_fuser.launch.py` can be deleted once the new pipeline is verified. If a deprecation period is desired, keep them for one cycle and log a warning.

---

## 6. Simulation vs. real hardware

### 6.1 Simulation (Stonefish)

Already produces raw `Image` + `CameraInfo`. Config example:

```yaml
image_transport: "raw"
image_topic: "/race_auv/camera1/stonefish/data/image_color"
info_topic:  "/race_auv/camera1/stonefish/data/camera_info"
```

### 6.2 Real hardware (`dwe_camera_driver`)

The DWE driver publishes `sensor_msgs/CompressedImage` on `<ns>/image/compressed`. It does **not** publish `CameraInfo` by default, and the existing raw-image publisher does not fill the camera matrix.

Two supported approaches:

1. **Use the DWE driver as the image source and supply intrinsics in YAML.**

   ```yaml
   image_transport: "compressed"
   image_topic: "/dwe_camera/cam1/image/compressed"
   info_topic: ""      # no CameraInfo
   camera_frame: "dwe_camera_frame"
   intrinsics:
     fx: 1234.0
     fy: 1234.0
     cx: 960.0
     cy: 540.0
     width: 1920
     height: 1080
     distortion: [0.0, 0.0, 0.0, 0.0, 0.0]
     fisheye: false
   ```

2. **Add a small `camera_info` publisher to `dwe_camera_driver`.**
   A minimal node (or extension to `camera_node` / `remote_node`) could publish `CameraInfo` from the driver's own `camera.intrinsics.*` parameters. The detector would then subscribe to that topic and ignore the YAML intrinsics.

Recommended path: support both. YAML intrinsics are the fallback when `info_topic` is empty or no messages arrive. This avoids changing the driver unless desired.

### 6.3 Topic remapping cheat sheet

| Source | `image_transport` | `image_topic` example | `info_topic` example |
|--------|-------------------|-----------------------|----------------------|
| Stonefish camera 1 | `raw` | `/race_auv/camera1/stonefish/data/image_color` | `/race_auv/camera1/stonefish/data/camera_info` |
| DWE camera 1 | `compressed` | `/dwe_camera/cam1/image/compressed` | `''` or `/dwe_camera/cam1/camera_info` |

---

## 7. `setup.py` / package.xml updates

`setup.py`:

```python
entry_points={
    'console_scripts': [
        'apriltag_detector_node = race_auv_sim_pkg.apriltag_detector_node:main',
        'apriltag_fuser_node      = race_auv_sim_pkg.apriltag_fuser_node:main',
        # Legacy entry points; remove after transition
        'stonefish_apriltag_node  = race_auv_sim_pkg.stonefish_apriltag_node:main',
    ],
}
```

`package.xml`:

- Add `compressed_image_transport` or `image_transport` as an exec dependency if the node uses image-transport helpers (optional; the node can subscribe directly to `CompressedImage`).
- No new hard dependencies if direct `CompressedImage` subscription is used.

---

## 8. Implementation checklist

- [ ] Create `apriltag_detector_node.py` from `stonefish_apriltag_node.py` with:
  - [ ] raw and compressed image subscription
  - [ ] optional `CameraInfo` subscription / YAML intrinsics fallback
  - [ ] removal of all `object_base` publishing logic
  - [ ] namespace-aware per-tag TF prefix
- [ ] Refactor `apriltag_fuser_node.py`:
  - [ ] read `detections_topics` from parameter (list) as it does today
  - [ ] keep existing solver, remove references to per-camera `object_base`
- [ ] Create `launch/apriltag.launch.py` that builds nodes from YAML
- [ ] Update `config/apriltag.yaml` to the new schema with two Stonefish cameras and the fuser block
- [ ] Update `bringup_simulation.launch.py` to include `apriltag.launch.py` with no arguments
- [ ] Update `setup.py` entry points
- [ ] Delete or deprecate `apriltag_stonefish.launch.py` and `apriltag_fuser.launch.py`
- [ ] Test:
  - [ ] Single-camera simulation
  - [ ] Two-camera simulation
  - [ ] Single DWE hardware camera with compressed image + YAML intrinsics
  - [ ] Two DWE hardware cameras
- [ ] Update package description/docstrings to reflect generic detector (not Stonefish-only)

---

## 9. Migration notes for existing users

- Any custom launch files that included `apriltag_stonefish.launch.py` with per-camera arguments should switch to editing `config/apriltag.yaml`.
- The old `stonefish_apriltag_node` console-script can remain temporarily; it is not launched by the new launch file.
- RViz users: per-tag TF frames will now be prefixed by camera namespace or `tag_frame_prefix`. If you previously expected `apriltag0`, update to `cam1/apriltag0` or the configured prefix.

---

## 10. Open questions / follow-up

1. Should the per-camera debug image be compressed by default to save bandwidth on real hardware?
2. Do we want a `camera_info` republisher inside `dwe_camera_driver` so that intrinsics live only in the driver's YAML and are not duplicated in `apriltag.yaml`?
3. Should the fuser publish a `vision_msgs/Detection3DArray` or `geometry_msgs/PoseWithCovarianceStamped` in addition to the TF?
