"""Launch the camera drivers and per-camera AprilTag detectors.

Reads ``config/apriltag.yaml`` (in race_auv_bringup) and starts, per enabled
camera entry:

* One ``dwe_camera_driver/camera_node`` (driver block).
* One ``race_auv_camera_pkg/apriltag_detector_node`` (everything else
  under each cameras: entry).

This launch file is self-contained: it includes no other launch files. The
shared hardware/auxiliary configs for the camera driver still live next to
the YAML at ``race_auv_bringup/config/camera/``.

No TF, no fuser -- the detectors only publish annotated images and
``Detection3DArray`` topics.

Sequential spawn order (Jetson / multi-USB-camera safety)
---------------------------------------------------------
The four processes are started one at a time, in pairs, with a fixed
delay between each pair:

    t = 0 s    camera_node_1         (driver for the alphabetically-
                                        first ``driver.node_name``;
                                        maps to ``/dev/video0`` on the
                                        stock Stellar setup)
    t = 2 s    camera_node_2         (driver for the second camera)
    t = 4 s    detector_1            (matches camera_node_1)
    t = 6 s    detector_2            (matches camera_node_2)

Two reasons for the staggered start:

1. **UVC-probe race.** ``dwe_camera_driver`` uses ``v4l2-ctl`` to
   locate the camera (returns ``/dev/videoN``) and then opens it via
   ``cv2.VideoCapture(N)``. The kernel can list the device node
   before the UVC driver has finished probing the capture endpoint,
   so the first ``cv2.VideoCapture(N)`` for the higher-indexed
   device can fail with ``ENODEV`` / ``EBUSY``. Asking for
   ``/dev/video0`` first (alphabetically by ``driver.node_name``)
   and letting the kernel settle for ~2 s before asking for
   ``/dev/video2`` is a deterministic workaround. See ``Jetson.md``
   §7.1.

2. **Detector not racing its camera.** The detector subscribes to the
   compressed image topic, so starting it after its camera driver has
   had time to publish a few frames keeps the detector's
   ``_info_cb`` / ``_try_build_from_yaml_intrinsics`` path from
   racing the driver's CameraInfo (if any) or yaml-intrinsics setup.

The delay is ``_INTER_NODE_DELAY_S``; tune in one place if you need
more or less on a different host.
"""

import json
import os
from typing import Any, Dict, List, NamedTuple

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Delay between consecutive spawns (camera1, camera2, detector1, detector2).
# 2 s is enough on Jetson Orin to clear the UVC probe race for /dev/video2.
_INTER_NODE_DELAY_S = 2.0


class _CamPair(NamedTuple):
    """A YAML camera entry paired with its driver + detector Node actions."""

    cam_name: str             # YAML name (front/down/...) for diagnostics
    driver_node_name: str     # used for ordering; matches kernel /dev/videoN
    driver: Node
    detector: Node


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data.get("apriltag", data) or {}


def _build_pairs(config_path: str) -> List[_CamPair]:
    """Parse the YAML and return one ``_CamPair`` per enabled camera."""
    cfg = _load_yaml(config_path)
    detector_defaults = cfg.get("detector_defaults", {}) or {}

    bringup_share = get_package_share_directory("race_auv_bringup")
    hardware_config = os.path.join(bringup_share, "config", "camera", "hardware_controls.yaml")
    aux_process_config = os.path.join(bringup_share, "config", "camera", "auxiliary_processors.yaml")
    stellar_config = os.path.join(bringup_share, "config", "camera", "camera_parameters", "stellar_cam.yaml")

    pairs: List[_CamPair] = []

    for cam in cfg.get("cameras", []) or []:
        if not cam.get("enabled", True):
            continue

        cam_name = str(cam.get("name", "")).strip()
        if not cam_name:
            continue

        # --- driver block ---
        driver_cfg = cam.get("driver", {}) or {}
        product_name = str(driver_cfg.get("product_name", "")).strip()
        if not product_name:
            raise RuntimeError(
                f"camera '{cam_name}' in {config_path} is missing "
                "'driver.product_name'"
            )

        driver_node_name = str(driver_cfg.get("node_name", f"camera_node_{cam_name}"))
        driver_ns = str(driver_cfg.get("namespace", "") or "").strip("/")
        remap_image = str(driver_cfg.get("remap_image_compressed", "") or "").strip()
        remap_image_lowbw = str(driver_cfg.get("remap_image_lowbw_compressed", "") or "").strip()
        remap_settings = str(driver_cfg.get("remap_camera_settings", "") or "").strip()

        driver_remaps = []
        if remap_image:
            driver_remaps.append(("image/compressed", remap_image))
        if remap_image_lowbw:
            driver_remaps.append(("image_lowbw/compressed", remap_image_lowbw))
        if remap_settings:
            driver_remaps.append(("camera_settings", remap_settings))

        driver_node = Node(
            package="dwe_camera_driver",
            executable="camera_node",
            name=driver_node_name,
            namespace=driver_ns,
            output="screen",
            parameters=[
                hardware_config,
                aux_process_config,
                stellar_config,
                {"video.product_name": product_name},
            ],
            remappings=driver_remaps,
        )

        # --- apriltag detector block ---
        det_ns = str(cam.get("detector_namespace", cam.get("namespace", "")) or "").strip("/")
        image_topic = cam.get("image_topic")
        if not image_topic:
            raise RuntimeError(
                f"camera '{cam_name}' in {config_path} is missing 'image_topic'"
            )
        image_transport = str(cam.get("image_transport", "raw")).lower()
        info_topic = cam.get("info_topic", "") or ""
        camera_frame = cam.get("camera_frame", "") or ""
        output_image_topic = cam.get("output_image_topic", "apriltag_detection/image")
        output_detections_topic = cam.get(
            "output_detections_topic", "apriltag_detection/detections3d"
        )
        publish_rate = float(
            cam.get("publish_rate", detector_defaults.get("publish_rate", 5.0))
        )
        intrinsics = cam.get("intrinsics", {}) or {}

        tags_override = cam.get("tags", None)
        if tags_override is None:
            tags_override = cfg.get("tags", []) or []
        tags_override_json = json.dumps(tags_override)

        # Jetson / Foxglove perf knobs (per-camera). HW-accel selection
        # lives in detector_defaults (use_cuda, jpeg_backend); these
        # per-camera knobs are pure image-pipeline tunings.
        process_scale = float(cam.get("process_scale", 1.0))
        jpeg_quality = int(cam.get("jpeg_quality", 80))

        detector_node = Node(
            package="race_auv_camera_pkg",
            executable="apriltag_detector_node",
            name=f"apriltag_detector_{cam_name}",
            namespace=det_ns,
            output="screen",
            parameters=[{
                "config_yaml": config_path,
                "image_transport": image_transport,
                "image_topic": image_topic,
                "info_topic": info_topic,
                "camera_frame": camera_frame,
                "output_image_topic": output_image_topic,
                "output_detections_topic": output_detections_topic,
                "publish_rate": publish_rate,
                "tags_override": tags_override_json,
                "process_scale": process_scale,
                "jpeg_quality": jpeg_quality,
                "intrinsics.fx": float(intrinsics.get("fx", 0.0)),
                "intrinsics.fy": float(intrinsics.get("fy", 0.0)),
                "intrinsics.cx": float(intrinsics.get("cx", 0.0)),
                "intrinsics.cy": float(intrinsics.get("cy", 0.0)),
                "intrinsics.width": int(intrinsics.get("width", 0)),
                "intrinsics.height": int(intrinsics.get("height", 0)),
                "intrinsics.distortion": list(
                    intrinsics.get("distortion", [0.0] * 5)
                ),
                "intrinsics.fisheye": bool(intrinsics.get("fisheye", False)),
            }],
        )

        pairs.append(_CamPair(
            cam_name=cam_name,
            driver_node_name=driver_node_name,
            driver=driver_node,
            detector=detector_node,
        ))

    # Sort by driver.node_name so the kernel-lowest /dev/videoN is
    # spawned first. With the stock Stellar YAML
    # (``stellar_camera_node_1`` -> ``usb-2.1`` -> ``/dev/video0``,
    # ``stellar_camera_node_2`` -> ``usb-2.3`` -> ``/dev/video2``),
    # alphabetical order matches kernel-enumeration order.
    pairs.sort(key=lambda p: p.driver_node_name)
    return pairs


def _build_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    pairs = _build_pairs(config_path)
    if len(pairs) > 2:
        # The fixed 4-step sequence below is hard-coded for the
        # 2-camera case. For >2 cameras we'd need a list-driven
        # TimerAction chain. The Stellar rig is always 2 cameras.
        raise RuntimeError(
            f"camera_apriltag.launch.py supports at most 2 cameras "
            f"(found {len(pairs)}). Refactor _build_nodes for >2."
        )

    # Build the 4-step sequence:
    #   t=0         camera_node_1
    #   t=DELAY     camera_node_2
    #   t=2*DELAY   detector_1   (pairs with camera_node_1)
    #   t=3*DELAY   detector_2   (pairs with camera_node_2)
    # If only one camera is configured, drop the missing steps.
    actions = []

    def _add(node: Node, delay_s: float) -> None:
        if delay_s <= 0.0:
            actions.append(node)
        else:
            actions.append(TimerAction(period=delay_s, actions=[node]))

    _add(pairs[0].driver, 0.0)
    if len(pairs) >= 2:
        _add(pairs[1].driver, _INTER_NODE_DELAY_S)
    _add(pairs[0].detector, _INTER_NODE_DELAY_S * 2)
    if len(pairs) >= 2:
        _add(pairs[1].detector, _INTER_NODE_DELAY_S * 3)

    return actions


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("race_auv_bringup")
    default_config = os.path.join(bringup_share, "config", "apriltag.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description=(
            "Path to apriltag.yaml. Each cameras: entry describes a "
            "dwe_camera_driver (driver: block) and a matching "
            "apriltag_detector_node (everything else)."
        ),
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=_build_nodes),
    ])