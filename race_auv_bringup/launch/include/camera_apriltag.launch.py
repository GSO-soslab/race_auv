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
"""

import json
import os
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data.get("apriltag", data) or {}


def _resolve(config_path: str, base: str) -> str:
    if not config_path:
        return ""
    if os.path.isabs(config_path):
        return config_path
    return os.path.join(base, config_path)


def _build_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    cfg = _load_yaml(config_path)
    detector_defaults = cfg.get("detector_defaults", {}) or {}

    bringup_share = get_package_share_directory("race_auv_bringup")
    camera_share = get_package_share_directory("race_auv_bringup")

    hardware_config = os.path.join(camera_share, "config", "camera", "hardware_controls.yaml")
    aux_process_config = os.path.join(camera_share, "config", "camera", "auxiliary_processors.yaml")
    stellar_config = os.path.join(camera_share, "config", "camera", "camera_parameters", "stellar_cam.yaml")

    actions = []

    for cam in cfg.get("cameras", []) or []:
        if not cam.get("enabled", True):
            continue

        cam_name = str(cam.get("name", "")).strip()
        if not cam_name:
            continue

        # --- driver block ---
        driver = cam.get("driver", {}) or {}
        product_name = str(driver.get("product_name", "")).strip()
        if not product_name:
            raise RuntimeError(
                f"camera '{cam_name}' in {config_path} is missing "
                "'driver.product_name'"
            )

        driver_node_name = str(driver.get("node_name", f"camera_node_{cam_name}"))
        driver_ns = str(driver.get("namespace", "") or "").strip("/")
        remap_image = str(driver.get("remap_image_compressed", "") or "").strip()
        remap_image_lowbw = str(driver.get("remap_image_lowbw_compressed", "") or "").strip()
        remap_settings = str(driver.get("remap_camera_settings", "") or "").strip()

        driver_remaps = []
        if remap_image:
            driver_remaps.append(("image/compressed", remap_image))
        if remap_image_lowbw:
            driver_remaps.append(("image_lowbw/compressed", remap_image_lowbw))
        if remap_settings:
            driver_remaps.append(("camera_settings", remap_settings))

        actions.append(
            Node(
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

        # Jetson / Foxglove perf knobs (per-camera).
        process_scale = float(cam.get("process_scale", 1.0))
        jpeg_quality = int(cam.get("jpeg_quality", 80))
        thread_priority = int(cam.get("thread_priority", 0))
        cpu_affinity_list = [int(c) for c in cam.get("cpu_affinity", []) or []]
        cpu_affinity = ",".join(str(c) for c in cpu_affinity_list)

        actions.append(
            Node(
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
                    "thread_priority": thread_priority,
                    "cpu_affinity": cpu_affinity,
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
        )

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