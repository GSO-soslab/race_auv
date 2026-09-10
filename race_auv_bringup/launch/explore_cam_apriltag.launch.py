"""Single-camera AprilTag test: DWE exploreHD + CUDA detector.

Everything is driven by one YAML file
(``race_auv_bringup/config/explore_cam_apriltag.yaml`` by default):

* ``camera:``      -> ``dwe_camera_driver/camera_node`` video settings,
                      intrinsics, V4L2 controls and auxiliary streams
* ``driver_config:`` -> base parameter files loaded before the overrides
* ``detector:``    -> ``race_auv_camera_pkg/apriltag_detector_node``
                      backend, tick rate, scale, topics and CUDA knobs
* ``tags: []``     -> empty means *all* tags: the detector falls back to
                      the global tag list in ``detector.config_yaml``

Usage::

    source install/setup.bash
    ros2 launch race_auv_bringup explore_cam_apriltag.launch.py
    # custom config file:
    ros2 launch race_auv_bringup explore_cam_apriltag.launch.py \
        config:=/path/to/explore_cam_apriltag.yaml

Topics (defaults; namespace from ``camera.namespace``):

    /explore/image/compressed                    raw camera JPEG
    /explore/apriltag_detection/detections3d     vision_msgs/Detection3DArray
    /explore/apriltag_detection/image            annotated JPEG

Watch detections::

    ros2 topic echo /explore/apriltag_detection/detections3d
    ros2 topic hz   /explore/apriltag_detection/detections3d
"""

import json
import os
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _resolve_path(path: str, *share_dirs: str) -> str:
    """Absolute path wins; otherwise try each package share dir."""
    if os.path.isabs(path):
        return path
    for share in share_dirs:
        candidate = os.path.join(share, path)
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(share_dirs[0], path)


def _prefix(namespace: str) -> str:
    ns = str(namespace or "").strip("/")
    return f"/{ns}" if ns else ""


def _driver_parameters(
    cam: Dict[str, Any],
    driver_cfg: Dict[str, Any],
    driver_share: str,
) -> List[Any]:
    """Base YAML files + one override dict built from ``camera:``."""
    base_files = []
    for key in (
        "hardware_controls",
        "camera_parameters",
        "auxiliary_processors",
    ):
        rel = driver_cfg.get(key)
        if rel:
            base_files.append(_resolve_path(str(rel), driver_share))

    video = cam.get("video", {}) or {}
    intrinsics = cam.get("intrinsics", {}) or {}

    overrides: Dict[str, Any] = {
        "video.id": int(video.get("id", 0)),
        "video.product_name": str(video.get("product_name", "") or ""),
        "video.width": int(video.get("width", 1920)),
        "video.height": int(video.get("height", 1080)),
        "video.framerate": int(video.get("framerate", 15)),
        "video.format": str(video.get("format", "MJPG")),
        "ros.frame_id": str(cam.get("frame_id", "dwe_camera_frame")),
        "camera.intrinsics.fx": float(intrinsics.get("fx", 1000.0)),
        "camera.intrinsics.fy": float(intrinsics.get("fy", 1000.0)),
        "camera.intrinsics.cx": float(intrinsics.get("cx", 960.0)),
        "camera.intrinsics.cy": float(intrinsics.get("cy", 540.0)),
        "camera.distortion": [
            float(v) for v in intrinsics.get("distortion", [0.0] * 5)
        ],
        "camera.fisheye": bool(intrinsics.get("fisheye", False)),
    }

    for name, value in (cam.get("controls", {}) or {}).items():
        overrides[f"camera.{name}"] = value

    aux = cam.get("aux_process", {}) or {}
    for name in (
        "img_raw",
        "img_raw_framerate",
        "img_calibrated",
        "img_calibrated_framerate",
    ):
        if name in aux:
            overrides[f"aux_process.{name}"] = aux[name]

    compression = cam.get("compression", {}) or {}
    for name in ("downscale", "target_fps", "jpeg_quality"):
        if name in compression:
            overrides[f"compression.{name}"] = compression[name]

    return base_files + [overrides]


def _normalized_tags(raw_tags: Any) -> str:
    """JSON list for the detector's ``tags_override`` parameter.

    ``"[]"`` means "no override": the detector falls back to the global
    tag list in ``config_yaml`` (i.e. all configured tags).
    """
    if not raw_tags:
        return "[]"
    normalized = []
    for tag in raw_tags:
        normalized.append({
            "family": str(tag.get("family", "tag36h11")),
            "id": int(tag["id"]),
            "size": float(tag["size"]),
        })
    return json.dumps(normalized)


def _build_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    cfg = _load_yaml(config_path)
    cfg = cfg.get("explore_apriltag", cfg) or {}

    cam = cfg.get("camera", {}) or {}
    det = cfg.get("detector", {}) or {}
    cuda = det.get("cuda", {}) or {}
    driver_cfg = cfg.get("driver_config", {}) or {}

    driver_share = get_package_share_directory("dwe_camera_driver")
    bringup_share = get_package_share_directory("race_auv_bringup")

    namespace = str(cam.get("namespace", "explore")).strip("/")
    frame_id = str(cam.get("frame_id", "dwe_camera_frame"))
    prefix = _prefix(namespace)

    video = cam.get("video", {}) or {}
    intrinsics = cam.get("intrinsics", {}) or {}

    camera_node = Node(
        package="dwe_camera_driver",
        executable="camera_node",
        name="camera_node",
        namespace=namespace,
        output="screen",
        parameters=_driver_parameters(cam, driver_cfg, driver_share),
    )

    tags = cfg.get("tags", det.get("tags", []))
    config_yaml = _resolve_path(
        str(det.get("config_yaml", "config/apriltag.yaml")),
        bringup_share,
        driver_share,
    )
    image_topic = det.get("image_topic") or f"{prefix}/image/compressed"
    output_image_topic = (
        det.get("output_image_topic")
        or f"{prefix}/apriltag_detection/image"
    )
    output_detections_topic = (
        det.get("output_detections_topic")
        or f"{prefix}/apriltag_detection/detections3d"
    )

    detector_node = Node(
        package="race_auv_camera_pkg",
        executable="apriltag_detector_node",
        name="apriltag_detector",
        output="screen",
        parameters=[{
            "config_yaml": config_yaml,
            "image_transport": str(det.get("image_transport", "compressed")),
            "image_topic": image_topic,
            "info_topic": str(det.get("info_topic", "") or ""),
            "camera_frame": frame_id,
            "output_image_topic": output_image_topic,
            "output_detections_topic": output_detections_topic,
            "publish_rate": float(det.get("publish_rate", 10.0)),
            "process_scale": float(det.get("process_scale", 1.0)),
            "detector_backend": str(det.get("backend", "cuda")).lower(),
            "image_pipeline": str(det.get("image_pipeline", "cpu")).lower(),
            "gpu_rectify_interval": int(det.get("gpu_rectify_interval", 4)),
            "cuda_nominal_size": float(cuda.get("nominal_size", 0.125)),
            "cuda_tile_size": int(cuda.get("tile_size", 4)),
            "cuda_max_tags": int(cuda.get("max_tags", 64)),
            "min_edge_dist": int(det.get("min_edge_dist", 10)),
            "jpeg_quality": int(det.get("jpeg_quality", 90)),
            "tags_override": _normalized_tags(tags),
            "intrinsics.fx": float(intrinsics.get("fx", 0.0)),
            "intrinsics.fy": float(intrinsics.get("fy", 0.0)),
            "intrinsics.cx": float(intrinsics.get("cx", 0.0)),
            "intrinsics.cy": float(intrinsics.get("cy", 0.0)),
            "intrinsics.width": int(video.get("width", 0)),
            "intrinsics.height": int(video.get("height", 0)),
            "intrinsics.distortion": [
                float(v)
                for v in intrinsics.get("distortion", [0.0] * 5)
            ],
            "intrinsics.fisheye": bool(intrinsics.get("fisheye", False)),
        }],
    )

    return [camera_node, detector_node]


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("race_auv_bringup"),
        "config",
        "explore_cam_apriltag.yaml",
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description=(
            "Path to explore_cam_apriltag.yaml. Defines the camera, "
            "driver base files, detector settings and tag list."
        ),
    )

    return LaunchDescription([
        config_arg,
        LogInfo(msg=[
            "explore_cam_apriltag: config = ",
            LaunchConfiguration("config"),
        ]),
        OpaqueFunction(function=_build_nodes),
    ])
