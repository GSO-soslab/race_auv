"""Launch the full AprilTag pipeline from a single YAML file.

Reads ``config/simulation/apriltag.yaml`` (in race_auv_bringup) and starts:

* One ``race_auv_camera_pkg/apriltag_detector_node`` per enabled camera in ``cameras:``.
* One ``race_auv_camera_pkg/apriltag_fuser_node`` if ``fuser.enabled`` is true.

The bringup launch file only needs to include this launch file with no
arguments; all per-camera topics, namespaces, intrinsics, and fuser settings
are read from the YAML.

The detector publishes per-tag ``vision_msgs/Detection3DArray`` topics
plus annotated images; the fuser fuses those into a single ``dock_point``
TF for downstream docking control.
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


def _join(ns: str, name: str) -> str:
    if not name:
        return ""
    if name.startswith("/"):
        return name
    if ns:
        return f"/{ns}/{name}"
    return f"/{name}"


def _build_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    cfg = _load_yaml(config_path)
    detector_defaults = cfg.get("detector_defaults", {}) or {}
    fuser_cfg = cfg.get("fuser", {}) or {}

    actions = []
    detection_topics: List[str] = []

    for cam in cfg.get("cameras", []) or []:
        if not cam.get("enabled", True):
            continue
        name = str(cam.get("name", "")).strip()
        if not name:
            continue
        ns = str(cam.get("namespace", "") or "").strip("/")
        image_topic = cam.get("image_topic")
        if not image_topic:
            raise RuntimeError(
                f"camera '{name}' in {config_path} is missing 'image_topic'"
            )
        image_transport = str(cam.get("image_transport", "raw")).lower()
        info_topic = cam.get("info_topic", "") or ""
        camera_frame = cam.get("camera_frame", "") or ""
        tag_frame_prefix = cam.get("tag_frame_prefix", "apriltag")
        output_image_topic = cam.get(
            "output_image_topic", "apriltag_detection/image"
        )
        output_detections_topic = cam.get(
            "output_detections_topic", "apriltag_detection/detections3d"
        )
        publish_rate = float(
            cam.get("publish_rate", detector_defaults.get("publish_rate", 5.0))
        )

        intrinsics = cam.get("intrinsics", {}) or {}

        # Per-camera tag override: if the camera entry has its own `tags:`
        # list, JSON-encode it as `tags_override` so the detector builds only
        # the (family, size) groups it can actually see. Otherwise the global
        # YAML `tags:` list (in `apriltag.tags`) is used as a fallback.
        cam_tags = cam.get("tags", None)
        if cam_tags is None:
            tags_override_json = "[]"
        else:
            tags_override_json = json.dumps(list(cam_tags))

        actions.append(
            Node(
                package="race_auv_camera_pkg",
                executable="apriltag_detector_node",
                name=f"apriltag_detector_{name}",
                namespace=ns,
                output="screen",
                parameters=[{
                    "config_yaml": config_path,
                    "image_transport": image_transport,
                    "image_topic": image_topic,
                    "info_topic": info_topic,
                    "camera_frame": camera_frame,
                    "tag_frame_prefix": tag_frame_prefix,
                    "output_image_topic": output_image_topic,
                    "output_detections_topic": output_detections_topic,
                    "publish_rate": publish_rate,
                    "tags_override": tags_override_json,
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
        detection_topics.append(_join(ns, output_detections_topic))

    if fuser_cfg.get("enabled", True):
        actions.append(
            Node(
                package="race_auv_camera_pkg",
                executable="apriltag_fuser_node",
                name="apriltag_fuser",
                output="screen",
                parameters=[{
                    "config_yaml": config_path,
                    "reference_frame": fuser_cfg.get("reference_frame", "base_link"),
                    "output_frame": fuser_cfg.get("output_frame", "dock_point"),
                    "publish_rate": float(fuser_cfg.get("publish_rate", 5.0)),
                    "min_pairs": int(fuser_cfg.get("min_pairs", 3)),
                    "detection_max_age": float(
                        fuser_cfg.get("detection_max_age", 0.5)
                    ),
                    "tf_timeout": float(fuser_cfg.get("tf_timeout", 0.1)),
                    "detections_topics": detection_topics,
                    "output_pose_topic": fuser_cfg.get(
                        "output_pose_topic", "dock_point/pose"
                    ),
                }],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("race_auv_bringup")
    default_config = os.path.join(
        pkg_share, "config", "simulation", "apriltag.yaml"
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description=(
            "Path to apriltag.yaml. Defines cameras, detector settings, "
            "URDF location, and the fuser block."
        ),
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=_build_nodes),
    ])