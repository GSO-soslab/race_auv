"""Launch the AprilTag detector nodes described in config/apriltag.yaml.

Assumes the cameras are already running -- see multi_camera.launch.py,
which publishes:

    /race/stellar1/image/compressed   -> cam_down
    /race/stellar2/image/compressed   -> cam_front

Each enabled entry in apriltag.yaml's ``cameras:`` list gets one
``race_auv_camera_pkg/apriltag_detector_node`` here. See
bringup_camera_perception.launch.py for the combined bringup.
"""

import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('race_auv_bringup')
    config_path = os.path.join(bringup_share, 'config', 'apriltag.yaml')

    with open(config_path, 'r') as f:
        cfg = (yaml.safe_load(f) or {}).get('apriltag', {}) or {}
    detector_defaults = cfg.get('detector_defaults', {}) or {}

    detector_nodes = []
    for cam in cfg.get('cameras', []) or []:
        if not cam.get('enabled', True):
            continue

        cam_name = str(cam.get('name', '')).strip()
        namespace = str(
            cam.get('detector_namespace', cam.get('namespace', '')) or ''
        ).strip('/')
        intrinsics = cam.get('intrinsics', {}) or {}
        tags = cam.get('tags')
        tags_override = json.dumps(list(tags)) if tags else '[]'

        detector_nodes.append(Node(
            package='race_auv_camera_pkg',
            executable='apriltag_detector_node',
            name=f'apriltag_detector_{cam_name}',
            namespace=namespace,
            output='screen',
            parameters=[{
                'config_yaml': config_path,
                'image_transport': str(cam.get('image_transport', 'raw')),
                'image_topic': cam['image_topic'],
                'info_topic': str(cam.get('info_topic', '') or ''),
                'camera_frame': str(cam.get('camera_frame', '') or ''),
                'output_image_topic': cam.get(
                    'output_image_topic', 'apriltag_detection/image'
                ),
                'output_detections_topic': cam.get(
                    'output_detections_topic', 'apriltag_detection/detections3d'
                ),
                'publish_rate': float(
                    cam.get('publish_rate', detector_defaults.get('publish_rate', 5.0))
                ),
                'process_scale': float(cam.get('process_scale', 1.0)),
                'jpeg_quality': int(cam.get('jpeg_quality', 80)),
                'min_edge_dist': int(
                    cam.get('min_edge_dist', detector_defaults.get('min_edge_dist', 10))
                ),
                'detector_backend': str(
                    cam.get('detector_backend',
                            detector_defaults.get('detector_backend', 'python'))
                ).lower(),
                'image_pipeline': str(
                    cam.get('image_pipeline',
                            detector_defaults.get('image_pipeline', 'cpu'))
                ).lower(),
                'gpu_rectify_interval': int(
                    cam.get('gpu_rectify_interval',
                            detector_defaults.get('gpu_rectify_interval', 4))
                ),
                'cuda_tile_size': int(
                    cam.get('cuda_tile_size', detector_defaults.get('cuda_tile_size', 4))
                ),
                'cuda_nominal_size': float(
                    cam.get('cuda_nominal_size',
                            detector_defaults.get('cuda_nominal_size', 0.125))
                ),
                'cuda_max_tags': int(
                    cam.get('cuda_max_tags', detector_defaults.get('cuda_max_tags', 64))
                ),
                'tags_override': tags_override,
                'intrinsics.fx': float(intrinsics.get('fx', 0.0)),
                'intrinsics.fy': float(intrinsics.get('fy', 0.0)),
                'intrinsics.cx': float(intrinsics.get('cx', 0.0)),
                'intrinsics.cy': float(intrinsics.get('cy', 0.0)),
                'intrinsics.width': int(intrinsics.get('width', 0)),
                'intrinsics.height': int(intrinsics.get('height', 0)),
                'intrinsics.distortion': [
                    float(v) for v in intrinsics.get('distortion', [0.0] * 5)
                ],
                'intrinsics.fisheye': bool(intrinsics.get('fisheye', False)),
            }],
        ))

    return LaunchDescription(detector_nodes)
