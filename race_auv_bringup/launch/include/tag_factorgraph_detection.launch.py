"""Launch the factor-graph tag36h11 detectors, one per camera.

Each is race_auv_camera_tag_factorgraph's tag_detector_node, named
tag_detector in the camera's namespace. Parameters: config/station_tags.yaml
(the tag list, shared with dock_fuser), then config/tag_factorgraph_detector.yaml
(shared values under ``/**``, the camera's own under ``/<camera>/tag_detector``).

Assumes the cameras are already running (multi_camera.launch.py). See
bringup_camera_tag_factorgraph.launch.py for the combined bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Cameras with a detector. dock_fuser itself listens to one, cam_front.
CAMERAS = ['cam_front', 'cam_down']


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('race_auv_bringup'), 'config')
    # The station's tag list (shared with dock_fuser), then the detector's own.
    params = [os.path.join(config_dir, 'station_tags.yaml'),
              os.path.join(config_dir, 'tag_factorgraph_detector.yaml')]
    return LaunchDescription([
        Node(
            package='race_auv_camera_tag_factorgraph',
            executable='tag_detector_node',
            name='tag_detector',
            namespace=camera,
            output='screen',
            parameters=params,
        )
        for camera in CAMERAS
    ])
