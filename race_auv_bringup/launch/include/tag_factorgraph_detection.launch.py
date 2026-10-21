"""Launch the factor-graph tag36h11 detectors, one per camera.

Each is race_auv_camera_tag_factorgraph's tag_detector_node, named
tag_detector in the camera's namespace. Parameters: config/station_tags.yaml
(the tag list, shared with dock_fuser), then config/tag_factorgraph_detector.yaml
(shared values under ``/**``, the camera's own under ``/<camera>/tag_detector``,
including the tags it reports, ``camera_tag_ids``).

Launch argument ``cameras`` (comma-separated, default ``cam_front,cam_down``)
picks which detectors run. Whether dock_fuser *uses* a camera is set
separately, in dock_fuser.yaml (``camera_names``, ``cameras.<name>.enabled``):
a detector can run, and be recorded, while the graph ignores it.

Assumes the cameras are already running (multi_camera.launch.py). See
bringup_camera_tag_factorgraph.launch.py for the combined bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _detectors(context):
    config_dir = os.path.join(get_package_share_directory('race_auv_bringup'), 'config')
    # The station's tag list (shared with dock_fuser), then the detector's own.
    params = [os.path.join(config_dir, 'station_tags.yaml'),
              os.path.join(config_dir, 'tag_factorgraph_detector.yaml')]
    cameras = [c.strip() for c in LaunchConfiguration('cameras').perform(context).split(',')
               if c.strip()]
    return [
        Node(
            package='race_auv_camera_tag_factorgraph',
            executable='tag_detector_node',
            name='tag_detector',
            namespace=camera,
            output='screen',
            parameters=params,
        )
        for camera in cameras
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cameras', default_value='cam_front,cam_down',
                              description='comma-separated cameras to run a tag detector for'),
        OpaqueFunction(function=_detectors),
    ])
