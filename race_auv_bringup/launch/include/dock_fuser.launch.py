"""Launch the factor-graph dock_fuser (race_auv_factorgraph) with the vehicle config.

Inputs: the EKF odometry (robot_localization on the front-seat computer,
over zenoh) and cam_front's corner detections from
tag_factorgraph_detection.launch.py. Outputs the odom -> station TF and
/race_auv/dock_point/pose. Parameters: config/station_tags.yaml, then
config/dock_fuser.yaml. The camera matrix comes from the detector's camera_info.

See bringup_camera_tag_factorgraph.launch.py for the combined bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('race_auv_bringup'), 'config')
    # The station's tag list (shared with the tag detector), then the fuser's own.
    params = [os.path.join(config_dir, 'station_tags.yaml'),
              os.path.join(config_dir, 'dock_fuser.yaml')]
    return LaunchDescription([
        Node(
            package='race_auv_factorgraph',
            executable='dock_fuser_node',
            name='dock_fuser',
            output='screen',
            parameters=params,
        ),
    ])
