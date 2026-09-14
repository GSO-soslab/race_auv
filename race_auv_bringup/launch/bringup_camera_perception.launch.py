import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Bring up the two cameras, then the AprilTag detectors that read them."""
    include_dir = os.path.join(
        get_package_share_directory('race_auv_bringup'), 'launch', 'include'
    )

    multi_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'multi_camera.launch.py')
        )
    )

    apriltag_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'apriltag_detection.launch.py')
        )
    )

    return LaunchDescription([
        multi_camera,
        # Give the camera drivers a few seconds to open the devices and
        # start publishing before the detectors subscribe.
        TimerAction(period=3.0, actions=[apriltag_detection]),
    ])
