import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Bring up the two cameras, the factor-graph tag detectors, and the factor graph.

    Like bringup_camera_perception.launch.py, with race_auv_camera_tag_factorgraph's
    detector instead of race_auv_camera_pkg's (both publish on
    /<camera>/apriltag_detection/..., so run one of the two), plus the
    dock_fuser factor graph that turns the corners and the EKF odometry into
    the station pose.
    """
    include_dir = os.path.join(
        get_package_share_directory('race_auv_bringup'), 'launch', 'include'
    )

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, name))
        )

    return LaunchDescription([
        include('multi_camera.launch.py'),
        # Give the camera drivers a few seconds to open the devices and
        # start publishing before the detectors subscribe.
        TimerAction(period=5.0, actions=[include('tag_factorgraph_detection.launch.py')]),
        # The fuser last, once the detectors are up (their camera_info is
        # latched, so it is received even if it was published earlier).
        TimerAction(period=12.0, actions=[include('dock_fuser.launch.py')]),
    ])
