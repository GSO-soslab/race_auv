"""Launch the Stonefish AprilTag bridge node.

The URDF is located by combining an ament package name with a relative path
inside its share/ directory, exactly like
``race_auv_bringup/launch/include/description.launch.py`` does for the robot
URDF (``get_package_share_directory(pkg) + 'urdf/base.urdf'``). The resolved
absolute path is then handed to the node as the ``urdf_path`` ROS parameter.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_urdf(context, *args, **kwargs):
    """If ``urdf_path`` is empty, join ``urdf_package`` + ``urdf_filename``.

    Returns a list of additional actions (a SetLaunchConfiguration) that
    pre-populate the ``urdf_path`` substitution so the Node below sees the
    absolute file path.
    """
    explicit = LaunchConfiguration('urdf_path').perform(context)
    if explicit:
        return []
    pkg = LaunchConfiguration('urdf_package').perform(context)
    if not pkg:
        return []
    rel = LaunchConfiguration('urdf_filename').perform(context) or 'urdf/base.urdf'
    share = get_package_share_directory(pkg)
    abs_path = os.path.join(share, rel)
    return [SetLaunchConfiguration('urdf_path', abs_path)]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('race_auv_sim_pkg')
    default_config = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    config_yaml_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value=default_config,
        description=(
            'YAML file listing the apriltag ids/sizes and the object URDF '
            'location. Override to point at a scene-specific config.'
        ),
    )
    urdf_package_arg = DeclareLaunchArgument(
        'urdf_package',
        default_value='',
        description=(
            'ROS package (workspace directory) that ships the URDF. '
            'Empty -> use object.urdf_package from the YAML.'
        ),
    )
    urdf_filename_arg = DeclareLaunchArgument(
        'urdf_filename',
        default_value='',
        description=(
            'URDF file path inside the package share/ dir. '
            'Empty -> use object.urdf_filename from the YAML '
            '(defaults to urdf/base.urdf).'
        ),
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value='',
        description=(
            'Final absolute URDF path. If empty, the launch joins '
            'urdf_package + urdf_filename via get_package_share_directory.'
        ),
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/race_auv/camera1/stonefish/data/image_color',
        description='Input color image topic.',
    )
    info_topic_arg = DeclareLaunchArgument(
        'info_topic',
        default_value='/race_auv/camera1/stonefish/data/camera_info',
        description='Input camera info topic.',
    )

    node = Node(
        package='race_auv_sim_pkg',
        executable='stonefish_apriltag_node',
        name='stonefish_apriltag_node',
        output='screen',
        parameters=[{
            'config_yaml': LaunchConfiguration('config_yaml'),
            'image_topic': LaunchConfiguration('image_topic'),
            'info_topic': LaunchConfiguration('info_topic'),
            'urdf_path': LaunchConfiguration('urdf_path'),
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_filename': LaunchConfiguration('urdf_filename'),
        }],
    )

    return LaunchDescription([
        config_yaml_arg,
        urdf_package_arg,
        urdf_filename_arg,
        urdf_path_arg,
        image_topic_arg,
        info_topic_arg,
        OpaqueFunction(function=_resolve_urdf),
        node,
    ])
