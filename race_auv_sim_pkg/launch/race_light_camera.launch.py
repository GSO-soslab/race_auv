import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _render_scn(template_path: str, args: dict) -> str:
    """Replace $(arg <name>) occurrences in the scenario template."""
    with open(template_path, 'r') as f:
        text = f.read()
    for key, value in args.items():
        text = text.replace(f'$(arg {key})', str(value))
    return text


def _build_simulator(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    sim_world  = LaunchConfiguration('sim_world').perform(context)

    world_of_stonefish_dir = get_package_share_directory('world_of_stonefish')
    simulation_data = os.path.join(world_of_stonefish_dir, 'data/')
    template_path   = os.path.join(world_of_stonefish_dir, 'world', sim_world)

    rendered = _render_scn(template_path, {'robot_name': robot_name})

    tmp_dir = tempfile.mkdtemp(prefix='race_light_camera_')
    rendered_path = os.path.join(tmp_dir, 'race_light_camera.scn')
    with open(rendered_path, 'w') as f:
        f.write(rendered)

    return [
        Node(
            package='stonefish_ros2',
            executable='stonefish_simulator',
            name='stonefish_simulator',
            output='screen',
            arguments=[
                simulation_data,
                rendered_path,
                '100',
                '1200',
                '800',
                'high',
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='race_light_camera',
            description='Namespace for ROS topics.',
        ),
        DeclareLaunchArgument(
            'sim_world',
            default_value='race_light_camera.scn',
            description='Stonefish scenario template (in world_of_stonefish/world/).',
        ),
        OpaqueFunction(function=_build_simulator),
    ])
