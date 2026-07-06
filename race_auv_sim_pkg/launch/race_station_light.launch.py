import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _render_scn(template_path: str, args: dict) -> str:
    """Replace $(arg <name>) occurrences in the scenario template."""
    with open(template_path, 'r') as f:
        text = f.read()
    for key, value in args.items():
        text = text.replace(f'$(arg {key})', str(value))
    return text


def _build_simulator(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)
    cfg = _load_yaml(config_path)

    sim_cfg   = cfg.get('scenario', {}) or {}
    env_cfg   = cfg.get('environment', {}) or {}
    ocean_cfg = env_cfg.get('ocean', {}) or {}
    atm_cfg   = env_cfg.get('atmosphere', {}) or {}
    waves_cfg = ocean_cfg.get('waves', {}) or {}
    part_cfg  = ocean_cfg.get('particles', {}) or {}
    sun_cfg   = atm_cfg.get('sun', {}) or {}
    rend_cfg  = cfg.get('rendering', {}) or {}

    sim_world  = LaunchConfiguration('sim_world').perform(context)
    robot_name = str(sim_cfg.get('robot_name', 'race_station_light'))

    world_of_stonefish_dir = get_package_share_directory('world_of_stonefish')
    simulation_data = os.path.join(world_of_stonefish_dir, 'data/')
    template_path   = os.path.join(world_of_stonefish_dir, 'world', sim_world)

    scn_args = {
        'robot_name':        robot_name,
        'density':           ocean_cfg.get('density', 1023.0),
        'jerlov':            ocean_cfg.get('jerlov', 0.2),
        'temperature':       ocean_cfg.get('temperature', 15.0),
        'wave_height':       waves_cfg.get('height', 0.0),
        'particles_enabled': str(part_cfg.get('enabled', True)).lower(),
        'uniform_current':   ocean_cfg.get('uniform_current', '0.0 0.0 0.0'),
        'sun_azimuth':       sun_cfg.get('azimuth', 40.0),
        'sun_elevation':     sun_cfg.get('elevation', 10.0),
        'station_xyz':       sim_cfg.get('station_xyz', '-1.0 0.0 1.0'),
        'station_rpy':       sim_cfg.get('station_rpy', '0.0 0.0 0.0'),
        'bottom_xyz':        sim_cfg.get('bottom_xyz', '0.0 0.0 40.0'),
        'world_xyz':         sim_cfg.get('station_xyz', '-1.0 0.0 1.0'),
        'world_rpy':         sim_cfg.get('station_rpy', '0.0 0.0 0.0'),
    }

    rendered = _render_scn(template_path, scn_args)

    tmp_dir = tempfile.mkdtemp(prefix='race_station_light_')
    rendered_path = os.path.join(tmp_dir, 'race_station_light.scn')
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
                str(rend_cfg.get('rate', 100)),
                str(rend_cfg.get('res_x', 1200)),
                str(rend_cfg.get('res_y', 800)),
                str(rend_cfg.get('quality', 'high')),
            ],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('race_auv_sim_pkg')
    default_config = os.path.join(pkg_share, 'config', 'sim.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to sim.yaml with scenario and environment settings.',
        ),
        DeclareLaunchArgument(
            'sim_world',
            default_value='race_station_light.scn',
            description='Stonefish scenario template (in world_of_stonefish/world/).',
        ),
        OpaqueFunction(function=_build_simulator),
    ])
