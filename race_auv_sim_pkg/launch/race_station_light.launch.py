import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
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


_BAG_RUNNER_SCRIPT = r'''
import argparse, os, signal, subprocess, sys, time
p = argparse.ArgumentParser()
p.add_argument("-o", "--output-base", required=True,
               help="Bag directory base name; numeric suffix _N is appended if it exists.")
p.add_argument("--duration", type=float, default=0.0)
p.add_argument("--jerlov-tag", default="")
p.add_argument("--jerlov-value", default="")
p.add_argument("--config", default="")
p.add_argument("--scn-file", default="")
p.add_argument("--start-delay", type=float, default=0.0,
               help="Seconds to wait before spawning ros2 bag record.")
p.add_argument("--topics", nargs="+", default=[])
a = p.parse_args()

n = 0
while os.path.exists(a.output_base if n == 0 else f"{a.output_base}_{n}"):
    n += 1
bag_dir = a.output_base if n == 0 else f"{a.output_base}_{n}"
run_suffix = "" if n == 0 else f"_{n}"

if a.start_delay > 0:
    time.sleep(a.start_delay)

cmd = ["ros2", "bag", "record", "-o", bag_dir, "--topics", *a.topics]
try:
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
except FileNotFoundError:
    print("ros2 CLI not found in PATH", file=sys.stderr)
    sys.exit(127)

def write_info():
    if not a.config:
        return
    with open(os.path.join(bag_dir, "sim_info.txt"), "w") as f:
        f.write(f"# jerlov_tag:     {a.jerlov_tag}\n")
        f.write(f"# jerlov_value:   {a.jerlov_value}\n")
        f.write(f"# run_suffix:     {run_suffix}\n")
        f.write(f"# start_delay:    {a.start_delay}\n")
        f.write(f"# duration_sec:   {a.duration}\n")
        f.write(f"# sim_yaml:       {a.config}\n")
        f.write(f"# scn_file:       {a.scn_file}\n\n")
        f.write("# sim.yaml contents:\n")
        with open(a.config, "r") as src:
            f.write(src.read())
        if a.scn_file and os.path.isfile(a.scn_file):
            f.write("\n# rendered scenario (.scn) contents:\n")
            with open(a.scn_file, "r") as src:
                f.write(src.read())

for _ in range(200):
    if os.path.isdir(bag_dir):
        write_info()
        break
    time.sleep(0.1)

exit_code = 0
if a.duration > 0:
    try:
        exit_code = proc.wait(timeout=a.duration)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        sys.exit(proc.wait())
    try:
        exit_code = proc.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        exit_code = proc.wait()
else:
    exit_code = proc.wait()

write_info()
sys.exit(exit_code)
'''


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
    overlay_cfg = cfg.get('overlay', {}) or {}
    rec_cfg = cfg.get('recording', {}) or {}

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

    actions = [
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
        Node(
            package='race_auv_sim_pkg',
            executable='light_camera_overlay',
            name='light_camera_overlay',
            output='screen',
            condition=IfCondition(str(overlay_cfg.get('enabled', True)).lower()),
            parameters=[overlay_cfg],
        ),
    ]

    rec_enabled = str(rec_cfg.get('enabled', False)).lower() == 'true'
    rec_topics = rec_cfg.get('topics', []) or []
    rec_output_dir = rec_cfg.get('output_dir', '/tmp')
    rec_bag_name = rec_cfg.get('bag_name', 'race_station_light')
    rec_duration = float(rec_cfg.get('duration_sec', 0) or 0)
    rec_start_delay = float(rec_cfg.get('start_delay_sec', 0) or 0)

    if rec_enabled and rec_topics:
        jerlov_tag = 'jerlov_' + str(ocean_cfg.get('jerlov', 0.0)).replace('.', 'p')
        bag_base = os.path.join(rec_output_dir, f'{rec_bag_name}_{jerlov_tag}')

        bag_cmd = [
            'python3', '-c', _BAG_RUNNER_SCRIPT,
            '-o', bag_base,
            '--duration', str(rec_duration),
            '--start-delay', str(rec_start_delay),
            '--jerlov-tag', jerlov_tag,
            '--jerlov-value', str(ocean_cfg.get('jerlov', 0.0)),
            '--config', config_path,
            '--scn-file', rendered_path,
            '--topics', *rec_topics,
        ]

        actions.append(
            ExecuteProcess(
                cmd=bag_cmd,
                name='ros2_bag_record',
                output='screen',
            )
        )

    return actions


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
