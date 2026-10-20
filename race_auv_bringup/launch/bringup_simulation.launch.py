import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import time




def generate_launch_description():
    arg_robot_name = 'race_auv'
    robot_bringup = arg_robot_name + '_bringup'
    robot_description = arg_robot_name + '_description'
    arg_station_name = 'race_station'
    station_bringup = arg_station_name + '_bringup'

    # simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_bringup), 'launch','include','simulation','simulation.launch.py')]),
        launch_arguments = {'arg_robot_name': arg_robot_name}.items()    
    )

    # robot localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_bringup), 'launch','include','simulation','localization_sim.launch.py')]),
        launch_arguments = {'arg_robot_name': arg_robot_name}.items()  
    )
    
    #description URDF
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_bringup), 'launch','include','description.launch.py')]),
        launch_arguments = {'arg_robot_name': arg_robot_name}.items()  
    )

    #mvp_control
    mvp_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_bringup), 'launch','include','simulation','mvp_control_sim.launch.py')]),
        launch_arguments = {'arg_robot_name': arg_robot_name}.items()  
    )

    #mvp_mission
    mvp_mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_bringup), 'launch','include','simulation','mvp_mission_sim.launch.py')]),
        launch_arguments = {'arg_robot_name': arg_robot_name}.items()  
    )

    #joy
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_bringup), 'launch','include','joy.launch.py')]),
        launch_arguments = {'arg_robot_name': arg_robot_name}.items()  
    )

    # c2 topside
    mvp_c2_top = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_bringup), 'launch','include','simulation','mvp_c2_topside_sim.launch.py')]),
        launch_arguments = {'arg_robot_name': arg_robot_name}.items()  
    )

    # c2 vehicle
    mvp_c2_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_bringup), 'launch','include','simulation','mvp_c2_vehicle_sim.launch.py')]),
        launch_arguments = {'arg_robot_name': arg_robot_name}.items()  
    )

    # apriltag pipeline (per-camera detectors + multi-camera fuser).
    # All topics, namespaces, and intrinsics live in
    # race_auv_bringup/config/simulation/apriltag.yaml; no arguments needed.
    apriltag_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(robot_bringup),
                         'launch', 'include', 'simulation',
                         'apriltag_sim.launch.py')
        ]),
    )

    # ground-truth docking-station pose in auv base_link
    # (provided by race_auv_sim_pkg/launch/ground_truth_pose.launch.py)
    # uncommented by maryam: also launches dock_point_rpy_monitor now
    # (see ground_truth_pose.launch.py) -- monitoring/verification for
    # the docking mission/control loop.
    ground_truth_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('race_auv_sim_pkg'),
                         'launch', 'ground_truth_pose.launch.py')
        ]),
    )

    # race station bringup
    station_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(station_bringup), 'launch','bringup_simulation.launch.py')]),
    )

    #Rviz
    rviz_config_dir = os.path.join( get_package_share_directory(robot_description), 'rviz', 'config.rviz' )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_config_dir]],
        )
    
    return LaunchDescription([
        simulation,
        localization,
        description,
        # uncommented by maryam: testing the PID-guidance path now --
        # race_docking_mission feeds mvp_helm's bhv_direct_control, which
        # forwards to mvp_control_ros_node, instead of running
        # race_docking_control's docking_control_node directly. Comment
        # these back out (and re-enable docking_control_node) to go back
        # to the RBF controller -- never run both at once, they write to
        # the same control/thruster/* topics.
        mvp_control,
        mvp_mission,
        # commented out by maryam: joy_node only feeds mvp_helm's
        # bhv_teleop -- not needed for the direct-control docking path.
        # joy,
        # mvp_c2_top,
        # mvp_c2_vehicle,
        apriltag_pipeline,
        # uncommented by maryam: also launches dock_point_rpy_monitor now
        # (see ground_truth_pose.launch.py) -- monitoring/verification
        # for the docking mission/control loop.
        ground_truth_pose,
        # station_simulation
        # rviz
    ])
