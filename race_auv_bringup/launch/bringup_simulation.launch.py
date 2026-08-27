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
    # (still provided by race_auv_sim_pkg/launch/ground_truth_pose.launch.py;
    # uncomment if you want it in the bringup)
    # ground_truth_pose = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('race_auv_sim_pkg'),
    #                      'launch', 'ground_truth_pose.launch.py')
    #     ]),
    # )

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
        mvp_control,
        mvp_mission,
        joy,
        # mvp_c2_top,
        # mvp_c2_vehicle,
        apriltag_pipeline,
        # ground_truth_pose,
        # station_simulation
        # rviz
    ])
