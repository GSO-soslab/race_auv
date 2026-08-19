
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml
# import xacro
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from setuptools import Command
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_name = 'race_auv'
    robot_description = robot_name + '_description'
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    path_to_urdf = os.path.join( get_package_share_directory(robot_description), 'urdf', 'base.urdf' )

    use_sim_time = LaunchConfiguration('use_sim_time')

    with open(path_to_urdf, 'r') as infp:
        robot_desc = infp.read()


    return LaunchDescription([
        use_sim_time_arg,
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'robot_description' : robot_desc},
                        {'frame_prefix': robot_name +'/'},
                        {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom2world',
            # arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", robot_name+'/odom', robot_name+'/world']    
            arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", robot_name+'/world', robot_name+'/odom'] 
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world2ned',
            parameters=[{'use_sim_time': use_sim_time}],
            # arguments = ["0.0", "0.0", "0.0", "1.571", "0.0", "3.1415", robot_name+'/world', robot_name+'/world_ned']    
            arguments = [
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '3.1415', '--pitch', '0.0', '--yaw', '1.571',
                '--frame-id', robot_name + '/world',
                '--child-frame-id', robot_name + '/world_ned',
            ]
        ),
])
