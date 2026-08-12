import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run two instances of the camera_node for two different cameras.
    Each node loads a combination of shared and specific parameter files.
    """
    pkg_share = get_package_share_directory('race_auv_bringup')

    # Common hardware settings that can be applied to both cameras
    hardware_config_path = os.path.join(pkg_share, 'config', 'camera', 'hardware_controls.yaml')
    aux_process_config_path = os.path.join(pkg_share, 'config', 'camera', 'auxiliary_processors.yaml')

    stellar_config_path = os.path.join(pkg_share, 'config', 'camera', 'camera_parameters', 'stellar_cam.yaml')
    
    stellar_1_camera_node = Node(
        package='dwe_camera_driver',
        executable='camera_node',
        name='stellar_camera_node',
        namespace='race',
        output='screen',
        parameters=[
            hardware_config_path,
            aux_process_config_path,
            stellar_config_path,
             {'video.product_name': 'usb-3610000.usb-2.1'}
        ],
        remappings=[
            ('image/compressed', 'stellar1/image/compressed'),
            ('image_lowbw/compressed', 'stellar1/image_lowbw/compressed'),
            ('camera_settings', 'stellar1/camera_settings'),
        ]
    )

    stellar_2_camera_node = Node(
        package='dwe_camera_driver',
        executable='camera_node',
        name='stellar_camera_node',
        namespace='race',
        output='screen',
        parameters=[
            hardware_config_path,
            aux_process_config_path,
            stellar_config_path,
            {'video.product_name': 'usb-3610000.usb-2.3'}
        ],
        remappings=[
            ('image/compressed', 'stellar2/image/compressed'),
            ('image_lowbw/compressed', 'stellar2/image_lowbw/compressed'),
            ('camera_settings', 'stellar2/camera_settings'),
        ]
    )


    return LaunchDescription([
        stellar_1_camera_node,
        stellar_2_camera_node
    ])