import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz(context, *args, **kwargs):
    """Fill the image topics into rviz/factorgraph.rviz and start rviz2 on it.

    RViz configs have no substitution, so the template's @...@ placeholders
    are replaced here and the result written to a temporary file.
    """
    lc = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    actions = []
    down = lc('down_image')
    if lc('down_rotate180').lower() in ('1', 'true', 'yes'):
        # cam_down is mounted upside down: turn its picture 180 deg for display
        # (race_auv_camera_tag_factorgraph image_rotate_node) and show that.
        rotated = down[:-len('/compressed')] if down.endswith('/compressed') else down
        rotated += '_rot180/compressed'
        actions.append(Node(
            package='race_auv_camera_tag_factorgraph', executable='image_rotate_node',
            name='cam_down_rotate', output='screen',
            parameters=[{'input': down, 'output': rotated}]))
        down = rotated
    template = os.path.join(get_package_share_directory('race_auv_bringup'),
                            'rviz', 'factorgraph.rviz')
    with open(template, encoding='utf-8') as f:
        config = (f.read()
                  .replace('@FRONT_IMAGE@', lc('front_image'))
                  .replace('@DOWN_IMAGE@', down)
                  .replace('@DETECTOR_IMAGE@', lc('detector_image')))
    out = tempfile.NamedTemporaryFile('w', suffix='_factorgraph.rviz', delete=False)
    out.write(config)
    out.close()
    actions.append(Node(
        package='rviz2', executable='rviz2', name='rviz_factorgraph', output='screen',
        arguments=['-d', out.name],
        parameters=[{'use_sim_time': lc('use_sim_time').lower() in ('1', 'true', 'yes')}],
    ))
    return actions


def generate_launch_description():
    """RViz on the topside for the factor-graph docking pipeline running on the vehicle.

    Shows, live over the network (same ROS_DOMAIN_ID / zenoh router as the
    vehicle):
      * top-down in race_auv/odom: the graph's real-time AUV track
        (/dock_fuser/path) and the EKF's (/dock_fuser/ekf_path), and TF;
      * the two camera feeds (top row) and the cam_front tag detector image
        (bottom row, larger). cam_down is mounted upside down, so its feed is
        turned 180 deg here, on the topside (image_rotate_node,
        down_rotate180:=false to show it as published).

    Needs, on the vehicle: dock_fuser publish_debug: true (dock_fuser.yaml)
    and the cam_front detector's publish_image: true
    (tag_factorgraph_detector.yaml), both set. On the topside:
    ros-jazzy-compressed-image-transport for the compressed image topics.

    Usage:
      ros2 launch race_auv_bringup bringup_rviz_factorgraph.launch.py
      # replaying a bag on the topside (nodes on the bag's /clock):
      ros2 launch race_auv_bringup bringup_rviz_factorgraph.launch.py use_sim_time:=true
    """
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when watching a bag replay with --clock'),
        # Camera topics, as published by multi_camera.launch.py. A topic
        # ending in /compressed is decoded by image_transport's compressed plugin.
        DeclareLaunchArgument('front_image', default_value='/race/stellar2/image/compressed',
                              description='cam_front (stellar2) image topic'),
        DeclareLaunchArgument('down_image', default_value='/race/stellar1/image/compressed',
                              description='cam_down (stellar1) image topic'),
        DeclareLaunchArgument('down_rotate180', default_value='true',
                              description='turn the cam_down feed 180 deg for display'),
        DeclareLaunchArgument('detector_image',
                              default_value='/cam_front/apriltag_detection/image/compressed',
                              description='tag detector debug image (rectified, tags drawn)'),
        OpaqueFunction(function=_rviz),
    ])
