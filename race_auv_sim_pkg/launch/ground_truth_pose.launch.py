"""Launch the ground-truth docking-station relative-pose node.

Publishes ``geometry_msgs/PoseStamped`` of ``race_station/base_link`` in
``race_auv/base_link`` using the Stonefish ground-truth odometry topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    auv_odom_topic_arg = DeclareLaunchArgument(
        "auv_odom_topic",
        default_value="/race_auv/stonefish/odometry",
        description="Ground-truth AUV odometry topic.",
    )
    station_odom_topic_arg = DeclareLaunchArgument(
        "station_odom_topic",
        default_value="/race_station/stonefish/odometry",
        description="Ground-truth docking-station odometry topic.",
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/race_auv/docking_station/ground_truth/pose",
        description="Output PoseStamped topic.",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="10.0",
        description="Publication rate (Hz).",
    )

    return LaunchDescription([
        auv_odom_topic_arg,
        station_odom_topic_arg,
        output_topic_arg,
        publish_rate_arg,
        Node(
            package="race_auv_sim_pkg",
            executable="ground_truth_docking_node",
            name="ground_truth_docking",
            output="screen",
            parameters=[{
                "auv_odom_topic": LaunchConfiguration("auv_odom_topic"),
                "station_odom_topic": LaunchConfiguration("station_odom_topic"),
                "auv_base_frame": "race_auv/base_link",
                "station_base_frame": "race_station/base_link",
                "output_topic": LaunchConfiguration("output_topic"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            }],
        ),
    ])
