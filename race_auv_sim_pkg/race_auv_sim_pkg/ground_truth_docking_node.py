"""Ground-truth relative pose of the docking station in the AUV base frame.

Subscribes to the two Stonefish odometry topics:

* ``/race_auv/stonefish/odometry``
* ``/race_station/stonefish/odometry``

and publishes ``geometry_msgs/PoseStamped`` representing the pose of
``race_station/base_link`` expressed in ``race_auv/base_link``.

The odometry sensors in the Stonefish scenario are mounted on the vehicle
``Base`` link with zero origin/rotation, so no static offset is applied by
default (offsets can be parameterized if that changes).
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R


def _pose_to_matrix(pose) -> np.ndarray:
    """geometry_msgs/Pose -> 4x4 homogeneous transform."""
    q = pose.orientation
    p = pose.position
    T = np.eye(4)
    T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _matrix_to_pose_msg(T: np.ndarray) -> PoseStamped:
    """4x4 homogeneous transform -> geometry_msgs/PoseStamped (pose only)."""
    msg = PoseStamped()
    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])
    q = R.from_matrix(T[:3, :3]).as_quat()
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    return msg


class GroundTruthDockingNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_docking")

        self.declare_parameter("auv_odom_topic", "/race_auv/stonefish/odometry")
        self.declare_parameter(
            "station_odom_topic", "/race_station/stonefish/odometry"
        )
        self.declare_parameter("auv_base_frame", "race_auv/base_link")
        self.declare_parameter("station_base_frame", "race_station/base_link")
        self.declare_parameter(
            "output_topic", "/race_auv/docking_station/ground_truth/pose"
        )
        self.declare_parameter("publish_rate", 10.0)

        auv_odom_topic = self.get_parameter("auv_odom_topic").value
        station_odom_topic = self.get_parameter("station_odom_topic").value
        self._auv_base_frame = self.get_parameter("auv_base_frame").value
        self._station_base_frame = self.get_parameter("station_base_frame").value
        output_topic = self.get_parameter("output_topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)

        self._latest_auv_odom: Odometry | None = None
        self._latest_station_odom: Odometry | None = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Odometry, auv_odom_topic, self._auv_odom_cb, qos)
        self.create_subscription(
            Odometry, station_odom_topic, self._station_odom_cb, qos
        )

        self._pub = self.create_publisher(PoseStamped, output_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f"Publishing ground-truth pose of {self._station_base_frame} "
            f"in {self._auv_base_frame} on {output_topic}"
        )

    def _auv_odom_cb(self, msg: Odometry) -> None:
        self._latest_auv_odom = msg

    def _station_odom_cb(self, msg: Odometry) -> None:
        self._latest_station_odom = msg

    def _publish(self) -> None:
        if self._latest_auv_odom is None or self._latest_station_odom is None:
            self.get_logger().warn(
                "Waiting for both odometry messages...", throttle_duration_sec=5.0
            )
            return

        T_w_auv = _pose_to_matrix(self._latest_auv_odom.pose.pose)
        T_w_station = _pose_to_matrix(self._latest_station_odom.pose.pose)

        # T_auv_station = inv(T_w_auv) * T_w_station
        T_auv_station = np.linalg.inv(T_w_auv) @ T_w_station

        msg = _matrix_to_pose_msg(T_auv_station)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._auv_base_frame

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
