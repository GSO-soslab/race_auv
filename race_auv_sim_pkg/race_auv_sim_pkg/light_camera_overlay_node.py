"""Light camera overlay node.

Subscribes to the docking-station light's and dwe_camera's odometries (both
expressed in world NED) plus the camera image and ``CameraInfo``, and
publishes:

* an annotated ``sensor_msgs/CompressedImage`` on ``overlay_topic`` with
  the light's local coordinate frame drawn into the camera view;
* a ``nav_msgs/Odometry`` on ``light_in_camera_odom_topic`` carrying the
  pose of the light expressed in the camera frame (``+X`` right,
  ``+Y`` down, ``+Z`` forward, matching the optical convention);
* a ``nav_msgs/Odometry`` on ``light_in_camera_pixel_odom_topic`` carrying
  the light origin's 2D pixel coordinates ``(u, v)`` in the image, packed
  as ``pose.pose.position = (u, v, 0)``.

Topics (defaults assume ``robot_name=race_station_light``):

* ``light_odom_topic``             - ``nav_msgs/Odometry``, light pose in
  world NED.
* ``camera_odom_topic``            - ``nav_msgs/Odometry``, camera pose in
  world NED.
* ``camera_info_topic``            - ``sensor_msgs/CameraInfo`` with the
  pinhole ``K`` used to project the 3D annotations into the image.
* ``camera_image_topic``           - ``sensor_msgs/Image`` (raw BGR) to
  annotate.
* ``overlay_topic``                - ``sensor_msgs/CompressedImage`` (jpeg)
  with the drawn overlay.
* ``light_in_camera_odom_topic``   - ``nav_msgs/Odometry``, light pose in
  the camera optical frame.
* ``light_in_camera_pixel_odom_topic`` - ``nav_msgs/Odometry``, light
  origin as ``(u, v)`` pixels in the image.

The annotation is a filled circle at the light's projected origin plus
three axis lines from that origin to the projections of points
``axis_length`` metres along the light's local ``+X`` (red), ``+Y``
(green), and ``+Z`` (blue) directions, with the axis tip labelled.
A small filled dot with outline and ``pixel_dot_label`` is drawn on top
of everything at the projected ``(u, v)`` to mark the pixel coordinate
of the light origin.
Axes whose 3D tip falls behind the camera (``z_cam <= 0``) are skipped
instead of being drawn at a meaningless location.

 Sync: image and camera info are aligned by stamp with
``message_filters.ApproximateTimeSynchronizer`` (slop ``sync_slop_sec``
seconds). The two odometries are received via simple subscriptions and
the latest snapshot is used for each image, so the derived output is
always fresh (within ~20 ms at 50 Hz camera odom). All three outputs
(``light_overlay``, ``light_in_camera/odometry``,
``light_in_camera/pixel_odometry``) are emitted at the same rate and
with the same ``header.stamp`` as the image they were computed against,
so the bag stream ``image_color / light_in_camera/odometry /
light_in_camera/pixel_odometry`` is trivially sample-aligned. The pixel
odometry is only emitted when the light origin projects in front of the
camera (``z_cam > 0``); otherwise the 2D coordinate is undefined.

``enabled`` (default ``True``) controls only the visual side: when
``False`` the image draw and ``overlay_topic`` publish are skipped
(``image_skipped`` counter ticks up), but the two derived odoms are
still published at the image rate, so the bag still records
``light_in_camera/odometry`` and ``light_in_camera/pixel_odometry``.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = qw * qx * s, qw * qy * s, qw * qz * s
    xx, xy, xz = qx * qx * s, qx * qy * s, qx * qz * s
    yy, yz, zz = qy * qy * s, qy * qz * s, qz * qz * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz,         xz + wy],
            [xy + wz,         1.0 - (xx + zz), yz - wx],
            [xz - wy,         yz + wx,         1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quat(R: np.ndarray) -> np.ndarray:
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        S = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif m00 > m11 and m00 > m22:
        S = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q


def _odom_to_T(odom: Odometry) -> np.ndarray:
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _k_matrix(info: CameraInfo) -> np.ndarray:
    return np.array(info.k, dtype=np.float64).reshape(3, 3)


def _project(p_cam: np.ndarray, K: np.ndarray) -> Optional[Tuple[int, int]]:
    z = p_cam[2]
    if z <= 1e-3:
        return None
    u = K[0, 0] * p_cam[0] / z + K[0, 2]
    v = K[1, 1] * p_cam[1] / z + K[1, 2]
    return int(round(u)), int(round(v))


def _bgr_param(node: Node, name: str, default: List[int]) -> Tuple[int, int, int]:
    raw = node.get_parameter(name).value
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raw = default
    return int(raw[0]), int(raw[1]), int(raw[2])


class LightCameraOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('light_camera_overlay')

        self.declare_parameter('enabled', True)
        self.declare_parameter('light_odom_topic',
                               '/race_station_light/stonefish/light/odometry')
        self.declare_parameter('camera_odom_topic',
                               '/race_station_light/stonefish/dwe_camera/odometry')
        self.declare_parameter('camera_info_topic',
                               '/race_station_light/stonefish/dwe_camera/camera_info')
        self.declare_parameter('camera_image_topic',
                               '/race_station_light/stonefish/dwe_camera/image_color')
        self.declare_parameter('overlay_topic',
                               '/race_station_light/light_overlay')
        self.declare_parameter('light_in_camera_odom_topic',
                               '/race_station_light/light_in_camera/odometry')
        self.declare_parameter('light_in_camera_pixel_odom_topic',
                               '/race_station_light/light_in_camera/pixel_odometry')
        self.declare_parameter('light_in_camera_pixel_frame_id', 'image_pixel_plane')
        self.declare_parameter('light_in_camera_pixel_child_frame_id', 'light_pixel')
        self.declare_parameter('camera_frame_id', '')
        self.declare_parameter('light_frame_id', 'dwe_light')
        self.declare_parameter('axis_length', 0.5)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('origin_radius_px', 8)
        self.declare_parameter('origin_color_bgr', [0, 255, 255])
        self.declare_parameter('axis_x_color_bgr', [0, 0, 255])
        self.declare_parameter('axis_y_color_bgr', [0, 255, 0])
        self.declare_parameter('axis_z_color_bgr', [255, 0, 0])
        self.declare_parameter('axis_thickness_px', 3)
        self.declare_parameter('pixel_dot_radius_px', 5)
        self.declare_parameter('pixel_dot_color_bgr', [255, 255, 255])
        self.declare_parameter('pixel_dot_outline_thickness_px', 2)
        self.declare_parameter('pixel_dot_outline_color_bgr', [0, 0, 0])
        self.declare_parameter('pixel_dot_label', 'uv')
        self.declare_parameter('pixel_dot_label_scale', 0.5)
        self.declare_parameter('pixel_dot_label_thickness_px', 1)
        self.declare_parameter('sync_slop_sec', 0.05)
        self.declare_parameter('sync_queue_size', 10)
        self._bridge = CvBridge()
        self._frames_seen = 0
        self._frames_published = 0
        self._frames_overlaid = 0
        self._frames_image_skipped = 0
        self._frames_synced = 0
        self._light_in_cam_published = 0
        self._light_in_cam_pixel_published = 0
        self._dropped_no_odom = 0
        self._latest_light_odom: Optional[Odometry] = None
        self._latest_camera_odom: Optional[Odometry] = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._image_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('camera_image_topic').value,
            qos_profile=sensor_qos,
        )
        self._camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, self.get_parameter('camera_info_topic').value,
            qos_profile=info_qos,
        )

        self._light_odom_sub = self.create_subscription(
            Odometry, self.get_parameter('light_odom_topic').value,
            self._light_odom_cb, qos_profile=sensor_qos,
        )
        self._camera_odom_sub = self.create_subscription(
            Odometry, self.get_parameter('camera_odom_topic').value,
            self._camera_odom_cb, qos_profile=sensor_qos,
        )

        slop = float(self.get_parameter('sync_slop_sec').value)
        queue_size = int(self.get_parameter('sync_queue_size').value)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            (
                self._image_sub,
                self._camera_info_sub,
            ),
            queue_size,
            slop,
        )
        self._sync.registerCallback(self._on_synced)

        self._overlay_pub = self.create_publisher(
            CompressedImage, self.get_parameter('overlay_topic').value,
            pub_qos,
        )
        self._light_in_cam_odom_pub = self.create_publisher(
            Odometry, self.get_parameter('light_in_camera_odom_topic').value,
            pub_qos,
        )
        self._light_in_cam_pixel_pub = self.create_publisher(
            Odometry, self.get_parameter('light_in_camera_pixel_odom_topic').value,
            pub_qos,
        )

        self.get_logger().info(
            f"light_camera_overlay up: enabled={bool(self.get_parameter('enabled').value)}, "
            f"image<={self.get_parameter('camera_image_topic').value}, "
            f"info<={self.get_parameter('camera_info_topic').value}, "
            f"light_odom<={self.get_parameter('light_odom_topic').value} (latest snapshot), "
            f"camera_odom<={self.get_parameter('camera_odom_topic').value} (latest snapshot), "
            f"out=>{self.get_parameter('overlay_topic').value}, "
            f"light_in_cam_odom=>{self.get_parameter('light_in_camera_odom_topic').value}, "
            f"light_in_cam_pixel=>{self.get_parameter('light_in_camera_pixel_odom_topic').value}, "
            f"slop={slop:.3f}s queue={queue_size}"
        )

    def _light_odom_cb(self, msg: Odometry) -> None:
        self._latest_light_odom = msg

    def _camera_odom_cb(self, msg: Odometry) -> None:
        self._latest_camera_odom = msg

    def _on_synced(
        self,
        image_msg: Image,
        camera_info: CameraInfo,
    ) -> None:
        self._frames_seen += 1
        self._frames_synced += 1

        if self._latest_light_odom is None or self._latest_camera_odom is None:
            self._dropped_no_odom += 1
            self.get_logger().warn(
                "no odom received yet; overlay skipped",
                throttle_duration_sec=2.0,
            )
            return

        light_odom = self._latest_light_odom
        camera_odom = self._latest_camera_odom

        try:
            cv_image = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(
                f"cv_bridge failed ({image_msg.encoding}): {exc}",
                throttle_duration_sec=2.0,
            )
            return

        u_origin: Optional[Tuple[int, int]] = None
        if bool(self.get_parameter('enabled').value):
            K = _k_matrix(camera_info)
            drew_overlay = False
            if float(K[0, 0]) > 1e-6 and float(K[1, 1]) > 1e-6:
                try:
                    drew_overlay, u_origin = self._draw_overlay(
                        cv_image, light_odom, camera_odom, K,
                    )
                except Exception as exc:
                    self.get_logger().warn(
                        f"overlay draw failed: {exc}",
                        throttle_duration_sec=2.0,
                    )
            else:
                self.get_logger().warn(
                    "camera_info K is zero; overlay skipped",
                    throttle_duration_sec=5.0,
                )
            self._publish_compressed(cv_image, image_msg.header, drew_overlay)
        else:
            u_origin = self._project_light_origin(
                light_odom, camera_odom, _k_matrix(camera_info),
            )
            self._frames_image_skipped += 1

        self._publish_light_in_camera_odom(
            light_odom, camera_odom, camera_info, image_msg.header.stamp,
        )
        self._light_in_cam_published += 1
        self._publish_light_in_camera_pixel_odom(
            u_origin, image_msg.header.stamp,
        )

    def _project_light_origin(
        self,
        light_odom: Odometry,
        camera_odom: Odometry,
        K: np.ndarray,
    ) -> Optional[Tuple[int, int]]:
        if float(K[0, 0]) <= 1e-6 or float(K[1, 1]) <= 1e-6:
            return None
        T_world_light = _odom_to_T(light_odom)
        T_world_cam = _odom_to_T(camera_odom)
        T_cam_world = np.linalg.inv(T_world_cam)
        light_origin_world = T_world_light[:3, 3]
        p_origin_cam = T_cam_world @ np.array(
            [light_origin_world[0], light_origin_world[1],
             light_origin_world[2], 1.0]
        )
        return _project(p_origin_cam[:3], K)

    def _draw_overlay(
        self,
        img: np.ndarray,
        light_odom: Odometry,
        camera_odom: Odometry,
        K: np.ndarray,
    ) -> Tuple[bool, Optional[Tuple[int, int]]]:
        T_world_light = _odom_to_T(light_odom)
        T_world_cam = _odom_to_T(camera_odom)
        T_cam_world = np.linalg.inv(T_world_cam)

        light_origin_world = T_world_light[:3, 3]
        light_R_world = T_world_light[:3, :3]
        axis_length = float(self.get_parameter('axis_length').value)

        p_origin_cam = T_cam_world @ np.array(
            [light_origin_world[0], light_origin_world[1], light_origin_world[2], 1.0]
        )
        u_origin = _project(p_origin_cam[:3], K)

        origin_color = _bgr_param(self, 'origin_color_bgr', [0, 255, 255])
        axis_x_color = _bgr_param(self, 'axis_x_color_bgr', [0, 0, 255])
        axis_y_color = _bgr_param(self, 'axis_y_color_bgr', [0, 255, 0])
        axis_z_color = _bgr_param(self, 'axis_z_color_bgr', [255, 0, 0])
        radius = int(self.get_parameter('origin_radius_px').value)
        thickness = int(self.get_parameter('axis_thickness_px').value)

        dot_radius = int(self.get_parameter('pixel_dot_radius_px').value)
        dot_color = _bgr_param(self, 'pixel_dot_color_bgr', [255, 255, 255])
        dot_outline_color = _bgr_param(
            self, 'pixel_dot_outline_color_bgr', [0, 0, 0],
        )
        dot_outline_thickness = int(
            self.get_parameter('pixel_dot_outline_thickness_px').value
        )
        dot_label = self.get_parameter('pixel_dot_label').value
        dot_label_scale = float(
            self.get_parameter('pixel_dot_label_scale').value
        )
        dot_label_thickness = int(
            self.get_parameter('pixel_dot_label_thickness_px').value
        )

        drew = False
        if u_origin is not None:
            cv2.circle(img, u_origin, radius, origin_color, -1, lineType=cv2.LINE_AA)
            cv2.circle(img, u_origin, radius, (0, 0, 0), 1, lineType=cv2.LINE_AA)
            drew = True

            axes = (
                (light_R_world[:, 0], axis_x_color, 'X'),
                (light_R_world[:, 1], axis_y_color, 'Y'),
                (light_R_world[:, 2], axis_z_color, 'Z'),
            )
            for axis_dir_world, color, label in axes:
                tip_world = light_origin_world + axis_dir_world * axis_length
                p_tip_cam = T_cam_world @ np.array(
                    [tip_world[0], tip_world[1], tip_world[2], 1.0]
                )
                u_tip = _project(p_tip_cam[:3], K)
                if u_tip is None:
                    continue
                cv2.line(img, u_origin, u_tip, color, thickness, lineType=cv2.LINE_AA)
                cv2.circle(img, u_tip, max(2, radius // 2), color, -1, lineType=cv2.LINE_AA)
                cv2.putText(
                    img, label, (u_tip[0] + 4, u_tip[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
                )
                drew = True

            if dot_radius > 0:
                cv2.circle(
                    img, u_origin, dot_radius, dot_color, -1,
                    lineType=cv2.LINE_AA,
                )
                if dot_outline_thickness > 0:
                    cv2.circle(
                        img, u_origin, dot_radius, dot_outline_color,
                        dot_outline_thickness, lineType=cv2.LINE_AA,
                    )
                if dot_label:
                    cv2.putText(
                        img, dot_label,
                        (u_origin[0] + dot_radius + 4,
                         u_origin[1] - dot_radius - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, dot_label_scale,
                        dot_outline_color, dot_label_thickness + 1,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        img, dot_label,
                        (u_origin[0] + dot_radius + 4,
                         u_origin[1] - dot_radius - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, dot_label_scale,
                        dot_color, dot_label_thickness,
                        cv2.LINE_AA,
                    )
        return drew, u_origin

    def _publish_light_in_camera_odom(
        self,
        light_odom: Odometry,
        camera_odom: Odometry,
        camera_info: CameraInfo,
        image_stamp,
    ) -> None:
        T_world_light = _odom_to_T(light_odom)
        T_world_cam = _odom_to_T(camera_odom)
        T_cam_light = np.linalg.inv(T_world_cam) @ T_world_light

        p = T_cam_light[:3, 3]
        q = _matrix_to_quat(T_cam_light[:3, :3])

        cam_frame = self.get_parameter('camera_frame_id').value
        if not cam_frame and camera_info.header.frame_id:
            cam_frame = camera_info.header.frame_id
        if not cam_frame:
            cam_frame = 'camera_optical_frame'
        light_frame = self.get_parameter('light_frame_id').value or 'dwe_light'

        out = Odometry()
        out.header.stamp = image_stamp
        out.header.frame_id = cam_frame
        out.child_frame_id = light_frame
        out.pose.pose.position.x = float(p[0])
        out.pose.pose.position.y = float(p[1])
        out.pose.pose.position.z = float(p[2])
        out.pose.pose.orientation.x = float(q[0])
        out.pose.pose.orientation.y = float(q[1])
        out.pose.pose.orientation.z = float(q[2])
        out.pose.pose.orientation.w = float(q[3])
        out.pose.covariance = [0.0] * 36
        out.twist.covariance = [0.0] * 36
        self._light_in_cam_odom_pub.publish(out)

    def _publish_light_in_camera_pixel_odom(
        self,
        u_origin: Optional[Tuple[int, int]],
        image_stamp,
    ) -> None:
        if u_origin is None:
            return
        parent_frame = (
            self.get_parameter('light_in_camera_pixel_frame_id').value
            or 'image_pixel_plane'
        )
        child_frame = (
            self.get_parameter('light_in_camera_pixel_child_frame_id').value
            or 'light_pixel'
        )
        out = Odometry()
        out.header.stamp = image_stamp
        out.header.frame_id = parent_frame
        out.child_frame_id = child_frame
        out.pose.pose.position.x = float(u_origin[0])
        out.pose.pose.position.y = float(u_origin[1])
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.w = 1.0
        out.pose.covariance = [0.0] * 36
        out.twist.covariance = [0.0] * 36
        self._light_in_cam_pixel_pub.publish(out)
        self._light_in_cam_pixel_published += 1

    def _publish_compressed(self, img: np.ndarray, header, drew: bool) -> None:
        try:
            quality = int(self.get_parameter('jpeg_quality').value)
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            ok, buf = cv2.imencode('.jpg', img, encode_params)
            if not ok:
                self.get_logger().warn("cv2.imencode failed", throttle_duration_sec=2.0)
                return
            out = CompressedImage()
            out.header = header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self._overlay_pub.publish(out)
            self._frames_published += 1
            if drew:
                self._frames_overlaid += 1
            if self._frames_published == 1 or self._frames_published % 30 == 0:
                self.get_logger().info(
                    f"overlay stats: synced={self._frames_synced} "
                    f"seen={self._frames_seen} "
                    f"published={self._frames_published} "
                    f"overlaid={self._frames_overlaid} "
                    f"image_skipped={self._frames_image_skipped} "
                    f"light_in_cam_pub={self._light_in_cam_published} "
                    f"pixel_pub={self._light_in_cam_pixel_published} "
                    f"skipped_no_odom={self._dropped_no_odom}"
                )
        except Exception as exc:
            self.get_logger().warn(
                f"publish failed: {exc}", throttle_duration_sec=2.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LightCameraOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
