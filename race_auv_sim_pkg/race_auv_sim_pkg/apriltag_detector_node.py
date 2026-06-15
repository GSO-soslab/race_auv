"""Per-camera AprilTag detector node.

A single instance of this node handles one camera. It subscribes to either a
raw ``sensor_msgs/Image`` or a ``sensor_msgs/CompressedImage`` topic, plus an
optional ``sensor_msgs/CameraInfo`` topic, runs the upstream
``dwe_camera_driver.apriltag_processor.AprilTagDetector`` on each image, and
publishes:

* ``vision_msgs/Detection3DArray`` on ``output_detections_topic`` with one
  entry per detected tag. ``Detection3D.id`` is ``f"{family}:{tag_id}"`` and
  ``results[0].pose.pose`` is the tag's pose in the camera frame.
* An annotated ``sensor_msgs/Image`` on ``output_image_topic``.
* TF: every detected tag as ``tag_frame_prefix<tag_id>`` child of the camera
  optical frame.

This node **never** publishes ``object_base``. The fused
``reference_frame -> object_base`` TF is owned by
``apriltag_fuser_node`` so multi-camera setups do not fight over the frame.

When ``info_topic`` is empty, the node falls back to intrinsics supplied via
ROS parameters (typically populated by the launch file from
``config/apriltag.yaml``). This is the path used for the DWE camera driver,
which does not publish ``CameraInfo`` by default.
"""

from __future__ import annotations

import threading
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import (
    BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import Pose, PoseWithCovariance, Vector3

from dwe_camera_driver.apriltag_processor import AprilTagDetector
from dwe_camera_driver.image_processing import ImageRectifier

from .apriltag_geom import (
    is_bad_rotation as _is_bad_rotation,
    load_yaml_config,
    matrix_to_pose_msg as _matrix_to_pose_msg,
    matrix_to_transform_stamped as _matrix_to_transform_stamped,
    sanitize_rotation as _sanitize_rotation,
)


def _as_float(v, default: float = 0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _as_int(v, default: int = 0) -> int:
    try:
        return int(v)
    except (TypeError, ValueError):
        return default


def _as_bool(v, default: bool = False) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes", "on")
    try:
        return bool(v)
    except (TypeError, ValueError):
        return default


class AprilTagDetectorNode(Node):
    """Per-camera detector: image (+ optional CameraInfo) -> detections + TF."""

    def __init__(self) -> None:
        super().__init__("apriltag_detector")

        # --- Parameters ---
        self.declare_parameter("config_yaml", "")
        self.declare_parameter("image_transport", "raw")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("info_topic", "")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("tag_frame_prefix", "apriltag")
        self.declare_parameter("output_image_topic", "apriltag_detection/image")
        self.declare_parameter("output_detections_topic", "apriltag_detection/detections3d")
        self.declare_parameter("publish_rate", 5.0)

        # Intrinsics fallback (used when info_topic is empty).
        self.declare_parameter("intrinsics.fx", 0.0)
        self.declare_parameter("intrinsics.fy", 0.0)
        self.declare_parameter("intrinsics.cx", 0.0)
        self.declare_parameter("intrinsics.cy", 0.0)
        self.declare_parameter("intrinsics.width", 0)
        self.declare_parameter("intrinsics.height", 0)
        self.declare_parameter("intrinsics.distortion", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("intrinsics.fisheye", False)

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._bridge = CvBridge()
        self._tf = TransformBroadcaster(self)

        # --- State ---
        self._lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_stamp = None
        self._info_msg: Optional[CameraInfo] = None
        self._detectors: Dict[Tuple[str, int], AprilTagDetector] = {}
        self._rectifier: Optional[ImageRectifier] = None
        self._image_size: Optional[Tuple[int, int]] = None
        self._ready = False
        self._bad_pose_count = 0

        # --- Load tag list + detector defaults from YAML ---
        self._tags_config: List[Dict] = []
        self._detector_params_template: Dict = {}
        self._load_yaml_config()

        # --- Subscriptions / publishers ---
        transport = str(self.get_parameter("image_transport").value or "raw").lower()
        image_topic = str(self.get_parameter("image_topic").value or "")
        info_topic = str(self.get_parameter("info_topic").value or "")
        if not image_topic:
            raise RuntimeError("image_topic parameter is required")

        if transport == "compressed":
            self.image_sub = self.create_subscription(
                CompressedImage, image_topic, self._image_cb_compressed, sensor_qos,
            )
        else:
            self.image_sub = self.create_subscription(
                Image, image_topic, self._image_cb_raw, sensor_qos,
            )

        if info_topic:
            self.info_sub = self.create_subscription(
                CameraInfo, info_topic, self._info_cb, sensor_qos,
            )
        else:
            self.info_sub = None
            # No CameraInfo: build detectors from YAML intrinsics immediately.
            self._try_build_from_yaml_intrinsics()

        self.image_pub = self.create_publisher(
            Image, self.get_parameter("output_image_topic").value, out_qos,
        )
        self.detections_pub = self.create_publisher(
            Detection3DArray, self.get_parameter("output_detections_topic").value, out_qos,
        )

        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be > 0")
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"apriltag_detector ready: image_topic={image_topic} "
            f"info_topic={info_topic or '<yaml intrinsics>'} "
            f"transport={transport} rate={rate} Hz "
            f"tags={len(self._tags_config)}"
        )

    # ------------------------------------------------------------------ YAML
    def _load_yaml_config(self) -> None:
        cfg_path = str(self.get_parameter("config_yaml").value or "")
        if not cfg_path:
            self.get_logger().warn(
                "No config_yaml provided; node will run with no tags configured."
            )
            return
        inner = load_yaml_config(cfg_path)
        tags_cfg = inner.get("tags", []) or []
        det_cfg = inner.get("detector_defaults", {}) or {}

        self._detector_params_template = {
            "nthreads": int(det_cfg.get("nthreads", 2)),
            "quad_decimate": float(det_cfg.get("quad_decimate", 1.0)),
            "quad_sigma": float(det_cfg.get("quad_sigma", 0.0)),
            "refine_edges": bool(det_cfg.get("refine_edges", True)),
            "decode_sharpening": float(det_cfg.get("decode_sharpening", 0.25)),
        }
        for entry in tags_cfg:
            try:
                tag_id = int(entry["id"])
                family = str(entry.get("family", "tag36h11"))
                size = float(entry["size"])
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().warn(f"Skipping malformed tag entry {entry}: {e}")
                continue
            self._tags_config.append({"id": tag_id, "family": family, "size": size})
        if not self._tags_config:
            raise RuntimeError("No valid tag entries in config_yaml.")
        self.get_logger().info(
            f"Configured {len(self._tags_config)} tags from {cfg_path}."
        )

    # ---------------------------------------------------------------- Intrinsics
    def _yaml_intrinsics(self) -> Optional[Tuple[float, float, float, float, int, int, list, bool]]:
        fx = _as_float(self.get_parameter("intrinsics.fx").value)
        fy = _as_float(self.get_parameter("intrinsics.fy").value)
        cx = _as_float(self.get_parameter("intrinsics.cx").value)
        cy = _as_float(self.get_parameter("intrinsics.cy").value)
        width = _as_int(self.get_parameter("intrinsics.width").value)
        height = _as_int(self.get_parameter("intrinsics.height").value)
        distortion = self.get_parameter("intrinsics.distortion").value or [0.0] * 5
        fisheye = _as_bool(self.get_parameter("intrinsics.fisheye").value)
        if fx <= 0.0 or fy <= 0.0 or width <= 0 or height <= 0:
            return None
        if not isinstance(distortion, (list, tuple)):
            return None
        d = [float(x) for x in distortion][:5]
        while len(d) < 5:
            d.append(0.0)
        return fx, fy, cx, cy, width, height, d, fisheye

    def _try_build_from_yaml_intrinsics(self) -> None:
        cfg = self._yaml_intrinsics()
        if cfg is None:
            self.get_logger().warn(
                "No info_topic and no usable YAML intrinsics; detector will "
                "wait for a CameraInfo message to build its pipeline."
            )
            return
        fx, fy, cx, cy, width, height, d, fisheye = cfg
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        D = np.array(d[:5], dtype=np.float32)
        self.get_logger().info(
            f"Building detectors from YAML intrinsics: {width}x{height} "
            f"fx={fx} fy={fy} cx={cx} cy={cy} fisheye={fisheye}"
        )
        self._build_detectors(K, D, width, height, is_fisheye=fisheye)
        self._ready = True

    # ----------------------------------------------------------------- Callbacks
    def _image_cb_raw(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return
        with self._lock:
            self._latest_bgr = bgr
            self._latest_stamp = msg.header.stamp

    def _image_cb_compressed(self, msg: CompressedImage) -> None:
        try:
            bgr = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge compressed failed: {e}")
            return
        with self._lock:
            self._latest_bgr = bgr
            self._latest_stamp = msg.header.stamp

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._ready:
            return
        self._info_msg = msg
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        width = int(msg.width)
        height = int(msg.height)
        d = list(msg.d) if msg.d else [0.0] * 5
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        D = np.array(d[:5], dtype=np.float32)
        if any(abs(x) > 1e-9 for x in d):
            self.get_logger().warn(
                f"Non-zero distortion coefficients {d}; using plumb-bob model."
            )
        self._build_detectors(K, D, width, height, is_fisheye=False)
        self._ready = True
        self.get_logger().info(
            f"CameraInfo received; {len(self._detectors)} detectors ready."
        )

    # ----------------------------------------------------------- Detector pool
    def _build_detectors(
        self,
        K: np.ndarray,
        D: np.ndarray,
        width: int,
        height: int,
        is_fisheye: bool,
    ) -> None:
        self._rectifier = ImageRectifier(
            logger=self.get_logger(),
            camera_matrix=K,
            dist_coeffs=D,
            image_size=(width, height),
            is_fisheye=is_fisheye,
            crop_to_valid_pixels=False,
        )
        new_K = self._rectifier.get_new_camera_params()
        new_size = self._rectifier.get_new_image_size()
        new_D = self._rectifier.get_new_distortion_coeffs()

        self._image_size = (new_size["img_width"], new_size["img_height"])

        for tag in self._tags_config:
            key = (tag["family"], tag["id"])
            child_logger = self.get_logger().get_child(
                f"det_{tag['family']}_{tag['id']}"
            )
            self._detectors[key] = AprilTagDetector(
                family=tag["family"],
                tag_size=tag["size"],
                camera_intrinsics=new_K,
                camera_distortion=new_D.tolist(),
                image_size=new_size,
                logger=child_logger,
                detector_params=dict(self._detector_params_template),
            )
            try:
                child_logger.set_level(rclpy.logging.LoggingSeverity.ERROR)
            except Exception:
                pass

    # ------------------------------------------------------------------- Tick
    def _tick(self) -> None:
        if not self._ready:
            return
        with self._lock:
            bgr = self._latest_bgr
            stamp = self._latest_stamp
        if bgr is None:
            return

        if self._rectifier is not None:
            work = self._rectifier.rectify(bgr)
        else:
            work = bgr
        gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)

        detections_msg = Detection3DArray()
        detections_msg.header.stamp = (
            stamp if stamp is not None else self.get_clock().now().to_msg()
        )
        detections_msg.header.frame_id = self._camera_frame_id()

        detected_tags: List[Tuple[str, int, np.ndarray, bool]] = []
        for (family, tag_id), detector in self._detectors.items():
            try:
                raw = detector.detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=detector.camera_params,
                    tag_size=detector.tag_size,
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Detector ({family}, {tag_id}) failed: {e}; skipping this tick."
                )
                continue
            for d in raw:
                if int(d.tag_id) != int(tag_id):
                    continue
                try:
                    T = np.eye(4)
                    T[:3, :3] = d.pose_R
                    T[:3, 3] = d.pose_t.flatten()
                    was_bad = _is_bad_rotation(T[:3, :3])
                    T[:3, :3] = _sanitize_rotation(T[:3, :3])
                    if not np.all(np.isfinite(T[:3, :3])) or \
                            np.linalg.det(T[:3, :3]) < 0.5:
                        raise ValueError("rotation unrecoverable after SVD")
                    detected_tags.append((family, tag_id, T, was_bad))
                except Exception as e:
                    self._bad_pose_count += 1
                    if self._bad_pose_count in (1, 10, 100, 1000) or (
                        self._bad_pose_count % 1000 == 0
                    ):
                        self.get_logger().warn(
                            f"Skipped tag {family}:{tag_id} with bad pose "
                            f"(total bad-rotation drops: {self._bad_pose_count}): {e}"
                        )

            try:
                detector.detect_and_draw(work)
            except Exception as e:
                self.get_logger().warn(
                    f"Detector ({family}, {tag_id}) draw failed: {e}"
                )

        try:
            img_msg = self._bridge.cv2_to_imgmsg(work, encoding="bgr8")
            img_msg.header = detections_msg.header
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

        cam_frame = self._camera_frame_id()
        for family, tag_id, T_cam_to_tag, _was_bad in detected_tags:
            det3d = Detection3D()
            det3d.header = detections_msg.header
            det3d.id = f"{family}:{tag_id}"
            hyp = ObjectHypothesis(class_id=str(tag_id), score=1.0)
            pwp = ObjectHypothesisWithPose(hypothesis=hyp)
            pwp.pose = PoseWithCovariance()
            pwp.pose.pose = _matrix_to_pose_msg(T_cam_to_tag)
            det3d.results = [pwp]
            det3d.bbox = BoundingBox3D()
            det3d.bbox.center = Pose()
            det3d.bbox.size = Vector3()
            detections_msg.detections.append(det3d)
        self.detections_pub.publish(detections_msg)

        now = self.get_clock().now().to_msg()
        prefix = str(self.get_parameter("tag_frame_prefix").value)
        for family, tag_id, T_cam_to_tag, _was_bad in detected_tags:
            child = f"{prefix}{tag_id}"
            self._tf.sendTransform(
                _matrix_to_transform_stamped(T_cam_to_tag, cam_frame, child, now)
            )

    def _camera_frame_id(self) -> str:
        override = str(self.get_parameter("camera_frame").value or "")
        if override:
            return override
        if self._info_msg is not None and self._info_msg.header.frame_id:
            return self._info_msg.header.frame_id
        return "camera_optical_frame"


def main() -> None:
    rclpy.init()
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
