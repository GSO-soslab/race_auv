"""Stonefish AprilTag bridge node.

Subscribes to a Stonefish-simulated camera (``sensor_msgs/Image`` +
``sensor_msgs/CameraInfo``), runs AprilTag detection using the upstream
``dwe_camera_driver.apriltag_processor.AprilTagDetector``, and publishes:

* Annotated ``sensor_msgs/Image`` (``apriltag_detection/image``).
* ``vision_msgs/Detection3DArray`` (``apriltag_detection/detections3d``) with
  one entry per detected tag. ``Detection3D.id`` is ``f"{family}:{tag_id}"``;
  ``results[0].pose.pose`` is the tag's pose in the camera frame.
* TF: every detected tag as ``apriltag<N>`` child of the camera frame, plus
  the object's base frame in the camera frame, computed as the mean pose of
  the detected tags transformed through the URDF-supplied
  ``base -> tag`` transforms.

The object's base-link pose in the camera frame is *not* duplicated in
``Detection3DArray``; consumers can read it from TF.
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import (
    BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import (
    Pose, PoseWithCovariance, TransformStamped, Vector3,
)

from dwe_camera_driver.apriltag_processor import AprilTagDetector
from dwe_camera_driver.image_processing import ImageRectifier

from .urdf_tag_parser import extract_tag_transforms, TagTransform


def _yaml_to_dict(node: Node, key: str) -> dict:
    """Read a parameter that is a YAML path on disk and return its parsed dict.

    Falls back to reading the parameter as a string dict if the file path is
    empty or the file does not exist.
    """
    raw = node.get_parameter(key).value
    if isinstance(raw, dict):
        return raw
    if not raw:
        raise RuntimeError(f"Parameter '{key}' is empty")
    path = Path(str(raw)).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"YAML not found for '{key}': {path}")
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _sanitize_rotation(M: np.ndarray) -> np.ndarray:
    """Project a near-rotation 3x3 matrix onto SO(3) via SVD.

    pupil_apriltags can return a slightly non-orthogonal rotation when the
    tag is noisy or its pose is degenerate; the matrix's determinant may end
    up negative or zero, which makes ``scipy.Rotation.from_matrix`` raise
    ``ValueError: Non-positive determinant``. This helper returns the closest
    proper rotation so downstream quaternion conversion never crashes.
    """
    M = np.asarray(M, dtype=np.float64)
    U, _, Vt = np.linalg.svd(M)
    R_fixed = U @ Vt
    if np.linalg.det(R_fixed) < 0.0:
        Vt[-1, :] *= -1.0
        R_fixed = U @ Vt
    return R_fixed


def _is_bad_rotation(M: np.ndarray) -> bool:
    """Heuristic: this 3x3 is not a usable proper rotation.

    True if the matrix is non-finite, has det <= 0, or is far from
    orthogonal. Used to decide whether a detection's pose is trustworthy
    enough to be included in the joint base-pose solve.
    """
    M = np.asarray(M, dtype=np.float64)
    if not np.all(np.isfinite(M)):
        return True
    try:
        if np.linalg.det(M) <= 1e-6:
            return True
    except Exception:
        return True
    if not np.allclose(M @ M.T, np.eye(3), atol=1e-3):
        return True
    return False


def _matrix_to_pose_msg(T: np.ndarray) -> Pose:
    """Homogeneous 4x4 -> geometry_msgs/Pose. Robust to noisy rotations."""
    p = T[:3, 3]
    R_clean = _sanitize_rotation(T[:3, :3])
    q = R.from_matrix(R_clean).as_quat()  # xyzw
    msg = Pose()
    msg.position.x = float(p[0])
    msg.position.y = float(p[1])
    msg.position.z = float(p[2])
    msg.orientation.x = float(q[0])
    msg.orientation.y = float(q[1])
    msg.orientation.z = float(q[2])
    msg.orientation.w = float(q[3])
    return msg


def _matrix_to_transform_stamped(
    T: np.ndarray, parent: str, child: str, stamp
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    p = T[:3, 3]
    R_clean = _sanitize_rotation(T[:3, :3])
    q = R.from_matrix(R_clean).as_quat()
    msg.transform.translation.x = float(p[0])
    msg.transform.translation.y = float(p[1])
    msg.transform.translation.z = float(p[2])
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])
    return msg


def _average_poses(Ts: List[np.ndarray]) -> Optional[np.ndarray]:
    """Mean position + mean unit quaternion; returns None if input is empty."""
    if not Ts:
        return None
    positions = np.stack([T[:3, 3] for T in Ts], axis=0)
    quats = np.stack([
        R.from_matrix(_sanitize_rotation(T[:3, :3])).as_quat() for T in Ts
    ], axis=0)
    avg_pos = positions.mean(axis=0)
    avg_q = quats.mean(axis=0)
    n = np.linalg.norm(avg_q)
    if n < 1e-12:
        # Degenerate; fall back to identity rotation with averaged position.
        avg_q = np.array([0.0, 0.0, 0.0, 1.0])
    else:
        avg_q = avg_q / n
    T_avg = np.eye(4)
    T_avg[:3, :3] = R.from_quat(avg_q).as_matrix()
    T_avg[:3, 3] = avg_pos
    return T_avg


def _solve_cam_to_base(
    pairs: List[Tuple[np.ndarray, np.ndarray]],
) -> Optional[np.ndarray]:
    """Joint SE(3) solve: one rigid transform explaining all tag observations.

    Parameters
    ----------
    pairs
        List of ``(T_cam_to_tag_observed, T_base_to_tag_known)`` 4x4
        transforms. ``T_base_to_tag_known`` comes from the URDF.

    Returns
    -------
    np.ndarray or None
        The single ``T_cam_to_base`` that best satisfies
        ``T_cam_to_tag_i ≈ T_cam_to_base @ T_base_to_tag_i`` for every
        detected tag, in the least-squares sense. ``None`` if fewer than 3
        pairs are supplied (we need 3 non-collinear points for a unique
        SE(3) solution).

    Notes
    -----
    This is the classic Umeyama / ArUco ``estimatePoseBoard`` formulation:
    point correspondences are the tag centers in the base and camera frames,
    and we solve for the rigid transform that aligns them. Doing it jointly
    over all detected tags is essential when the tags sit at different
    orientations on the base, because averaging per-tag inverse solutions
    would mix inconsistent orientation estimates.
    """
    if len(pairs) < 3:
        return None

    P = np.stack([T_b[:3, 3] for _, T_b in pairs], axis=0)  # base-frame points
    Q = np.stack([T_c[:3, 3] for T_c, _ in pairs], axis=0)  # cam-frame points
    p_bar = P.mean(axis=0)
    q_bar = Q.mean(axis=0)
    Pc = P - p_bar
    Qc = Q - q_bar
    # Covariance H: minimizes ||Q_c - R P_c||_F^2 over R in SO(3).
    H = Qc.T @ Pc
    U, _, Vt = np.linalg.svd(H)
    R_sol = U @ Vt
    if np.linalg.det(R_sol) < 0.0:
        Vt[-1, :] *= -1.0
        R_sol = U @ Vt
    t_sol = q_bar - R_sol @ p_bar

    T = np.eye(4)
    T[:3, :3] = R_sol
    T[:3, 3] = t_sol
    return T


class StonefishAprilTagNode(Node):
    """Bridge node: Stonefish camera topics -> AprilTag detection + TF."""

    def __init__(self) -> None:
        super().__init__('stonefish_apriltag_node')

        # --- Parameters (declared up front so YAMLs can be loaded later) ---
        self.declare_parameter('config_yaml', '')
        self.declare_parameter('image_topic', '/race_auv/camera1/stonefish/data/image_color')
        self.declare_parameter('info_topic', '/race_auv/camera1/stonefish/data/camera_info')
        self.declare_parameter('output_image_topic', 'apriltag_detection/image')
        self.declare_parameter('output_detections_topic', 'apriltag_detection/detections3d')
        self.declare_parameter('tag_frame_prefix', 'apriltag')
        self.declare_parameter('object_base_frame', 'object_base')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('urdf_package', '')
        self.declare_parameter('urdf_filename', '')

        # --- QoS (Stonefish publishes BEST_EFFORT in some configs) ---
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
        self._tag_transforms: Dict[int, TagTransform] = {}
        self._rectifier: Optional[ImageRectifier] = None
        self._image_size: Optional[Tuple[int, int]] = None
        self._ready = False

        # --- Load YAML config (tags list, detector params, etc.) ---
        self._load_yaml_config()

        # --- Subscriptions / publishers ---
        self.image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._image_cb, sensor_qos,
        )
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self._info_cb, sensor_qos,
        )
        self.image_pub = self.create_publisher(
            Image, self.get_parameter('output_image_topic').value, out_qos,
        )
        self.detections_pub = self.create_publisher(
            Detection3DArray, self.get_parameter('output_detections_topic').value,
            out_qos,
        )

        # --- Timer ---
        rate = float(self.get_parameter('publish_rate').value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be > 0")
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"stonefish_apriltag_node ready: image_topic={self.image_sub.topic_name}, "
            f"info_topic={self.info_sub.topic_name}, rate={rate} Hz"
        )

    # ------------------------------------------------------------------ YAML
    def _load_yaml_config(self) -> None:
        cfg_path = self.get_parameter('config_yaml').value
        if not cfg_path:
            self.get_logger().warn(
                "No config_yaml provided; node will run with no tags configured."
            )
            return
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        # Allow either a top-level dict or a nested dict under ``apriltag:``.
        inner = cfg.get('apriltag', cfg)
        tags_cfg = inner.get('tags', []) or []
        obj_cfg = inner.get('object', {}) or {}
        det_cfg = inner.get('detector_defaults', {}) or {}

        # Build URDF path: absolute override > ROS param overrides on
        # package+filename > YAML values.
        urdf_override = self.get_parameter('urdf_path').value
        if urdf_override:
            urdf_path = str(urdf_override)
        else:
            # Apply ROS-param overrides to the YAML's object.urdf_* keys.
            pkg_override = self.get_parameter('urdf_package').value
            file_override = self.get_parameter('urdf_filename').value
            if pkg_override:
                obj_cfg = dict(obj_cfg)
                obj_cfg['urdf_package'] = str(pkg_override)
            if file_override:
                obj_cfg = dict(obj_cfg)
                obj_cfg['urdf_filename'] = str(file_override)
            urdf_path = self._resolve_urdf_path(obj_cfg)
        base_link = obj_cfg.get('base_link_name') or None
        prefix = obj_cfg.get('tag_link_prefix', 'apriltag')
        if not urdf_path:
            raise RuntimeError(
                "config_yaml needs 'object.urdf_package' + 'object.urdf_filename' "
                "(or override via the 'urdf_path' ROS param)"
            )
        tag_transforms = extract_tag_transforms(
            urdf_path, prefix=prefix, base_link_name=base_link
        )
        if not tag_transforms:
            raise RuntimeError(
                f"No '{prefix}<N>' links found in URDF {urdf_path} under base '{base_link}'"
            )
        self.get_logger().info(
            f"URDF resolved: {urdf_path} ({len(tag_transforms)} tag links)"
        )
        # Surface the per-tag T_base_to_tag so it is obvious when a tag is
        # fixed at the base origin (its TF in the camera frame will then be
        # numerically equal to the broadcast object_base TF).
        for tid in sorted(tag_transforms):
            tt = tag_transforms[tid]
            t = tt.T_base_to_tag[:3, 3]
            self.get_logger().info(
                f"  apriltag{tid}: T_base_to_tag xyz=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})"
            )
        # Warn when two or more tags share the same base-frame pose: their
        # published apriltag<N> TFs will be visually identical in RViz, and
        # the joint Umeyama solve becomes degenerate (the points are
        # coincident in the base frame). Usually this means the URDF has
        # `<origin xyz="0 0 0"/>` on multiple tag joints -- the fix is to
        # give every tag a distinct origin in the URDF.
        by_pose: Dict[Tuple[float, float, float], List[int]] = {}
        for tid, tt in tag_transforms.items():
            t = tuple(np.round(tt.T_base_to_tag[:3, 3], 6).tolist())
            by_pose.setdefault(t, []).append(tid)
        for t, ids in by_pose.items():
            if len(ids) > 1:
                self.get_logger().warn(
                    f"Tags {ids} share the same T_base_to_tag {t}; their TFs "
                    f"will overlap and the joint base-pose solve is degenerate. "
                    f"Give each tag a distinct <origin> in the URDF."
                )
        self._tag_transforms = tag_transforms

        # Cache detector params from YAML; detector construction is deferred
        # until the first CameraInfo arrives (we need K, D, W, H).
        self._detector_params_template = {
            'nthreads': int(det_cfg.get('nthreads', 2)),
            'quad_decimate': float(det_cfg.get('quad_decimate', 1.0)),
            'quad_sigma': float(det_cfg.get('quad_sigma', 0.0)),
            'refine_edges': bool(det_cfg.get('refine_edges', True)),
            'decode_sharpening': float(det_cfg.get('decode_sharpening', 0.25)),
        }
        self._tags_config: List[Dict] = []
        for entry in tags_cfg:
            tag_id = int(entry['id'])
            family = str(entry.get('family', 'tag36h11'))
            size = float(entry['size'])
            if tag_id not in tag_transforms:
                self.get_logger().warn(
                    f"Tag id {tag_id} listed in YAML but not present in URDF; skipping."
                )
                continue
            self._tags_config.append(
                {'id': tag_id, 'family': family, 'size': size}
            )
        if not self._tags_config:
            raise RuntimeError("No valid tag entries in config_yaml after URDF check.")
        self.get_logger().info(
            f"Configured {len(self._tags_config)} tags from {len(tag_transforms)} URDF tag links."
        )

    # ----------------------------------------------------------------- Callbacks
    def _image_cb(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return
        with self._lock:
            self._latest_bgr = bgr
            self._latest_stamp = msg.header.stamp

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._ready:
            return
        self._info_msg = msg
        self._build_detectors(msg)
        self._ready = True
        self.get_logger().info(
            f"CameraInfo received; {len(self._detectors)} detectors ready."
        )

    # ----------------------------------------------------------- Detector pool
    def _build_detectors(self, info: CameraInfo) -> None:
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        width = int(info.width)
        height = int(info.height)
        d = list(info.d) if info.d else [0.0] * 5
        if any(abs(x) > 1e-9 for x in d):
            self.get_logger().warn(
                f"Non-zero distortion coefficients {d}; passing to standard plumb-bob model."
            )

        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        D = np.array(d[:5], dtype=np.float32)

        self._rectifier = ImageRectifier(
            logger=self.get_logger(),
            camera_matrix=K,
            dist_coeffs=D,
            image_size=(width, height),
            is_fisheye=False,
            crop_to_valid_pixels=False,
        )
        new_K = self._rectifier.get_new_camera_params()
        new_size = self._rectifier.get_new_image_size()
        new_D = self._rectifier.get_new_distortion_coeffs()

        self._image_size = (new_size['img_width'], new_size['img_height'])

        for tag in self._tags_config:
            key = (tag['family'], tag['id'])
            child_logger = self.get_logger().get_child(
                f"det_{tag['family']}_{tag['id']}"
            )
            self._detectors[key] = AprilTagDetector(
                family=tag['family'],
                tag_size=tag['size'],
                camera_intrinsics=new_K,
                camera_distortion=new_D.tolist(),
                image_size=new_size,
                logger=child_logger,
                detector_params=dict(self._detector_params_template),
            )
            # The upstream AprilTagDetector.detect_and_draw emits a per-tick
            # WARN every time pupil_apriltags returns a slightly non-right-
            # handed rotation (noisy / degenerate corners). Our pose pipeline
            # already recovers from that via the SVD sanitizer, so the warning
            # is purely cosmetic and very chatty. Drop it on the child
            # logger; we still log a periodic summary on the parent below.
            try:
                child_logger.set_level(
                    rclpy.logging.LoggingSeverity.ERROR
                )
            except Exception:
                pass
        self._bad_pose_count = 0

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
        detections_msg.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        detections_msg.header.frame_id = self._camera_frame_id()

        # detected_tags: (family, id, T_cam_to_tag, was_bad)
        #   was_bad=True means the original pose_R was non-right-handed /
        #   non-finite. We still publish its per-tag TF (after SVD fix-up)
        #   and include it in the Detection3DArray, but we EXCLUDE it from
        #   the joint base-pose solve, because a noisy false-positive would
        #   otherwise drag the solved base pose around.
        detected_tags: List[Tuple[str, int, np.ndarray, bool]] = []
        for (family, tag_id), detector in self._detectors.items():
            # Pose collection pass
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
                    # If the post-sanitize R is still unusable, drop it.
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

            # Drawing pass (uses the same detector to keep the visual style;
            # upstream already guards against bad rotations internally).
            try:
                detector.detect_and_draw(work)
            except Exception as e:
                self.get_logger().warn(
                    f"Detector ({family}, {tag_id}) draw failed: {e}"
                )

        # Publish annotated image
        try:
            img_msg = self._bridge.cv2_to_imgmsg(work, encoding='bgr8')
            img_msg.header = detections_msg.header
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

        # Build Detection3DArray (all detected tags, including bad-pose
        # ones, since we still have a sanitized pose for them).
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
            det3d.bbox.center = Pose()  # zero; object base goes to TF
            det3d.bbox.size = Vector3()  # unknown from a single tag
            detections_msg.detections.append(det3d)
        self.detections_pub.publish(detections_msg)

        # TF: per-tag (all detections) + joint-solved object base
        # (good-pose detections only).
        now = self.get_clock().now().to_msg()
        for family, tag_id, T_cam_to_tag, _was_bad in detected_tags:
            child = f"{self.get_parameter('tag_frame_prefix').value}{tag_id}"
            self._tf.sendTransform(
                _matrix_to_transform_stamped(T_cam_to_tag, cam_frame, child, now)
            )

        # Restrict the joint base-pose solve to detections whose original
        # pose was a proper rotation. Bad-pose detections (e.g. noisy
        # false positives that yielded a non-right-handed matrix) get a
        # sanitized per-tag TF above but are NOT allowed to drag the
        # solved base pose around.
        # Each entry: (T_cam_to_tag, T_base_to_tag, tag_id)
        good_pairs = [
            (T_cam_to_tag, self._tag_transforms[tag_id].T_base_to_tag, tag_id)
            for _family, tag_id, T_cam_to_tag, was_bad in detected_tags
            if not was_bad
        ]
        if len(good_pairs) >= 3:
            # Joint SE(3) solve over all good-pose tag centers -- this is
            # the right answer when tags are mounted at different
            # orientations on the base link, because a single rigid
            # transform must explain them all simultaneously (Umeyama /
            # ArUco estimatePoseBoard).
            T_cam_to_base = _solve_cam_to_base(
                [(T_c, T_b) for T_c, T_b, _ in good_pairs]
            )
            if T_cam_to_base is not None:
                base_frame = self.get_parameter('object_base_frame').value
                self._tf.sendTransform(
                    _matrix_to_transform_stamped(
                        T_cam_to_base, cam_frame, base_frame, now
                    )
                )
        elif len(good_pairs) == 1:
            T_cam_to_tag, T_base_to_tag, _tag_id = good_pairs[0]
            T_cam_to_base = T_cam_to_tag @ np.linalg.inv(T_base_to_tag)
            base_frame = self.get_parameter('object_base_frame').value
            self._tf.sendTransform(
                _matrix_to_transform_stamped(
                    T_cam_to_base, cam_frame, base_frame, now
                )
            )

    def _camera_frame_id(self) -> str:
        override = self.get_parameter('camera_frame').value
        if override:
            return str(override)
        if self._info_msg is not None and self._info_msg.header.frame_id:
            return self._info_msg.header.frame_id
        return "camera_optical_frame"

    def _resolve_urdf_path(self, obj_cfg: dict) -> str:
        """Resolve a URDF path from ``object.urdf_package`` + ``object.urdf_filename``.

        Equivalent to ``description.launch.py``'s ``get_package_share_directory`` pattern.
        Returns an empty string if the package name is missing.
        """
        pkg = obj_cfg.get('urdf_package') or obj_cfg.get('urdf_path_pkg')
        rel = obj_cfg.get('urdf_filename') or obj_cfg.get('urdf_path_in_pkg')
        if not pkg:
            return ''
        if not rel:
            rel = 'urdf/base.urdf'
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory(str(pkg))
        except Exception as e:
            raise RuntimeError(
                f"Could not resolve ROS package '{pkg}' for URDF lookup: {e}"
            )
        return str(Path(share) / rel)


def main() -> None:
    rclpy.init()
    node = StonefishAprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
