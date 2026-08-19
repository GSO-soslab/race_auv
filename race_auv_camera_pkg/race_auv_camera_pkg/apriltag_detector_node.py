"""Per-camera AprilTag detector node (no TF, Jetson-tuned).

A single instance of this node handles one camera. It subscribes to
either a raw ``sensor_msgs/Image`` or a ``sensor_msgs/CompressedImage``
topic, plus an optional ``sensor_msgs/CameraInfo`` topic, runs the
``apriltag_processor.AprilTagDetector`` on each frame, and publishes:

* ``vision_msgs/Detection3DArray`` on ``output_detections_topic`` with
  one entry per detected tag. ``Detection3D.id`` is
  ``f"{family}:{tag_id}"`` and ``results[0].pose.pose`` is the tag's
  pose in the camera frame.
* An annotated ``sensor_msgs/CompressedImage`` on
  ``output_image_topic``: tag bounding boxes + id labels + pose
  overlays, plus a red crosshair at the image center. This is the
  image shown by Foxglove / topside.

This node does NOT publish any TF. The original pipeline in
``race_auv_sim_pkg/apriltag_detector_node.py`` broadcast per-tag frames
(``apriltag<family>_<id>``) for the multi-camera fuser; that
responsibility is dropped here: annotated images + ``Detection3DArray``
topics are the only outputs.

Multi-family / multi-size tags
------------------------------
Tags are bucketed by ``(family, tag_size)`` and one
``AprilTagDetector`` is created per bucket because
``pupil_apriltags.Detector`` only supports a single ``tag_size`` per
``detect()`` call.

Two sources for the tag list:

* ``tags_override`` (JSON-encoded list of ``{id, family, size}``
  dicts) -- set by the per-camera launch file from ``apriltag.yaml``.
  When provided and non-empty, this list drives the detector and the
  YAML ``tags:`` list is ignored.
* Otherwise the YAML ``tags:`` list (the global fallback) is used.

Jetson / Orin performance knobs
------------------------------
All hardware-acceleration is configured in ``apriltag.yaml`` under
``detector_defaults:`` -- no environment variables.

* ``use_cuda`` (bool, default false) -- master switch for GPU
  acceleration. When true, the rectifier tries ``cv2.cuda.remap`` and
  the JPEG encoder tries hardware paths. Falls back to CPU gracefully
  on any failure.
* ``jpeg_backend`` (one of ``"auto"``, ``"nvjpeg"``, ``"cuda"``,
  ``"cpu"``, default ``"auto"``) -- explicit selection of the
  annotated-frame JPEG encoder. ``auto`` with ``use_cuda=true`` tries
  pyNvJPEG (NVIDIA nvjpeg hardware), then ``cv2.cuda.encodeJpeg``,
  then CPU. ``auto`` with ``use_cuda=false`` is CPU.
* ``process_scale`` (per-camera, default 1.0) -- detect on a
  downscaled image; annotation stays at full resolution. 0.5 gives
  ~3-5x faster detection on Orin with negligible accuracy loss for
  dock-sized tags.
* ``jpeg_quality`` (per-camera, default 80) -- quality for the
  annotated ``CompressedImage`` (Foxglove bandwidth knob).

When ``info_topic`` is empty, the node falls back to intrinsics
supplied via ROS parameters (typically populated by the launch file
from ``apriltag.yaml``). This is the path used for the DWE camera
driver, which does not publish ``CameraInfo`` by default.
"""

from __future__ import annotations

import json
import threading
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseWithCovariance, Vector3
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from vision_msgs.msg import (
    BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from .apriltag_geom import (
    is_bad_rotation, load_yaml_config, matrix_to_pose_msg, sanitize_rotation,
)
from .apriltag_processor import AprilTagDetector
from .image_jpeg import build_jpeg_encoder
from .image_processing import build_rectifier


# --- Module-level QoS profiles -------------------------------------------------
# Subscriptions: best-effort, low depth. The detector always works on the
# latest frame; queued frames are wasted CPU.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Publications: reliable, low depth. foxglove_bridge relays them as-is.
_OUTPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# Crosshair drawing constants (red, AA lines).
_CROSSHAIR_COLOR = (0, 0, 255)
_CROSSHAIR_THICKNESS = 2
_CROSSHAIR_ARM_DIVISOR = 15


def _as_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _as_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except (TypeError, ValueError):
        return default


def _draw_crosshair(image: np.ndarray) -> None:
    """Draw a small red cross at the center of ``image`` (in-place)."""
    h, w = image.shape[:2]
    cx, cy = w // 2, h // 2
    arm = max(1, min(h, w) // _CROSSHAIR_ARM_DIVISOR)
    cv2.line(image, (cx - arm, cy), (cx + arm, cy),
             _CROSSHAIR_COLOR, _CROSSHAIR_THICKNESS, cv2.LINE_AA)
    cv2.line(image, (cx, cy - arm), (cx, cy + arm),
             _CROSSHAIR_COLOR, _CROSSHAIR_THICKNESS, cv2.LINE_AA)


class AprilTagDetectorNode(Node):
    """Per-camera detector: image (+ optional CameraInfo) -> detections + image."""

    def __init__(self) -> None:
        super().__init__("apriltag_detector")

        # -------------------------------------------------------------- params
        # YAML config path (set by the launch file).
        self.declare_parameter("config_yaml", "")

        # Transport / topics.
        self.declare_parameter("image_transport", "raw")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("info_topic", "")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("output_image_topic", "apriltag_detection/image")
        self.declare_parameter("output_detections_topic", "apriltag_detection/detections3d")
        self.declare_parameter("publish_rate", 5.0)

        # Tag list: per-camera JSON override (set by launch), else YAML tags.
        self.declare_parameter("tags_override", "[]")

        # Jetson perf knobs (per-camera).
        self.declare_parameter("process_scale", 1.0)
        self.declare_parameter("jpeg_quality", 80)

        # Intrinsics fallback (used when info_topic is empty).
        self.declare_parameter("intrinsics.fx", 0.0)
        self.declare_parameter("intrinsics.fy", 0.0)
        self.declare_parameter("intrinsics.cx", 0.0)
        self.declare_parameter("intrinsics.cy", 0.0)
        self.declare_parameter("intrinsics.width", 0)
        self.declare_parameter("intrinsics.height", 0)
        self.declare_parameter("intrinsics.distortion", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("intrinsics.fisheye", False)

        # --------------------------------------------------------------- state
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_stamp = None
        self._info_msg: Optional[CameraInfo] = None
        # Keyed by (family, tag_size). One detector per group because
        # pupil_apriltags.Detector only supports a single tag_size per call.
        self._detectors: Dict[Tuple[str, float], AprilTagDetector] = {}
        self._rectifier: Optional[object] = None
        self._rectifier_backend: str = "cpu"
        self._jpeg_encoder: Optional[object] = None
        self._jpeg_backend: str = "cpu"
        self._jpeg_backend_requested: str = "cpu"
        self._process_scale: float = 1.0
        self._ready = False
        self._bad_pose_count = 0
        self._use_cuda = False

        # Load tag list + detector defaults from YAML.
        self._tags_config: List[Dict] = []
        self._detector_params_template: Dict = {}
        self._load_yaml_config()

        # -------------------------------------------------------- subscriptions
        transport = str(self.get_parameter("image_transport").value or "raw").lower()
        image_topic = str(self.get_parameter("image_topic").value or "")
        info_topic = str(self.get_parameter("info_topic").value or "")
        if not image_topic:
            raise RuntimeError("image_topic parameter is required")

        image_type = CompressedImage if transport == "compressed" else Image
        image_cb = self._image_cb_compressed if transport == "compressed" else self._image_cb_raw
        self.image_sub = self.create_subscription(image_type, image_topic, image_cb, _SENSOR_QOS)

        if info_topic:
            self.info_sub = self.create_subscription(CameraInfo, info_topic, self._info_cb, _SENSOR_QOS)
        else:
            self.info_sub = None
            self._try_build_from_yaml_intrinsics()

        # --------------------------------------------------------- publications
        self.image_pub = self.create_publisher(
            CompressedImage, self.get_parameter("output_image_topic").value, _OUTPUT_QOS,
        )
        self.detections_pub = self.create_publisher(
            Detection3DArray, self.get_parameter("output_detections_topic").value, _OUTPUT_QOS,
        )

        # --------------------------------------------------------- tick / init
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be > 0")
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"apriltag_detector ready: image={image_topic} "
            f"info={info_topic or '<yaml intrinsics>'} transport={transport} "
            f"rate={rate}Hz tags={len(self._tags_config)} groups={len(self._detectors)}"
        )

    # =====================================================================
    # YAML / configuration
    # =====================================================================
    def _load_yaml_config(self) -> None:
        """Populate ``_tags_config`` and ``_detector_params_template``.

        ``tags_override`` (a JSON-encoded list of ``{id, family, size}``)
        takes precedence when non-empty; otherwise the YAML ``tags:``
        list is used. Detector defaults always come from the YAML.
        """
        cfg_path = str(self.get_parameter("config_yaml").value or "")
        inner: Dict = {}
        det_cfg: Dict = {}
        if cfg_path:
            inner = load_yaml_config(cfg_path)
            det_cfg = inner.get("detector_defaults", {}) or {}

        self._detector_params_template = {
            "nthreads": int(det_cfg.get("nthreads", 4)),
            "quad_decimate": float(det_cfg.get("quad_decimate", 2.0)),
            "quad_sigma": float(det_cfg.get("quad_sigma", 0.0)),
            "refine_edges": bool(det_cfg.get("refine_edges", True)),
            "decode_sharpening": float(det_cfg.get("decode_sharpening", 0.25)),
        }

        # HW-accel knobs (package-level, read from detector_defaults).
        self._use_cuda = bool(det_cfg.get("use_cuda", False))
        self._jpeg_backend_requested = str(det_cfg.get("jpeg_backend", "auto")).lower()

        tags_cfg: List[Dict] = []
        try:
            parsed = json.loads(str(self.get_parameter("tags_override").value or "[]"))
        except json.JSONDecodeError as e:
            self.get_logger().warn(
                f"Could not parse tags_override JSON ({e!r}); falling back to YAML tags."
            )
            parsed = []
        if isinstance(parsed, list) and parsed:
            tags_cfg = parsed
            self.get_logger().info(f"Using tags_override ({len(tags_cfg)} entries).")
        else:
            tags_cfg = inner.get("tags", []) or []
            self.get_logger().info(f"Using YAML tags ({len(tags_cfg)} entries).")

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
            raise RuntimeError("No valid tag entries (tags_override or YAML tags).")

    def _group_tags_by_family_size(self) -> Dict[Tuple[str, float], List[int]]:
        """Bucket ``_tags_config`` by ``(family, size)`` -> ``[tag_id, ...]``.

        Required because ``pupil_apriltags.Detector`` only supports a
        single ``tag_size`` per ``detect()`` call. Tags in the same
        family at different sizes must therefore be handled by separate
        detectors, even though they share the same family decoder.
        """
        groups: Dict[Tuple[str, float], List[int]] = {}
        for tag in self._tags_config:
            key = (tag["family"], float(tag["size"]))
            groups.setdefault(key, []).append(int(tag["id"]))
        return groups

    # =====================================================================
    # Intrinsics -> rectifier + detector pool
    # =====================================================================
    def _yaml_intrinsics(self) -> Optional[Tuple[float, float, float, float, int, int, list, bool]]:
        """Read the intrinsics.* ROS params into an 8-tuple, or ``None``.

        Returns ``(fx, fy, cx, cy, width, height, distortion_list, fisheye)``
        when the YAML has all the required fields; ``None`` when any
        focal length or dimension is missing/zero.
        """
        fx = _as_float(self.get_parameter("intrinsics.fx").value)
        fy = _as_float(self.get_parameter("intrinsics.fy").value)
        cx = _as_float(self.get_parameter("intrinsics.cx").value)
        cy = _as_float(self.get_parameter("intrinsics.cy").value)
        width = _as_int(self.get_parameter("intrinsics.width").value)
        height = _as_int(self.get_parameter("intrinsics.height").value)
        distortion = self.get_parameter("intrinsics.distortion").value or [0.0] * 5
        fisheye = bool(self.get_parameter("intrinsics.fisheye").value)
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
        self._build_pipeline(K, D, width, height, is_fisheye=fisheye)
        self._ready = True

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
        self._build_pipeline(K, D, width, height, is_fisheye=False)
        self._ready = True
        self.get_logger().info(
            f"CameraInfo received; {len(self._detectors)} detectors ready."
        )

    def _build_pipeline(
        self,
        K: np.ndarray,
        D: np.ndarray,
        width: int,
        height: int,
        is_fisheye: bool,
    ) -> None:
        """Build the rectifier, JPEG encoder, and per-group detector pool."""
        # --- Rectifier (CPU or CUDA) ----------------------------------------
        scale = float(self.get_parameter("process_scale").value or 1.0)
        if scale <= 0.0 or scale > 1.0:
            self.get_logger().warn(
                f"process_scale={scale} out of range (0, 1]; clamping to 1.0."
            )
            scale = 1.0
        self._process_scale = scale

        self._rectifier, self._rectifier_backend = build_rectifier(
            logger=self.get_logger(),
            camera_matrix=K,
            dist_coeffs=D,
            image_size=(width, height),
            is_fisheye=is_fisheye,
            crop_to_valid_pixels=True,
            use_cuda=self._use_cuda,
        )
        new_K = self._rectifier.get_intrinsics()
        new_size = {
            "img_width": new_K["img_width"],
            "img_height": new_K["img_height"],
        }
        new_D = np.zeros(5, dtype=np.float32)

        # If process_scale < 1.0, build a small intrinsics matrix for
        # pupil_apriltags so its pose solve runs on the downscaled image.
        if scale < 1.0:
            small_K = dict(new_K)
            small_K["fx"] = float(new_K["fx"]) * scale
            small_K["fy"] = float(new_K["fy"]) * scale
            small_K["cx"] = float(new_K["cx"]) * scale
            small_K["cy"] = float(new_K["cy"]) * scale
            small_size = {
                "img_width": max(1, int(new_size["img_width"] * scale)),
                "img_height": max(1, int(new_size["img_height"] * scale)),
            }
        else:
            small_K = new_K
            small_size = new_size

        # --- JPEG encoder (CPU, cv2.cuda, or pyNvJPEG) -----------------------
        quality = int(self.get_parameter("jpeg_quality").value or 80)
        self._jpeg_encoder, self._jpeg_backend = build_jpeg_encoder(
            quality=quality,
            use_cuda=self._use_cuda,
            requested_backend=self._jpeg_backend_requested,
        )
        if self._jpeg_backend_requested not in ("auto", "cpu") and \
                self._jpeg_backend != self._jpeg_backend_requested:
            self.get_logger().warn(
                f"Requested jpeg_backend={self._jpeg_backend_requested!r} but "
                f"fell back to {self._jpeg_backend!r} (see startup banner)."
            )

        # --- Per-(family, size) detectors -----------------------------------
        groups = self._group_tags_by_family_size()
        for (family, size), ids in groups.items():
            child_logger = self.get_logger().get_child(
                f"det_{family}_{int(round(size * 1000))}mm"
            )
            self._detectors[(family, size)] = AprilTagDetector(
                family=family,
                tag_size=size,
                tag_ids=ids,
                camera_intrinsics=small_K,
                camera_distortion=new_D.tolist(),
                image_size=small_size,
                logger=child_logger,
                detector_params=dict(self._detector_params_template),
            )

        # --- Startup HW-accel banner ----------------------------------------
        self.get_logger().info(
            "=== HW acceleration ===\n"
            f"  use_cuda (yaml) : {self._use_cuda}\n"
            f"  rectify backend : {self._rectifier_backend}\n"
            f"  jpeg   backend  : {self._jpeg_backend} (requested: {self._jpeg_backend_requested})\n"
            f"  process_scale   : {self._process_scale}\n"
            f"  jpeg_quality    : {quality}"
        )

    # =====================================================================
    # Image callbacks
    # =====================================================================
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

    # =====================================================================
    # Tick: process_frame -> build_detection_message -> publish_annotated
    # =====================================================================
    def _tick(self) -> None:
        if not self._ready:
            return
        with self._lock:
            bgr = self._latest_bgr
            stamp = self._latest_stamp
        if bgr is None:
            return

        work, detected_tags = self._process_frame(bgr)
        stamp = stamp if stamp is not None else self.get_clock().now().to_msg()

        _draw_crosshair(work)
        self._publish_annotated(work, stamp)
        self._publish_detections(detected_tags, stamp)

    def _process_frame(
        self, bgr: np.ndarray,
    ) -> Tuple[np.ndarray, List[Tuple[str, int, np.ndarray, bool]]]:
        """Rectify + downscale + detect + annotate. Returns ``(work, detections)``.

        ``work`` is the full-resolution rectified BGR ready for the
        crosshair + JPEG encode pass. ``detections`` is the list of
        ``(family, tag_id, T_cam_to_tag, was_bad)`` for the message.
        """
        work = self._rectifier.rectify(bgr)

        if self._process_scale < 1.0:
            small = cv2.resize(
                work, None,
                fx=self._process_scale, fy=self._process_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            small = work
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        detected: List[Tuple[str, int, np.ndarray, bool]] = []
        for (family, _size), detector in self._detectors.items():
            try:
                detections = detector.detect(gray)
            except Exception as e:
                self.get_logger().warn(
                    f"Detector (family={family}, ids={sorted(detector.tag_ids)}) "
                    f"failed: {e}; skipping this tick."
                )
                continue

            # Scale corners back up to full-res so annotate draws on ``work``.
            if self._process_scale < 1.0 and detections:
                inv = 1.0 / self._process_scale
                scaled = []
                for det in detections:
                    d2 = dict(det)
                    d2["corners"] = det["corners"] * inv
                    scaled.append(d2)
                detections = scaled

            for det in detections:
                tag_id = det["tag_id"]
                T = det["T"]
                try:
                    was_bad = is_bad_rotation(T[:3, :3])
                    T[:3, :3] = sanitize_rotation(T[:3, :3])
                    if (
                        not np.all(np.isfinite(T[:3, :3]))
                        or np.linalg.det(T[:3, :3]) < 0.5
                    ):
                        raise ValueError("rotation unrecoverable after SVD")
                    detected.append((family, tag_id, T, was_bad))
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
                detector.annotate(work, detections)
            except Exception as e:
                self.get_logger().warn(
                    f"Detector (family={family}) annotate failed: {e}"
                )

        return work, detected

    # =====================================================================
    # Publications
    # =====================================================================
    def _publish_annotated(self, work: np.ndarray, stamp) -> None:
        """Encode ``work`` to JPEG (HW or CPU) and publish as CompressedImage."""
        try:
            jpeg_bytes = self._jpeg_encoder.encode(work)
        except Exception as e:
            self.get_logger().error(f"JPEG encode failed ({self._jpeg_backend}): {e}")
            return

        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = jpeg_bytes
        msg.header.stamp = stamp
        msg.header.frame_id = self._camera_frame_id()
        self.image_pub.publish(msg)

    def _publish_detections(
        self,
        detected: List[Tuple[str, int, np.ndarray, bool]],
        stamp,
    ) -> None:
        msg = Detection3DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._camera_frame_id()

        for family, tag_id, T_cam_to_tag, _was_bad in detected:
            det3d = Detection3D()
            det3d.header = msg.header
            det3d.id = f"{family}:{tag_id}"
            hyp = ObjectHypothesis(class_id=str(tag_id), score=1.0)
            pwp = ObjectHypothesisWithPose(hypothesis=hyp)
            pwp.pose = PoseWithCovariance()
            pwp.pose.pose = matrix_to_pose_msg(T_cam_to_tag)
            det3d.results = [pwp]
            det3d.bbox = BoundingBox3D()
            det3d.bbox.center = Pose()
            det3d.bbox.size = Vector3()
            msg.detections.append(det3d)

        self.detections_pub.publish(msg)

    # =====================================================================
    # Frame id
    # =====================================================================
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