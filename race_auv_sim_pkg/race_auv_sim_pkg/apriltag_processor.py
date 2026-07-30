import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage

from scipy.spatial.transform import Rotation as R
from pupil_apriltags import Detector

from .image_processing import ImageRectifier

class AprilTagDetector:
    """
    A class to detect AprilTags in an image using the pupil-apriltags library,
    perform pose estimation, and draw the results on the image.
    """
    def __init__(self, family, tag_size, camera_intrinsics, camera_distortion, image_size, logger, detector_params):
        """
        Initializes the AprilTag detector.

        :param family: The family of AprilTags to detect (e.g., 'tag36h11').
        :param tag_size: The size of the tags in meters.
        :param camera_intrinsics: A dictionary with camera intrinsic parameters [fx, fy, cx, cy].
        :param camera_distortion: A list or tuple of camera distortion coefficients.
        :param image_size: A dictionary with image dimensions {'img_width', 'img_height'}.
        :param logger: A ROS 2 logger object for logging messages.
        :param detector_params: A dictionary of tunable parameters for the pupil-apriltags detector.
        """
        self.logger = logger
        self.tag_size = float(tag_size)

        # pupil-apriltags takes camera params as a simple list/tuple: [fx, fy, cx, cy]
        self.camera_params = (
            camera_intrinsics['fx'],
            camera_intrinsics['fy'],
            camera_intrinsics['cx'],
            camera_intrinsics['cy']
        )
        
        # NOTE: pupil-apriltags does not use distortion coefficients for its internal pose estimation.
        # The user should provide an undistorted image if pose accuracy is critical.
        # The provided coefficients are only used for drawing the axes, so we build the matrix here.
        self.distCoeffs = np.array(camera_distortion, dtype=np.float32)
        self.camera_intrinsics_mtx = np.array([
            [self.camera_params[0], 0, self.camera_params[2]],
            [0, self.camera_params[1], self.camera_params[3]],
            [0, 0, 1]
        ], dtype=np.float32)

        self.img_width = image_size['img_width']
        self.img_height = image_size['img_height']

        # --- Create pupil-apriltags Detector ---
        try:
            self.detector = Detector(
                families=family,
                **detector_params # Pass YAML parameters directly
            )
            self.logger.info(f"pupil-apriltags detector created for family '{family}' with params: {detector_params}")
            self.logger.info(f"Detector configured for image size {self.img_width}x{self.img_height} and camera params: {self.camera_params}")

        except Exception as e:
            self.logger.error(f"Failed to create pupil-apriltags detector: {e}")
            self.detector = None

    def _rotation_matrix_to_euler_angles(self, R_matrix):
        """
        Converts a rotation matrix to Euler angles (roll, pitch, yaw) in degrees.
        """
        r = R.from_matrix(R_matrix)
        return r.as_euler('xyz', degrees=True)

    def detect_and_draw(self, image):
        """
        Detects AprilTags in the given image, estimates their pose, and draws
        visualizations on the image. This method is now robust against bad
        detections that could cause crashes.
        """
        if self.detector is None:
            self.logger.error("AprilTag detector is not initialized. Cannot process image.")
            return image # Return original image
            
        if image is None:
            self.logger.warn("Received a null image for AprilTag detection.")
            return None
        
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect tags. The library handles pose estimation internally if camera_params and tag_size are provided.
        detections = self.detector.detect(
            gray_image, 
            estimate_tag_pose=True, 
            camera_params=self.camera_params, 
            tag_size=self.tag_size
        )
        
        # If tags are successfully detected, process and draw them
        for tag in detections:
            corners = tag.corners.astype(int)
            
            try:
                # --- POSE ESTIMATION AND VISUALIZATION ---
                # This block is wrapped in a try-except to handle cases where pose
                # estimation results in an invalid rotation matrix (e.g., from a
                # noisy or false positive detection), which would crash scipy.
                
                # Extract pose info
                tvec = tag.pose_t.flatten()
                R_matrix = tag.pose_R
                
                # This can fail if R_matrix is not a valid 3x3 matrix
                rvec, _ = cv2.Rodrigues(R_matrix)
                
                # This is the call that was causing the crash
                roll, pitch, yaw = self._rotation_matrix_to_euler_angles(R_matrix)

                # --- DRAW SUCCESSFUL DETECTION ---
                # If all conversions are successful, draw the full pose info
                # Green bounding box for successfully identified tags with valid poses
                cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.drawFrameAxes(image, self.camera_intrinsics_mtx, self.distCoeffs, rvec, tvec, self.tag_size * 0.5)

                pose_t_str = f"xyz: ({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f})"
                pose_r_str = f"rpy: ({roll:.0f}, {pitch:.0f}, {yaw:.0f})"
                id_str = f"ID: {tag.tag_id}"
                text_anchor = [tag.center[0].astype(int), tag.center[1].astype(int)]
                cv2.putText(image, id_str, (text_anchor[0] - 125, text_anchor[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                cv2.putText(image, pose_t_str, (text_anchor[0] - 125, text_anchor[1] - 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)
                cv2.putText(image, pose_r_str, (text_anchor[0] - 125, text_anchor[1] - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 0), 2)

            except (ValueError, cv2.error) as e:
                # --- DRAW FAILED DETECTION ---
                # If pose estimation fails (e.g., non-positive determinant in rotation matrix),
                # log a warning and mark the tag on the image without crashing.
                self.logger.warn(f"Could not process pose for tag {tag.tag_id}. It might be a false positive. Error: {e}")
                
                # Draw a red bounding box to indicate a detection with a bad pose
                cv2.polylines(image, [corners], isClosed=True, color=(0, 0, 255), thickness=2)
                
                # Add text to indicate the failed pose estimation
                text_anchor = tuple(corners[0])
                id_str = f"ID: {tag.tag_id} (Bad Pose)"
                cv2.putText(image, id_str, (text_anchor[0], text_anchor[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            
        return image
    
class AprilTagProcessor:
    """
    An auxiliary processor for detecting AprilTags. It initializes the detector,
    runs detection on its own timer, and publishes the annotated image.
    """
    def __init__(self, parent_node: rclpy.node.Node, callback_group, qos_profile):
        """
        Initializes the AprilTag processor.
        
        :param parent_node: The main camera node.
        :param callback_group: The ROS 2 callback group for the timer.
        """
        self._node = parent_node
        self._logger = self._node.get_logger().get_child('apriltag_processor')

        # Get AprilTag parameters
        publish_rate = self._node.get_parameter('apriltag.publish_rate').value
        tag_family = self._node.get_parameter('apriltag.family').value
        tag_size = self._node.get_parameter('apriltag.size').value
        self._jpeg_quality = int(self._node.get_parameter('compression.jpeg_quality').value)
        
        # Get original camera parameters for rectification
        camera_intrinsics = {
            'fx': self._node.get_parameter('camera.intrinsics.fx').value,
            'fy': self._node.get_parameter('camera.intrinsics.fy').value,
            'cx': self._node.get_parameter('camera.intrinsics.cx').value,
            'cy': self._node.get_parameter('camera.intrinsics.cy').value
        }
        camera_distortion = self._node.get_parameter('camera.distortion').value
        is_fisheye = self._node.get_parameter('camera.fisheye').value
        crop_image = self._node.get_parameter('camera.undistort_crop').value
        image_size_tuple = (
            self._node.get_parameter('video.width').value,
            self._node.get_parameter('video.height').value
        )
        detector_params = {
            'nthreads': self._node.get_parameter('apriltag.detector.nthreads').value,
            'quad_decimate': self._node.get_parameter('apriltag.detector.quad_decimate').value,
            'quad_sigma': self._node.get_parameter('apriltag.detector.quad_sigma').value,
            'refine_edges': self._node.get_parameter('apriltag.detector.refine_edges').value,
            'decode_sharpening': self._node.get_parameter('apriltag.detector.decode_sharpening').value,
        }
        camera_matrix = np.array([
            [camera_intrinsics['fx'], 0, camera_intrinsics['cx']],
            [0, camera_intrinsics['fy'], camera_intrinsics['cy']],
            [0, 0, 1]
        ], dtype=np.float32)
        dist_coeffs = np.array(camera_distortion, dtype=np.float32)

        # Initialize the centralized image rectifier
        self._rectifier = ImageRectifier(
            logger=self._logger,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_size=image_size_tuple,
            is_fisheye=is_fisheye,
            crop_to_valid_pixels=crop_image
        )
        # Get the new, correct parameters for the rectified image from the rectifier
        new_camera_intrinsics = self._rectifier.get_new_camera_params()
        new_distortion_coeffs = self._rectifier.get_new_distortion_coeffs().tolist()
        new_image_size = self._rectifier.get_new_image_size()

        # Initialize the actual AprilTag detector logic with the rectified parameters
        self._detector = AprilTagDetector(
            family=tag_family,
            tag_size=tag_size,
            camera_intrinsics=new_camera_intrinsics,
            camera_distortion=new_distortion_coeffs,
            image_size=new_image_size,
            logger=self._logger,
            detector_params=detector_params
        )

        self._bridge = CvBridge()
        self._frame_id = self._node.get_parameter('ros.frame_id').value
        # Create ROS publisher and timer
        self._publisher = self._node.create_publisher(CompressedImage, "apriltag_detection/compressed", qos_profile=qos_profile)
        self._timer = self._node.create_timer(
            1.0 / publish_rate,
            self._timer_callback,
            callback_group=callback_group
        )
        self._logger.info(f"Initialized. Publishing detection results at {publish_rate} Hz.")

    def _timer_callback(self):

        """Periodically rectifies and publishes the result."""
        if self._node._latest_msg is not None:
            frame_data = self._node._latest_msg
            try:
                header = Header(stamp=self._node.get_clock().now().to_msg(), frame_id=self._frame_id)
                # Rectify the image using the centralized rectifier
                rectified_image = self._rectifier.rectify(frame_data)
                # Perform detection on the rectified image and get the annotated image
                annotated_image = self._detector.detect_and_draw(rectified_image)

                if annotated_image is not None:
                    # Publish calibrated image as compressed image
                    detection_img_msg = self._bridge.cv2_to_compressed_imgmsg(rectified_image)
                    detection_img_msg.header = header
                    self._publisher.publish(detection_img_msg)
            except CvBridgeError as e:
                self._logger.error(f"Error converting frame to Image message: {e}")

    def shutdown(self):
        """Cancels the timer to cleanly shut down the processor."""
        self._logger.info("Shutting down.")
        if self._timer: self._timer.cancel()