import cv2
import numpy as np

class ImageRectifier:
    """
    A class to handle camera image rectification (undistortion).

    This class performs the necessary one-time calculations for undistortion
    based on the camera's intrinsic parameters and distortion model (standard
    or fisheye). It pre-computes remap matrices and regions of interest (ROI)
    to efficiently rectify images on the fly.

    It provides the rectified image, the new camera matrix adjusted for any
    cropping, and the new image dimensions.
    """

    def __init__(self, logger, camera_matrix, dist_coeffs, image_size, is_fisheye, crop_to_valid_pixels):
        """
        Initializes the ImageRectifier and computes rectification parameters.

        :param logger: A ROS 2 logger object for logging messages.
        :param camera_matrix: 3x3 numpy array of the camera's intrinsic matrix (K).
        :param dist_coeffs: Numpy array of distortion coefficients (D).
        :param image_size: A tuple (width, height) of the original image.
        :param is_fisheye: Boolean, True if using the fisheye distortion model.
        :param crop_to_valid_pixels: Boolean, True to crop the image to the area
                                     with valid pixels, removing black borders.
        """
        self._logger = logger
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs
        self._image_size = image_size
        self._is_fisheye = is_fisheye
        self._crop = crop_to_valid_pixels
        self.map1, self.map2 = None, None

        self._logger.info(f"Initializing ImageRectifier: fisheye={self._is_fisheye}, crop={self._crop}")

        if self._is_fisheye:
            self._init_fisheye()
        else:
            self._init_standard()

        # After initialization, self.new_camera_matrix and self.roi are set.
        # Now, calculate the final parameters for the *cropped* image.

        # The new image dimensions are the dimensions of the ROI.
        self._new_width = self.roi[2]
        self._new_height = self.roi[3]

        # Adjust the principal point (cx, cy) of the new camera matrix to account
        # for the cropping defined by the ROI. The new origin is the top-left
        # corner of the ROI.
        self._final_camera_matrix = self.new_camera_matrix.copy()
        self._final_camera_matrix[0, 2] -= self.roi[0] # Adjust cx
        self._final_camera_matrix[1, 2] -= self.roi[1] # Adjust cy
        
        self._logger.info(f"Rectification initialized. New image size: {self._new_width}x{self._new_height}")
        self._logger.debug(f"Original camera matrix:\n{self._camera_matrix}")
        self._logger.debug(f"New camera matrix (before crop adjustment):\n{self.new_camera_matrix}")
        self._logger.info(f"Final camera matrix (for cropped image):\n{self._final_camera_matrix}")

    def _init_fisheye(self):
        """Initializes rectification for the fisheye model."""
        if len(self._dist_coeffs) != 4:
            self._logger.warn(f"Fisheye model selected, but {len(self._dist_coeffs)} distortion coefficients "
                              f"were provided. Expected 4 (k1, k2, k3, k4).")

        # balance=0.0 crops to valid pixels, balance=1.0 shows all pixels.
        balance = 0.0 if self._crop else 1.0
        log_msg = "with cropping" if self._crop else "without cropping"
        self._logger.info(f"Using fisheye model for undistortion {log_msg}.")

        # Estimate the new camera matrix.
        # The new matrix is needed for initUndistortRectifyMap.
        self.new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self._camera_matrix, self._dist_coeffs, self._image_size, np.eye(3), balance=balance
        )
        
        # Pre-compute the remap maps for efficiency.
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self._camera_matrix, self._dist_coeffs, np.eye(3), self.new_camera_matrix,
            self._image_size, cv2.CV_16SC2
        )

        if self._crop:
            # To find the ROI for a cropped fisheye image, we must undistort a mask
            # and find the bounding box of the non-black area.
            mask = np.ones(self._image_size[::-1], dtype=np.uint8) * 255 # H, W format
            undistorted_mask = cv2.remap(mask, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            contours, _ = cv2.findContours(undistorted_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                self.roi = cv2.boundingRect(largest_contour)
            else:
                self._logger.warn("Could not find contour in fisheye undistorted mask. Not cropping.")
                self.roi = (0, 0, self._image_size[0], self._image_size[1])
        else:
            # If not cropping, the ROI is the full image size.
            self.roi = (0, 0, self._image_size[0], self._image_size[1])

    def _init_standard(self):
        """Initializes rectification for the standard (plumb bob) model."""
        if len(self._dist_coeffs) < 4:
            self._logger.warn(f"Standard model selected, but only {len(self._dist_coeffs)} distortion "
                              f"coefficients were provided. Expected at least 4 (k1, k2, p1, p2).")
        
        # alpha=0 crops to valid pixels, alpha=1 shows all pixels.
        alpha = 0.0 if self._crop else 1.0
        log_msg = "with cropping" if self._crop else "without cropping"
        self._logger.info(f"Using standard (plumb bob) model for undistortion {log_msg}.")

        # getOptimalNewCameraMatrix calculates both the new matrix and the ROI.
        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self._camera_matrix, self._dist_coeffs, self._image_size, alpha, self._image_size
        )

    def rectify(self, image):
        """
        Applies the pre-computed rectification to an image.

        :param image: The original, distorted input image (numpy array).
        :return: The rectified and cropped image (numpy array).
        """
        if self._is_fisheye:
            # Use the pre-computed remap for fisheye, it's faster.
            rect_img = cv2.remap(image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        else:
            # For standard model, cv2.undistort is convenient.
            rect_img = cv2.undistort(image, self._camera_matrix, self._dist_coeffs, None, self.new_camera_matrix)

        # Crop the rectified image to the calculated ROI.
        x, y, w, h = self.roi
        return rect_img[y:y+h, x:x+w]

    def get_new_camera_params(self):
        """
        Returns the camera parameters for the *final, rectified, and cropped* image.

        :return: A dictionary containing the new intrinsics (fx, fy, cx, cy).
        """
        return {
            'fx': self._final_camera_matrix[0, 0],
            'fy': self._final_camera_matrix[1, 1],
            'cx': self._final_camera_matrix[0, 2],
            'cy': self._final_camera_matrix[1, 2]
        }
    
    def get_new_distortion_coeffs(self):
        """
        Returns the distortion coefficients for the rectified image.
        This is always an array of zeros.

        :return: Numpy array of zeros.
        """
        num_coeffs = 4 if self._is_fisheye else 5
        return np.zeros(num_coeffs, dtype=np.float32)

    def get_new_image_size(self):
        """
        Returns the dimensions of the final, rectified, and cropped image.

        :return: A dictionary containing the new width and height.
        """
        return {
            'img_width': self._new_width,
            'img_height': self._new_height
        }