import cv2
import numpy as np
from pathlib import Path
from typing import Tuple, Optional, Union, List

from ..types import (
    ImageSize,
    ObjectPoints,
    ImagePoints,
    CameraMatrix,
    DistCoeffs,
    CalibrationResult,
    FindCornersResult,
)
from ..calibration_base import CameraBaseCalibrator


class PatternBasedCalibrator(CameraBaseCalibrator):
    def __init__(
        self,
        image_folder: Union[str, Path],
        pattern_size: Tuple[int, int],
        pattern_length_mm: Union[float, int] = 40.0
    ):
        if not isinstance(pattern_length_mm, (float, int)):
            raise TypeError("pattern_length_mm must be a float or int")

        super().__init__(image_folder, pattern_size)
        self.pattern_length_mm = pattern_length_mm

    @property
    def pattern_length_mm(self) -> Union[float, int]:
        return self._pattern_length_mm

    @pattern_length_mm.setter
    def pattern_length_mm(self, value: Union[float, int]) -> None:
        if not isinstance(value, (float, int)):
            raise ValueError("pattern_length_mm must be a numeric value (float or int)")

        if value <= 0:
            raise ValueError("pattern_length_mm must be positive")

        self._pattern_length_mm = value
        self._pattern_length = self._pattern_length_mm / 1000.0

    def find_corners(self, gray_image: np.ndarray) -> FindCornersResult:
        """
        Abstract method stub for finding corners on a grayscale image.
        Must be overridden in child classes.

        :param gray_image: Grayscale image as numpy array
        :return: Tuple of success flag and corner coordinates (if found)
        """
        raise NotImplementedError("This method should be implemented in a subclass.")

    def _prepare_3d_points(self) -> ObjectPoints:
        """
        Generates a grid of 3D object points in real-world units (meters),
        based on the calibration pattern dimensions.

        :return: (N, 3) float32 numpy array of 3D points
        """
        width, height = self._pattern_size
        object_points = np.zeros((height * width, 3), dtype=np.float32)
        object_points[:, :2] = (
            np.mgrid[0:width, 0:height].T.reshape(-1, 2) * self._pattern_length
        )
        return object_points

    def _preprocess_images(
        self, pattern: str = "*"
    ) -> Tuple[ImageSize, List[ObjectPoints], List[ImagePoints]]:
        """
        Loads and processes calibration images, finds corners and associates
        them with corresponding 3D pattern points.

        :param pattern: File glob pattern for images
        :return: Image size, list of 3D object points, list of 2D image points
        """
        if not isinstance(pattern, str):
            raise TypeError("pattern must be a string")

        object_points: List[ObjectPoints] = []
        image_points: List[ImagePoints] = []

        objps = self._prepare_3d_points()
        img_size, image_paths = self._check_images_size(pattern=pattern)

        for img_path in image_paths:
            image = cv2.imread(str(img_path))
            if image is None:
                print(f"Failed to load image: {img_path.name}")
                continue

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            found, corners = self.find_corners(gray)

            if found and corners is not None:
                object_points.append(objps.copy())
                image_points.append(corners)
            else:
                print(f"No corners found in image: {img_path.name}")

        if len(image_points) < 5:
            raise ValueError(
                "There are not enough images with detected points for calibration (minimum is 5)"
            )

        return img_size, object_points, image_points

    def calibrate(
        self,
        pattern: str = "*",
        camera_matrix: Optional[CameraMatrix] = None,
        dist_coeffs: Optional[DistCoeffs] = None,
    ) -> CalibrationResult:
        """
        Performs camera calibration based on loaded and processed images.

        :param pattern: Glob pattern for image selection
        :param camera_matrix: Optional initial guess for camera matrix
        :param dist_coeffs: Optional initial guess for distortion coefficients
        :return: Dictionary with calibration results
        """
        if not isinstance(pattern, str):
            raise TypeError("pattern must be a string")

        if camera_matrix is not None and (
            not isinstance(camera_matrix, np.ndarray) or camera_matrix.shape != (3, 3)
        ):
            raise ValueError("camera_matrix must be a (3, 3) numpy array")

        if dist_coeffs is not None and not isinstance(dist_coeffs, np.ndarray):
            raise ValueError("dist_coeffs must be a numpy array")

        img_size, object_points, image_points = self._preprocess_images(pattern=pattern)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objectPoints=object_points,
            imagePoints=image_points,
            imageSize=img_size,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
        )

        self._calibration_result = {
            "ret": ret,
            "matrix": mtx,
            "distortion": dist,
            "rotation_vectors": rvecs,
            "translation_vectors": tvecs,
        }

        return self._calibration_result
    
    @CameraBaseCalibrator._handeye_calibration_decorator
    def handeye_calibrate(self, image):
        found, corners = self.find_corners(image)
        obj_points = self._prepare_3d_points()

        if not found:
            return None
        
        ret, rvec, tvec = cv2.solvePnP(obj_points,
                                       corners,
                                       self._calibration_result["matrix"],
                                       self._calibration_result["distortion"])

        if ret:
            return cv2.Rodrigues(rvec)[0], tvec

        return None