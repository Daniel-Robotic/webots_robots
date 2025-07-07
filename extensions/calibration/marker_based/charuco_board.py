import cv2
import numpy as np
from pathlib import Path
from typing import Tuple, Optional, Dict, Union

from cv2 import aruco

from ..marker_based.base import ArucoMarkerCalibrator
from ..types import FindArucoCornerResult


class CharucoBoardCalibrator(ArucoMarkerCalibrator):
    """
    Calibrator for a ChArUco board combining chessboard and ArUco markers.
    """

    def __init__(
        self,
        image_folder: Union[str, Path],
        pattern_size: Tuple[int, int],
        aruco_dict_name: str = "5x5_250",
        marker_length_mm: Union[float, int] = 25.0,
        pattern_length_mm: Union[float, int] = 40.0
    ):
        """
        :param image_folder: Directory containing calibration images
        :param pattern_size: Number of inner corners per chessboard row and column (columns, rows)
        :param aruco_dict_name: ArUco dictionary name
        :param marker_length_mm: Side length of ArUco markers in mm
        :param pattern_length_mm: Length of ChArUco squares in mm
        """
        if not isinstance(pattern_length_mm, (float, int)):
            raise TypeError("pattern_length_mm must be a float or int")

        super().__init__(image_folder, pattern_size, aruco_dict_name, marker_length_mm)

        self.pattern_length_mm = pattern_length_mm  # triggers setter

        self._board = aruco.CharucoBoard(
            size=self._pattern_size,
            squareLength=self._pattern_length,
            markerLength=self._marker_lenght,
            dictionary=self._aruco_dict
        )

    @property
    def pattern_length_mm(self) -> Union[float, int]:
        return self._pattern_length_mm

    @pattern_length_mm.setter
    def pattern_length_mm(self, value: Union[float, int]) -> None:
        if not isinstance(value, (float, int)):
            raise TypeError("pattern_length_mm must be a float or int")
        if value <= 0:
            raise ValueError("pattern_length_mm must be positive")

        self._pattern_length_mm = value
        self._pattern_length = self._pattern_length_mm / 1000.0

    def find_corners(self, image: np.ndarray) -> FindArucoCornerResult:
        """
        Detects ChArUco board corners in a grayscale image.

        :param image: Grayscale image
        :return: Tuple (found, charuco_corners, charuco_ids)
        """
        if not isinstance(image, np.ndarray):
            raise TypeError("image must be a NumPy ndarray")

        corners, ids, _ = self._detector.detectMarkers(image)

        if ids is None or len(ids) < 2:
            return False, None, None

        retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=image,
            board=self._board
        )

        if retval and charuco_corners is not None and len(charuco_corners) > 5:
            return True, charuco_corners, charuco_ids

        return False, None, None

    def calibrate(
        self,
        pattern: str = "*",
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None
    ) -> Dict[str, Union[float, np.ndarray]]:
        """
        Calibrates the camera using detected ChArUco corners.

        :param pattern: Glob pattern for image filenames
        :param camera_matrix: Optional initial camera matrix
        :param dist_coeffs: Optional initial distortion coefficients
        :return: Dictionary containing calibration result
        """
        if not isinstance(pattern, str):
            raise TypeError("pattern must be a string")

        all_corners, all_ids, _, img_size = self._preprocess_images(
            error_msg="Insufficient valid images with ChArUco markers for calibration",
            pattern=pattern,
            extend_corners=False
        )

        ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=all_corners,
            charucoIds=all_ids,
            board=self._board,
            imageSize=img_size,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs
        )

        self._calibration_result = {
            "ret": ret,
            "matrix": mtx,
            "distortion": dist,
            "rotation_vectors": rvecs,
            "translation_vectors": tvecs,
        }

        return self._calibration_result

    @ArucoMarkerCalibrator._handeye_calibration_decorator
    def handeye_calibrate(self, image):
        found, corners, ids = self.find_corners(image)

        if not found:
            return None
        
        found, rvec, tvec = aruco.estimatePoseCharucoBoard(
            corners, ids, self._board,
            self._calibration_result["matrix"], 
            self._calibration_result["distortion"], 
            None, None)
        
        if found:
            return cv2.Rodrigues(rvec)[0], tvec
        
        return None