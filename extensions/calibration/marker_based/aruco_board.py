import cv2
import numpy as np
from cv2 import aruco
from pathlib import Path
from typing import Tuple, Optional, Dict, Any, Union

from ..marker_based.base import ArucoMarkerCalibrator
from ..types import FindArucoCornerResult


class ArucoBoardCalibrator(ArucoMarkerCalibrator):
    """
    ArUco marker board calibrator based on a regular grid of markers.
    """

    def __init__(
        self,
        image_folder: Union[str, Path],
        pattern_size: Tuple[int, int],
        aruco_dict_name: str = "5x5_250",
        marker_length_mm: Union[float, int] = 25.0,
        marker_separation: float = 0.25
    ):
        """
        :param image_folder: Path to folder with images
        :param pattern_size: (cols, rows) of the marker grid
        :param aruco_dict_name: Dictionary name for ArUco markers
        :param marker_length_mm: Marker side length in millimeters
        :param marker_separation: Separation between markers (relative to marker size)
        """
        if not isinstance(marker_separation, float):
            raise TypeError("marker_separation must be a float")

        super().__init__(
            image_folder=image_folder,
            pattern_size=pattern_size,
            aruco_dict_name=aruco_dict_name,
            marker_length_mm=marker_length_mm
        )

        self.marker_separation = marker_separation  # invokes setter

        self._board = aruco.GridBoard(
            size=self._pattern_size,
            markerLength=self._marker_lenght,
            markerSeparation=self._marker_lenght * self._marker_separation,
            dictionary=self._aruco_dict
        )

    @property
    def marker_separation(self) -> float:
        return self._marker_separation

    @marker_separation.setter
    def marker_separation(self, value: float) -> None:
        if not isinstance(value, float):
            raise TypeError("marker_separation must be a float")

        if not 0.01 <= value <= 1.0:
            raise ValueError("marker_separation should be between 0.01 and 1.0")

        self._marker_separation = value

    def find_corners(self, image: np.ndarray) -> FindArucoCornerResult:
        """
        Detects ArUco markers in the image using the configured detector.

        :param image: Grayscale image
        :return: Tuple (found, corners, ids)
        """
        if not isinstance(image, np.ndarray):
            raise TypeError("image must be a NumPy ndarray")

        corners, ids, _ = self._detector.detectMarkers(image)

        if ids is not None and len(ids) > 0:
            return True, corners, ids

        return False, None, None

    def calibrate(
        self,
        pattern: str = "*",
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None
    ) -> Dict[str, Any]:
        """
        Calibrates the camera using detected ArUco markers.

        :param pattern: Glob pattern to find images
        :param camera_matrix: Initial camera matrix (optional)
        :param dist_coeffs: Initial distortion coefficients (optional)
        :return: Calibration results dictionary
        """
        if not isinstance(pattern, str):
            raise TypeError("pattern must be a string")
        if camera_matrix is not None and not isinstance(camera_matrix, np.ndarray):
            raise TypeError("camera_matrix must be a NumPy ndarray or None")
        if dist_coeffs is not None and not isinstance(dist_coeffs, np.ndarray):
            raise TypeError("dist_coeffs must be a NumPy ndarray or None")

        all_corners, all_ids, counter, img_size = self._preprocess_images(
            error_msg="Insufficient valid images with ArUco markers for calibration",
            pattern=pattern,
            extend_corners=True
        )

        all_ids = np.vstack(all_ids).astype(np.int32)
        counter = np.array(counter, dtype=np.int32)

        ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(
            corners=all_corners,
            ids=all_ids,
            counter=counter,
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
        
        found, rvec, tvec = aruco.estimatePoseBoard(
            corners, ids, self._board,
            self._calibration_result["matrix"], 
            self._calibration_result["distortion"], 
            None, None)
        
        if found:
            return cv2.Rodrigues(rvec)[0], tvec
        
        return None