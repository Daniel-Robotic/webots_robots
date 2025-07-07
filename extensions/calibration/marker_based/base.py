import cv2
import numpy as np
from pathlib import Path
from typing import Union, List, Tuple

from cv2 import aruco
from ..calibration_base import CameraBaseCalibrator
from ..types import (
    ImageSize,
    FindArucoCornerResult
)


class ArucoMarkerCalibrator(CameraBaseCalibrator):
    """
    Base class for ArUco marker-based camera calibration.
    Supports detection of markers using different predefined dictionaries.
    """

    ARUCO_DICTS = {
        "4x4_50": aruco.DICT_4X4_50,
        "5x5_100": aruco.DICT_5X5_100,
        "5x5_250": aruco.DICT_5X5_250,
        "6x6_1000": aruco.DICT_6X6_1000,
        "7x7_1000": aruco.DICT_7X7_1000
    }

    def __init__(
        self,
        image_folder: Union[str, Path],
        pattern_size: Tuple[int, int],
        aruco_dict_name: str = "5x5_250",
        marker_length_mm: Union[float, int] = 25.0
    ):
        """
        :param image_folder: Directory containing images
        :param pattern_size: Size of marker grid (cols, rows)
        :param aruco_dict_name: Name of the ArUco dictionary
        :param marker_length_mm: Physical marker size in mm
        """
        if not isinstance(aruco_dict_name, str):
            raise TypeError("aruco_dict_name must be a string")

        if not isinstance(marker_length_mm, (float, int)):
            raise TypeError("marker_length_mm must be a numeric value (float or int)")

        super().__init__(image_folder, pattern_size)

        self._aruco_dict_name = ""
        self._aruco_dict = None
        self.aruco_dict = aruco_dict_name
        self.marker_length_mm = marker_length_mm

        self._params = aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(self._aruco_dict, self._params)
        self._board = None

    @property
    def aruco_dict(self) -> str:
        return self._aruco_dict_name

    @aruco_dict.setter
    def aruco_dict(self, value: str) -> None:
        if not isinstance(value, str):
            raise TypeError("aruco_dict must be a string representing the dictionary name")

        if value not in self.ARUCO_DICTS:
            raise ValueError(
                f"Unsupported ArUco dictionary: {value}. "
                f"Supported values are: {list(self.ARUCO_DICTS.keys())}"
            )

        self._aruco_dict = aruco.getPredefinedDictionary(self.ARUCO_DICTS[value])
        self._aruco_dict_name = value

    @property
    def marker_length_mm(self) -> Union[float, int]:
        return self._marker_length_mm

    @marker_length_mm.setter
    def marker_length_mm(self, value: Union[float, int]) -> None:
        if not isinstance(value, (float, int)):
            raise TypeError("marker_length_mm must be a float or int")

        self._marker_length_mm = value
        self._marker_lenght = value / 1000.0  # in meters

    def find_corners(self, image: np.ndarray) -> FindArucoCornerResult:
        """
        Stub method for marker detection. Should be overridden in subclasses.

        :param image: Grayscale image
        :return: Tuple of (success flag, corners or None, ids or None)
        """
        raise NotImplementedError("This method should be implemented in a subclass.")

    def calibrate(self) -> dict:
        """
        Stub for calibration logic to be implemented in derived classes.
        """
        raise NotImplementedError("Calibration must be implemented in a derived class.")


    def _preprocess_images(
        self,
        error_msg: str,
        pattern: str = "*",
        extend_corners: bool = True
    ) -> Tuple[List[np.ndarray], List[np.ndarray], List[int], ImageSize]:
        """
        Detects ArUco markers across all valid images.

        :param error_msg: Error to raise if not enough valid detections
        :param pattern: Glob pattern for file search
        :param extend_corners: Whether to flatten corners list across images
        :return: Tuple with detected corners, IDs, marker counts, and image size
        """
        if not isinstance(error_msg, str):
            raise TypeError("error_msg must be a string")
        if not isinstance(pattern, str):
            raise TypeError("pattern must be a string")
        if not isinstance(extend_corners, bool):
            raise TypeError("extend_corners must be a boolean")

        all_corners: List[np.ndarray] = []
        all_ids: List[np.ndarray] = []
        counter: List[int] = []

        img_size, image_paths = self._check_images_size(pattern=pattern)

        for img_path in image_paths:
            image = cv2.imread(str(img_path))
            if image is None:
                print(f"Failed to load image: {img_path}")
                continue

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found, corners, ids = self.find_corners(gray)

            if found and corners is not None and ids is not None:
                all_ids.append(ids)
                counter.append(len(ids))

                if extend_corners:
                    all_corners.extend(corners)
                else:
                    all_corners.append(corners)

        if len(counter) < 5:
            raise ValueError(error_msg)

        return all_corners, all_ids, counter, img_size
    
