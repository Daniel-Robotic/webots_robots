import cv2
import numpy as np
from pathlib import Path
from typing import Tuple, Union

from ..types import FindCornersResult
from ..pattern_based.base import PatternBasedCalibrator


class ChessboardCalibrator(PatternBasedCalibrator):
    def __init__(
        self,
        image_folder: Union[str, Path],
        pattern_size: Tuple[int, int],
        pattern_length_mm: Union[float, int] = 40
    ):
        """
        Chessboard calibrator using OpenCV's findChessboardCorners.

        :param image_folder: Path to directory containing calibration images
        :param pattern_size: Number of inner corners per chessboard row and column (cols, rows)
        :param pattern_length_mm: Square side length in millimeters
        """
        if not isinstance(pattern_length_mm, (int, float)):
            raise TypeError("pattern_length_mm must be a float or int")

        super().__init__(image_folder, pattern_size, pattern_length_mm)

    def find_corners(self, image: np.ndarray) -> FindCornersResult:
        """
        Detects chessboard corners in the input image.

        :param image: Grayscale image as NumPy array
        :return: Tuple (success flag, corner points or None)
        """
        if not isinstance(image, np.ndarray):
            raise TypeError("image must be a numpy ndarray")

        if image.ndim != 2:
            raise ValueError("image must be a single-channel (grayscale) image")

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK

        found, corners = cv2.findChessboardCorners(image, self._pattern_size, flags)

        if found and corners is not None:
            corners = cv2.cornerSubPix(
                image, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=criteria
            )
            return True, corners

        return False, None
