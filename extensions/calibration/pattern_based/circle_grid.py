import cv2
import numpy as np
from pathlib import Path
from typing import Tuple, Union

from ..types import FindCornersResult
from ..pattern_based.base import PatternBasedCalibrator


class CircleGridCalibrator(PatternBasedCalibrator):
    def __init__(
        self,
        image_folder: Union[str, Path],
        pattern_size: Tuple[int, int],
        pattern_length_mm: Union[float, int] = 40,
        asymmetric: bool = False
    ):
        """
        Circle grid calibrator supporting symmetric and asymmetric patterns.

        :param image_folder: Path to folder with imagesfrom ..calibration import (ChessboardCalibrator, 
                        CircleGridCalibrator, 
                        ArucoBoardCalibrator,
                        CharucoBoardCalibrator)
        :param pattern_size: Tuple (cols, rows) of the circle grid
        :param pattern_length_mm: Distance between circle centers in millimeters
        :param asymmetric: Whether the grid is asymmetric
        """
        if not isinstance(asymmetric, bool):
            raise TypeError("asymmetric must be a boolean")

        super().__init__(image_folder, pattern_size, pattern_length_mm)
        self.asymmetric = asymmetric

    @property
    def asymmetric(self) -> bool:
        return self._asymmetric

    @asymmetric.setter
    def asymmetric(self, value: bool) -> None:
        if not isinstance(value, bool):
            raise ValueError("asymmetric must be a boolean")
        self._asymmetric = value

    # TODO: Необходимо почитать подробнее про калибровку маркеров
    # def _prepare_3d_points(self) -> ObjectPoints:
    #     """
    #     Generates 3D object points based on circle grid geometry.

    #     :return: (N, 3) float32 numpy array of 3D points
    #     """
    #     width, height = self._pattern_size

    #     if self._asymmetric:
    #         object_points = []
    #         for row in range(height):
    #             for col in range(width):
    #                 x = col * self._pattern_length
    #                 y = row * self._pattern_length

    #                 # Stagger every second row for asymmetric pattern
    #                 if row % 2 == 1:
    #                     x += self._pattern_length / 2

    #                 object_points.append([x, y, 0])
    #         return np.array(object_points, dtype=np.float32)
    #     else:
    #         objp = np.zeros((height * width, 3), dtype=np.float32)
    #         objp[:, :2] = (
    #             np.mgrid[0:width, 0:height].T.reshape(-1, 2) * self._pattern_length
    #         )
    #         return objp

    def find_corners(self, image: np.ndarray) -> FindCornersResult:
        """
        Detects circle grid pattern in the image.

        :param image: Grayscale image
        :return: Tuple (success flag, detected corners or None)
        """
        if not isinstance(image, np.ndarray):
            raise TypeError("image must be a numpy ndarray")

        if image.ndim != 2:
            raise ValueError("image must be a grayscale image")

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        flags = cv2.CALIB_CB_SYMMETRIC_GRID
        if self._asymmetric:
            flags = cv2.CALIB_CB_ASYMMETRIC_GRID

        found, corners = cv2.findCirclesGrid(image, self._pattern_size, None, flags)
        
        if found and corners is not None:
            corners = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
            return True, corners

        return False, None
