import numpy as np

from pathlib import Path
from typing import Tuple, Union, List, Dict, Any, Optional


ImagePath = Path
CharucoIds = np.ndarray                                                                 # shape: (N, 1), dtype=int32
DistCoeffs = np.ndarray                                                                 # shape: (1, 5) or (5, 1) or other depending on the model
ImagePoints = np.ndarray                                                                # shape: (N, 1, 2), dtype=float32
ObjectPoints = np.ndarray                                                               # shape: (N, 3), dtype=float32
CameraMatrix = np.ndarray                                                               # shape: (3, 3), dtype=float64
ArucoCounter = np.ndarray                                                               # shape: (N,), dtype=int32
CharucoCorners = np.ndarray                                                             # shape: (N, 1, 2), dtype=float32
ImageSize = Tuple[int, int]                                                             # (width, height)
ArucoCorners = List[np.ndarray]                                                         # list of arrays of marker corners
CalibrationResult = Dict[str, Any]                                                      # {'ret', 'matrix', 'distortion', 'rotation_vectors', 'translation_vectors'}
ArucoIds = Union[np.ndarray, List[np.ndarray]]                                          # shape: (N, 1), dtype=int32
FindCornersResult = Tuple[bool, Optional[np.ndarray]]                                   # (success, angles)
FindArucoCornerResult = Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]         # (success, angles, id)
