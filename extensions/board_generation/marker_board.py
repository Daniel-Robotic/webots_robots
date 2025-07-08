from cv2 import aruco
from typing import Tuple, Union, Dict

from .base import BoardCalibrator

class MarkersBoard(BoardCalibrator):
    """
    Base class for marker-based calibration boards (Aruco/Charuco).
    Handles common marker dictionary functionality.
    """
    
    # Supported ArUco dictionary types
    ARUCO_DICTS: Dict[str, int] = {
        "4x4_50": aruco.DICT_4X4_50,
        "5x5_100": aruco.DICT_5X5_100,
        "5x5_250": aruco.DICT_5X5_250,
        "6x6_1000": aruco.DICT_6X6_1000,
        "7x7_1000": aruco.DICT_7X7_1000
    }

    def __init__(
        self, 
        grid_cells: Tuple[int, int], 
        cell_size_mm: int, 
        dpi: int = 600, 
        paper_size_mm: Union[Tuple[int, int], str] = "A3",
        aruco_dict_name: str = "5x5_250"
    ) -> None:
        """
        Initialize the marker board generator.
        
        Args:
            grid_cells: Number of grid cells (columns, rows)
            cell_size_mm: Size of each cell in millimeters
            dpi: Dots per inch for output image
            paper_size_mm: Paper size as standard name (str) or custom dimensions (tuple)
            aruco_dict_name: Name of the ArUco dictionary to use
        """
        super().__init__(grid_cells, cell_size_mm, dpi, paper_size_mm)
        self.aruco_dict_name = aruco_dict_name

    @property
    def aruco_dict_name(self) -> str:
        """Get the name of the ArUco dictionary being used."""
        return self._aruco_dict_name

    @aruco_dict_name.setter
    def aruco_dict_name(self, value: str) -> None:
        """Set the ArUco dictionary with validation."""
        if value not in self.ARUCO_DICTS:
            raise ValueError(f"aruco_dict_name must be one of: {list(self.ARUCO_DICTS.keys())}")
        self._aruco_dict_name = value
        self._aruco_dict = aruco.getPredefinedDictionary(self.ARUCO_DICTS[self._aruco_dict_name])

    @classmethod
    def get_aruco_dict(cls) -> Dict[str, int]:
        """Get the available ArUco dictionary types."""
        return cls.ARUCO_DICTS