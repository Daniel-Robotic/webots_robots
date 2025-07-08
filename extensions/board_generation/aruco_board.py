import numpy as np
from cv2 import aruco
from typing import Tuple, Union

from .marker_board import MarkersBoard

class ArucoBoard(MarkersBoard):
    """Class for generating ArUco marker calibration boards."""
    
    def __init__(
        self, 
        grid_cells: Tuple[int, int], 
        cell_size_mm: int,
        marker_length_ratio: float = 0.75,
        dpi: int = 600, 
        paper_size_mm: Union[Tuple[int, int], str] = "A3", 
        aruco_dict_name: str = "5x5_250"
    ) -> None:
        """
        Initialize the ArUco board generator.
        
        Args:
            grid_cells: Number of grid cells (columns, rows)
            cell_size_mm: Size of each cell in millimeters
            marker_length_ratio: Ratio of marker size to cell size (0-1]
            dpi: Dots per inch for output image
            paper_size_mm: Paper size as standard name (str) or custom dimensions (tuple)
            aruco_dict_name: Name of the ArUco dictionary to use
        """
        super().__init__(grid_cells, cell_size_mm, dpi, paper_size_mm, aruco_dict_name)
        self.marker_length_ratio = marker_length_ratio

    @property
    def marker_length_ratio(self) -> float:
        """Get the marker size ratio (marker size / cell size)."""
        return self._marker_length_ratio

    @marker_length_ratio.setter
    def marker_length_ratio(self, value: float) -> None:
        """Set the marker size ratio with validation."""
        if not isinstance(value, (int, float)) or not (0 < value <= 1):
            raise ValueError("marker_length_ratio must be a number in range (0, 1]")
        self._marker_length_ratio = float(value)

    def _generate_board_image(self, width: int, height: int) -> np.ndarray:
        """
        Generate an ArUco board image.
        
        Args:
            width: Width of the board in pixels
            height: Height of the board in pixels
            
        Returns:
            numpy.ndarray: Generated board image
        """
        board = aruco.GridBoard(
            size=self._grid_cells,
            markerLength=self._cell_size_mm * self._marker_length_ratio / 1000,
            markerSeparation=self._cell_size_mm * (1 - self._marker_length_ratio) / 1000,
            dictionary=self._aruco_dict
        )
        
        return board.generateImage((width, height))