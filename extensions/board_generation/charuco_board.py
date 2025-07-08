import numpy as np
from cv2 import aruco
from typing import Tuple, Union

from .marker_board import MarkersBoard


class CharucoBoard(MarkersBoard):
    """Class for generating ChArUco (chessboard + ArUco) calibration boards."""
    
    def __init__(
        self, 
        grid_cells: Tuple[int, int], 
        cell_size_mm: int,
        marker_length_mm: float, 
        dpi: int = 600, 
        paper_size_mm: Union[Tuple[int, int], str] = "A3", 
        aruco_dict_name: str = "5x5_250"
    ) -> None:
        """
        Initialize the ChArUco board generator.
        
        Args:
            grid_cells: Number of grid cells (columns, rows)
            cell_size_mm: Size of each cell in millimeters
            marker_length_mm: Size of markers in millimeters
            dpi: Dots per inch for output image
            paper_size_mm: Paper size as standard name (str) or custom dimensions (tuple)
            aruco_dict_name: Name of the ArUco dictionary to use
        """
        super().__init__(grid_cells, cell_size_mm, dpi, paper_size_mm, aruco_dict_name)
        self.marker_length_mm = marker_length_mm

    @property
    def marker_length_mm(self) -> float:
        """Get the marker size in millimeters."""
        return self._marker_length_mm

    @marker_length_mm.setter
    def marker_length_mm(self, value: float) -> None:
        """Set the marker size with validation."""
        if not isinstance(value, (int, float)) or value <= 0:
            raise ValueError("marker_length_mm must be a positive number")
        self._marker_length_mm = float(value)

    def _generate_board_image(self, width: int, height: int) -> np.ndarray:
        """
        Generate a ChArUco board image.
        
        Args:
            width: Width of the board in pixels
            height: Height of the board in pixels
            
        Returns:
            numpy.ndarray: Generated board image
        """
        board = aruco.CharucoBoard(
            size=self._grid_cells,
            squareLength=self._cell_size_mm/1000,
            markerLength=self._marker_length_mm/1000,
            dictionary=self._aruco_dict
        )

        return board.generateImage((width, height))