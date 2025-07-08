import cv2
import numpy as np
from typing import Tuple, Union

from .base import BoardCalibrator


class CircleGridBoard(BoardCalibrator):
    """Class for generating circle grid calibration patterns."""
    
    def __init__(
        self, 
        grid_cells: Tuple[int, int], 
        cell_size_mm: int, 
        dpi: int = 600, 
        paper_size_mm: Union[Tuple[int, int], str] = "A3",
        asymmetric: bool = False
    ) -> None:
        """
        Initialize the circle grid generator.
        
        Args:
            grid_cells: Number of grid cells (columns, rows)
            cell_size_mm: Size of each cell in millimeters
            dpi: Dots per inch for output image
            paper_size_mm: Paper size as standard name (str) or custom dimensions (tuple)
            asymmetric: Whether to use asymmetric grid pattern
        """
        super().__init__(grid_cells, cell_size_mm, dpi, paper_size_mm)
        self.asymmetric = asymmetric

    @property
    def asymmetric(self) -> bool:
        """Get whether asymmetric grid pattern is enabled."""
        return self._asymmetric

    @asymmetric.setter
    def asymmetric(self, value: bool) -> None:
        """Set asymmetric grid pattern with validation."""
        if not isinstance(value, bool):
            raise ValueError("asymmetric must be a boolean value")
        self._asymmetric = value

    def _generate_board_image(self, width: int, height: int) -> np.ndarray:
        """
        Generate a circle grid pattern image.
        
        Args:
            width: Width of the board in pixels
            height: Height of the board in pixels
            
        Returns:
            numpy.ndarray: Generated board image
        """
        board_img = 255 * np.ones((height, width), dtype=np.uint8)
        
        grid_x, grid_y = self._grid_cells
        radius = int(0.25 * min(width / grid_x, height / grid_y))

        for y in range(grid_y):
            for x in range(grid_x):
                cx = int((x + 0.5) * width / grid_x)
                cy = int((y + 0.5) * height / grid_y)

                if self._asymmetric:
                    cx = int((x + 0.5 + 0.5 * (y % 2)) * width / grid_x)

                if 0 <= cx < width and 0 <= cy < height:
                    cv2.circle(board_img, (cx, cy), radius, 0, -1)

        return board_img
    