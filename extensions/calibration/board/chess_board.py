import cv2
import numpy as np

from ..board.base import BoardCalibrator


class ChessBoard(BoardCalibrator):
    """Class for generating chessboard calibration patterns."""
    
    def _generate_board_image(self, width: int, height: int) -> np.ndarray:
        """
        Generate a chessboard pattern image.
        
        Args:
            width: Width of the board in pixels
            height: Height of the board in pixels
            
        Returns:
            numpy.ndarray: Generated board image
        """
        grid_x, grid_y = self._grid_cells
        base = ((np.indices((grid_y, grid_x)).sum(axis=0) % 2) * 255).astype(np.uint8)
        board_img = cv2.resize(base, (width, height), interpolation=cv2.INTER_NEAREST)

        return board_img