import io
import cv2
import numpy as np
from reportlab.pdfgen import canvas
from typing import Tuple, Union, Optional, Dict
from reportlab.lib.utils import ImageReader


class BoardCalibrator:
    """
    Base class for generating calibration boards with various patterns.
    Handles common functionality like paper sizes, DPI settings, and file output.
    """
    
    # Standard paper sizes in millimeters (width, height)
    PAPER_SIZES: Dict[str, Tuple[int, int]] = {
        "A0": (841, 1189), "A1": (594, 841), "A2": (420, 594),
        "A3": (297, 420), "A4": (210, 297), "A5": (148, 210),
        "A6": (105, 148), "A7": (74, 105), "A8": (52, 74),
        "A9": (37, 52), "A10": (26, 37),

        "B0": (1000, 1414), "B1": (707, 1000), "B2": (500, 707),
        "B3": (353, 500), "B4": (250, 353), "B5": (176, 250),
        "B6": (125, 176), "B7": (88, 125), "B8": (62, 88),
        "B9": (44, 62), "B10": (31, 44),

        "C0": (917, 1297), "C1": (648, 917), "C2": (458, 648),
        "C3": (324, 458), "C4": (229, 324), "C5": (162, 229),
        "C6": (114, 162), "C7": (81, 114), "C8": (57, 81),
        "C9": (40, 57), "C10": (28, 40)
    }

    def __init__(
        self,
        grid_cells: Tuple[int, int],
        cell_size_mm: int,
        dpi: int = 600,
        paper_size_mm: Union[Tuple[int, int], str] = "A3"
    ) -> None:
        """
        Initialize the calibration board generator.
        
        Args:
            grid_cells: Number of grid cells (columns, rows)
            cell_size_mm: Size of each cell in millimeters
            dpi: Dots per inch for output image
            paper_size_mm: Paper size as standard name (str) or custom dimensions (tuple)
        """
        self.__pixel2mm = 25.4  # mm per inch (conversion factor)
        self.__mm2pt = 2.83465  # points per mm (conversion factor)
        self.__px2mm: Optional[float] = None

        self.grid_cells = grid_cells
        self.cell_size_mm = cell_size_mm
        self.dpi = dpi
        self.paper_size_mm = paper_size_mm

    @property
    def grid_cells(self) -> Tuple[int, int]:
        """Get the grid dimensions in cells (columns, rows)."""
        return self._grid_cells
    
    @grid_cells.setter
    def grid_cells(self, value: Tuple[int, int]) -> None:
        """Set the grid dimensions with validation."""
        if not isinstance(value, tuple) or len(value) != 2:
            raise TypeError("grid_cells must be a tuple of two integers")

        if not all(isinstance(val, int) and val >= 2 for val in value):
            raise ValueError("Each dimension in grid_cells must be an integer >= 2")

        self._grid_cells = value

    @property
    def cell_size_mm(self) -> int:
        """Get the cell size in millimeters."""
        return self._cell_size_mm
    
    @cell_size_mm.setter
    def cell_size_mm(self, value: int) -> None:
        """Set the cell size with validation."""
        if not isinstance(value, int):
            raise TypeError("cell_size_mm must be an integer")

        if value <= 2:
            raise ValueError("cell_size_mm must be an integer >= 2")

        self._cell_size_mm = value

    @property
    def dpi(self) -> int:
        """Get the output resolution in dots per inch."""
        return self._dpi
    
    @dpi.setter
    def dpi(self, value: int) -> None:
        """Set the output resolution with validation."""
        if not isinstance(value, int):
            raise ValueError("DPI must be an integer")

        self._dpi = value
        self.__px2mm = self._dpi / self.__pixel2mm
    
    @property
    def paper_size_mm(self) -> Tuple[int, int]:
        """Get the paper size in millimeters."""
        return self._paper_size_mm
    
    @paper_size_mm.setter
    def paper_size_mm(self, value: Union[Tuple[int, int], str]) -> None:
        """Set the paper size with validation."""
        if isinstance(value, str):
            if value.upper() not in self.PAPER_SIZES:
                available_sizes = ', '.join(sorted(self.PAPER_SIZES.keys()))
                raise ValueError(
                    f"Unknown paper size '{value}'. Available sizes: {available_sizes} "
                    "or custom size as tuple (width, height) in mm"
                )
            value = self.PAPER_SIZES[value.upper()]

        if not isinstance(value, tuple) or len(value) != 2:
            raise TypeError(
                "paper_size_mm must be either a paper format name (str) "
                "or a tuple of two integers (width, height) in mm"
            )

        if not all(isinstance(dim, int) and dim > 0 for dim in value):
            raise ValueError(
                "Paper dimensions must be positive integers (width, height) in mm"
            )
        
        self._paper_size_mm = value
    
    def _generate_board_image(self, width: int, height: int) -> np.ndarray:
        """
        Abstract method to generate the board image.
        
        Args:
            width: Width of the board in pixels
            height: Height of the board in pixels
            
        Returns:
            numpy.ndarray: Generated board image
        """
        raise NotImplementedError("This method must be implemented in child classes")

    def __compute_canvas_and_scale(self) -> Tuple[np.ndarray, int, int]:
        """
        Compute the canvas size and board dimensions in pixels.
        
        Returns:
            Tuple containing:
                - Canvas image (numpy.ndarray)
                - Board width in pixels (int)
                - Board height in pixels (int)
        """
        paper_w_mm, paper_h_mm = self._paper_size_mm
        canvas_w_px = int(paper_w_mm * self.__px2mm)
        canvas_h_px = int(paper_h_mm * self.__px2mm)

        grid_w, grid_h = self._grid_cells
        board_w_px = int(grid_w * self._cell_size_mm * self.__px2mm)
        board_h_px = int(grid_h * self._cell_size_mm * self.__px2mm)

        canvas = 255 * np.ones((canvas_h_px, canvas_w_px), dtype=np.uint8)

        return canvas, board_w_px, board_h_px
    
    def __save_to_pdf(self, image: np.ndarray, filename: str) -> None:
        """
        Save the board image to a PDF file.
        
        Args:
            image: Board image to save
            filename: Output PDF filename
        """
        paper_w_pt, paper_h_pt = (mm * self.__mm2pt for mm in self._paper_size_mm)

        is_success, buffer = cv2.imencode(".png", image)
        if not is_success:
            raise ValueError("Could not convert image to PNG format")
        
        c = canvas.Canvas(filename, pagesize=(paper_w_pt, paper_h_pt))
        img_reader = ImageReader(io.BytesIO(buffer))

        c.drawImage(img_reader, 0, 0, width=paper_w_pt, height=paper_h_pt)
        c.save()

    def generate_board(self, filename: Optional[str] = None) -> np.ndarray:
        """
        Generate and optionally save the calibration board.
        
        Args:
            filename: Optional output filename (PDF or image format)
            
        Returns:
            numpy.ndarray: Generated board image
            
        Raises:
            ValueError: If filename has unsupported extension or saving fails
        """
        if filename is not None:
            if not isinstance(filename, str):
                raise ValueError("Filename must be a string")
            
            ext = filename.lower().split('.')[-1] if '.' in filename else None
            supported_img_ext = {'png', 'jpg', 'jpeg'}
            
            if ext not in {'pdf'} | supported_img_ext:
                raise ValueError(f"Unsupported file format. Supported: .pdf, {', '.join(supported_img_ext)}")

        canvas, board_w_px, board_h_px = self.__compute_canvas_and_scale()
        board_img = self._generate_board_image(board_w_px, board_h_px)

        board_h_px, board_w_px = board_img.shape[:2]

        offset_x = (canvas.shape[1] - board_w_px) // 2
        offset_y = (canvas.shape[0] - board_h_px) // 2

        canvas[offset_y:offset_y + board_h_px, offset_x:offset_x + board_w_px] = board_img

        if filename:
            ext = filename.lower().split('.')[-1]
            
            if ext == 'pdf':
                self.__save_to_pdf(image=canvas, filename=filename)
            else:
                if not cv2.imwrite(filename, canvas):
                    raise ValueError(f"Failed to save image to {filename}")

        return canvas
