import re
import argparse

from pathlib import Path
from extensions.board_generation.aruco_board import ArucoBoard
from extensions.board_generation.chess_board import ChessBoard
from extensions.board_generation.charuco_board import CharucoBoard
from extensions.board_generation.circlegrid_board import CircleGridBoard    


def update_proto_file(proto_path: str, texture_path: str, size_mm: tuple[float, float], depth_m: float = 0.005):
    """
    Обновляет поля `textureUrl` и `size` в .proto-файле

    :param proto_path: путь до CalibrationPattern.proto
    :param texture_path: путь до изображения (относительно proto-файла или абсолютный)
    :param size_mm: (ширина, высота) в миллиметрах
    :param depth_m: толщина пластины в метрах (по умолчанию 5 мм)
    """
    proto_file = Path(proto_path)
    if not proto_file.exists():
        raise FileNotFoundError(f".proto file not found: {proto_file}")

    width_m = size_mm[0] / 1000
    height_m = size_mm[1] / 1000

    size_string = f"{width_m:.6f} {height_m:.6f} {depth_m:.6f}"
    text = proto_file.read_text()

    text = re.sub(
        r'(field\s+SFString\s+textureUrl\s+)"[^"]+"',
        lambda m: f'{m.group(1)}"{texture_path}"',
        text
    )

    text = re.sub(
        r'(field\s+SFVec3f\s+size\s+)[^\n]+',
        lambda m: f'{m.group(1)}{size_string}',
        text
    )

    proto_file.write_text(text)
    print(f"[✔] Updated .proto: textureUrl={texture_path}, size={size_string}")


def main():
    parser = argparse.ArgumentParser(description="Generate a calibration pattern image.")

    parser.add_argument(
        "--type", "-t",
        type=str,
        required=True,
        choices=["charuco", "aruco", "chessboard", "circlegrid"],
        help="Type of calibration pattern"
    )

    parser.add_argument(
        "--output_image", "-oi",
        type=str,
        required=True,
        default="./protos/textures/handeye_pattern.png",
        help="Path to output image (e.g., ./textures/handeye_pattern.png)"
    )

    parser.add_argument(
        "--proto_path", "-pp",
        type=str,
        default="./protos/CalibrationPattern.proto",
        help="Path to .proto (e.g., ./protos/CalibrationPattern.proto)"
    )

    parser.add_argument(
        "--grid_cells", "-gc",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        default=(5, 7),
        help="Grid size of the pattern as (width height)"
    )

    parser.add_argument(
        "--cell_size", "-cs",
        type=int,
        default=40,
        help="Cell size of the pattern in pixels or mm depending on context"
    )

    parser.add_argument(
        "--dpi",
        type=int,
        default=600,
        help="Dots per inch for output image"
    )

    parser.add_argument(
        "--paper_format", "-pf",
        type=str,
        default="A4",
        help="Paper format (e.g., A4, A3)"
    )

    parser.add_argument(
        "--aruco_dict", "-ad",
        choices=["4x4_50", "5x5_100", "5x5_250", "6x6_1000", "7x7_1000"],
        type=str,
        default="5x5_250",
        help="Aruco dictionary type"
    )

    parser.add_argument(
        "--marker_length_ratio", "-mlr",
        type=float,
        default=0.75,
        help="Relative size of inner ArUco marker to square size (only for Charuco)"
    )

    parser.add_argument(
        "--marker_length", "-ml",
        type=int,
        default=25,
        help="Absolute marker size in mm (only for ArUco)"
    )

    parser.add_argument(
        "--asymmetric", "-as",
        action="store_true",
        help="Use asymmetric circle grid (default: symmetric)"
    )

    args = parser.parse_args()

    # Выбор шаблона
    if args.type == "charuco":
        board = CharucoBoard(
            grid_cells=tuple(args.grid_cells),
            cell_size_mm=args.cell_size,
            marker_length_mm=args.marker_length,
            dpi=args.dpi,
            paper_size_mm=args.paper_format,
            aruco_dict_name=args.aruco_dict
        )
    elif args.type == "aruco":
        board = ArucoBoard(
            grid_cells=tuple(args.grid_cells),
            cell_size_mm=args.cell_size,
            marker_length_ratio=args.marker_length_ratio,
            dpi=args.dpi,
            paper_size_mm=args.paper_format,
            aruco_dict_name=args.aruco_dict
        )
    elif args.type == "chessboard":
        board = ChessBoard(
            grid_cells=tuple(args.grid_cells),
            cell_size_mm=args.cell_size,
            dpi=args.dpi,
            paper_size_mm=args.paper_format
        )
    elif args.type == "circlegrid":
        board = CircleGridBoard(
            grid_cells=tuple(args.grid_cells),
            cell_size_mm=args.cell_size,
            dpi=args.dpi,
            asymmetric=args.asymmetric
        )
    else:
        raise ValueError(f"Unsupported pattern type: {args.type}")

    board.generate_board(str(args.output_image))
    print(f"[✔] Calibration pattern saved to: {args.output_image}")

    proto_dir = Path(args.proto_path).parent
    texture_path_relative = str(Path(args.output_image).resolve().relative_to(proto_dir.resolve()))
    update_proto_file(proto_path=args.proto_path,
                      texture_path=texture_path_relative,
                      size_mm=board.paper_size_mm,
                      depth_m=0.005)
    print(f"[✔] Protofile calibration pattern updated to: {args.proto_path}")

if __name__ == "__main__":
    main()


# python generate_calibration_pattern.py \
# --type charuco \ 
# -oi protos/textures/handeye_pattern.png \
# -gc 5 7 -cs 55 -ml 40 --dpi 50 -pf A3 -ad 4x4_50
