import os
import cv2
import numpy as np

from datetime import datetime
from typing import Optional, Dict, Tuple
from controller import Camera, RangeFinder, Display


class CameraRecognitionProcessor:
    def __init__(self, verbose: bool = False):
        self._verbose = verbose

    def process(self, cam: Camera) -> dict:
        w, h = cam.getWidth(), cam.getHeight()
        objs = cam.getRecognitionObjects()
        img_bgr = np.frombuffer(cam.getImage(), dtype=np.uint8).reshape((h, w, 4))[:, :, :3]
        img_bgr = cv2.cvtColor(img_bgr, cv2.COLOR_RGB2BGR)

        return {
            "camera": cam.getName(),
            "image": img_bgr,
            "width": w,
            "height": h,
            "objects": [{
                "position": list(o.getPosition()),
                "orientation": list(o.getOrientation()),
                "size": list(o.getSize()),
                "model": str(o.getModel()),
                "position_on_image": list(o.getPositionOnImage()),
            } for o in objs]
        }


class CameraImageSaverComponent:
    def _make_filenames(self, camera_name: str, folder: str) -> dict:
        os.makedirs(folder, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        base = os.path.join(folder, f"{camera_name}_{timestamp}")
        return {
            "rgb": f"{base}_rgb.jpg",
            "depth": f"{base}_depth.jpg"
        }

    def save(self, sensor, sensor_type: str = "rgb", folder: str = None):
        folder = folder or self._default_dir
        sensor_name = sensor.getName()
        filenames = self._make_filenames(sensor_name, folder)

        if isinstance(sensor, Camera):
            if sensor_type in ("rgb", "rgbd"):
                width, height = sensor.getWidth(), sensor.getHeight()
                img = np.frombuffer(sensor.getImage(), np.uint8).reshape((height, width, 4))
                cv2.imwrite(filenames["rgb"], img)

        if isinstance(sensor, RangeFinder):
            if sensor_type in ("depth", "rgbd"):
                width, height = sensor.getWidth(), sensor.getHeight()
                depth = np.array(sensor.getRangeImage()).reshape((height, width)).astype(np.float32)
                normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                depth_img = np.uint8(normalized)
                cv2.imwrite(filenames["depth"], depth_img)


class PatternDetectorComponent:
    def __init__(self, camera_sensors):
        self._camera_sensors = camera_sensors

    def detect_and_overlay(self, pattern_type: str, params: Dict) -> Optional[Dict[str, np.ndarray]]:
        for camera in self._camera_sensors.cameras:
            width = camera.getWidth()
            height = camera.getHeight()

            rgba_image = np.frombuffer(camera.getImage(), np.uint8).reshape((height, width, 4))
            image = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2BGR)

            if pattern_type == "aruco":
                overlay = self._detect_aruco(image, **params)
            elif pattern_type == "charuco":
                overlay = self._detect_charuco(image, **params)
            elif pattern_type == "chessboard":
                overlay = self._detect_chessboard(image, **params)
            elif pattern_type == "circlegrid":
                overlay = self._detect_circlegrid(image, **params)
            else:
                raise ValueError(f"Unsupported pattern type: {pattern_type}")

            return {"camera": camera.getName(), "image": overlay}

        return None

    def _detect_aruco(self, image: np.ndarray, aruco_dict_name: str = "5x5_250") -> np.ndarray:
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f'DICT_{aruco_dict_name.upper()}'))
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                image = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
        except Exception as e:
            print(f"Error detecting Aruco pattern: {e}")
        finally:
            return image

    def _detect_charuco(self, image: np.ndarray,
                       grid_cells: Tuple[int, int],
                       cell_size_mm: int,
                       marker_length_mm: float,
                       aruco_dict_name: str = "5x5_250") -> np.ndarray:
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f'DICT_{aruco_dict_name.upper()}'))
            board = cv2.aruco.CharucoBoard(grid_cells, cell_size_mm / 1000, marker_length_mm / 1000, aruco_dict)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None and len(corners) > 0:
                _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    markerCorners=corners,
                    markerIds=ids,
                    image=image,
                    board=board
                )
                image = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
                if charuco_corners is not None and charuco_ids is not None:
                    image = cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids)
        except Exception as e:
            print(f"Error detecting Charuco pattern: {e}")
        finally:
            return image

    def _detect_chessboard(self, image: np.ndarray, grid_cells: Tuple[int, int]) -> np.ndarray:
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, grid_cells)
            if ret:
                image = cv2.drawChessboardCorners(image.copy(), grid_cells, corners, ret)
        except Exception as e:
            print(f"Error detecting Chessboard pattern: {e}")
        finally:
            return image

    def _detect_circlegrid(self, image: np.ndarray, grid_cells: Tuple[int, int], asymmetric: bool = False) -> np.ndarray:
        try:
            flags = cv2.CALIB_CB_ASYMMETRIC_GRID if asymmetric else cv2.CALIB_CB_SYMMETRIC_GRID
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, centers = cv2.findCirclesGrid(gray, grid_cells, None, flags)
            if ret:
                image = cv2.drawChessboardCorners(image.copy(), grid_cells, centers, ret)
        except Exception as e:
            print(f"Error detecting CircleGrid pattern: {e}")
        finally:
            return image
    