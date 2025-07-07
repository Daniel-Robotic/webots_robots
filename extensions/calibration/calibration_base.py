import cv2
import json
import numpy as np

from pathlib import Path
from typing import Tuple, Union, List, Any
from .types import ImageSize, ImagePath, Optional, Dict


class CameraBaseCalibrator:
    def __init__(self,
                image_folder: Union[str, Path],
                pattern_size: Tuple[int, int]) -> None:
        """
        Base abstract class for camera calibration.

        :param image_folder: Path to folder containing calibration images
        :param pattern_size: Pattern size as (columns, rows) of inner corners
        """

        self.image_folder = image_folder
        self.pattern_size = pattern_size
        self._images_size: ImageSize = ()
        self._calibration_result: Optional[Dict[str, Any]] = None

    @property
    def image_folder(self) -> Path:
        return self._image_folder

    @image_folder.setter
    def image_folder(self, value: Union[str, Path]) -> None:
        if isinstance(value, str):
            value = Path(value)

        if not isinstance(value, Path):
            raise TypeError("image_folder must be a string or Path object")

        if not value.exists():
            raise ValueError(f"The specified image folder does not exist: {value}")

        if not value.is_dir():
            raise ValueError(f"The specified path is not a directory: {value}")

        self._image_folder = value

    @property
    def pattern_size(self) -> Tuple[int, int]:
        return self._pattern_size

    @pattern_size.setter
    def pattern_size(self, value: Tuple[int, int]) -> None:
        if not isinstance(value, tuple) or len(value) != 2:
            raise TypeError("pattern_size must be a tuple of two integers")

        if not all(isinstance(val, int) and val >= 2 for val in value):
            raise ValueError("Each dimension in pattern_size must be an integer >= 2")

        self._pattern_size = value

    def _check_images_size(self,
                            pattern: str = "*") -> Tuple[ImageSize, List[ImagePath]]:
        """
        Ensures all images in the folder have the same resolution.

        :param pattern: Glob pattern to match image files
        :return: Tuple of reference image size and list of valid image paths
        """
        if not isinstance(pattern, str):
            raise TypeError("pattern must be a string")

        image_sizes: List[ImageSize] = []
        image_paths: List[ImagePath] = list(self._image_folder.glob(pattern))

        if not image_paths:
            raise ValueError("No images found with the given pattern")

        for img_path in image_paths:
            if not img_path.is_file():
                print(f"Skipped non-file path: {img_path}")
                continue

            image = cv2.imread(str(img_path))
            if image is None:
                print(f"Failed to load image: {img_path.name}")
                continue

            height, width = image.shape[:2]
            image_sizes.append((width, height))

        if not image_sizes:
            raise ValueError("No valid images found for calibration")

        ref_size = image_sizes[0]
        for idx, size in enumerate(image_sizes):
            if size != ref_size:
                raise ValueError(
                    f"Image at index {idx} has different resolution: {size} vs {ref_size}"
                )

        return ref_size, image_paths

    # TODO: Починить загрузку и сохранение в файл
    def save_calibration_result(self,
                                output_path: Union[str, Path]) -> None:
        """
        Save calibration results to file. Format is determined automatically by file extension.

        Supported formats:
            - .npz: Binary numpy format
            - .json: Human-readable JSON format
            - .txt: Simple text format with matrices

        :param output_path: Path to save the result
        """

        if self._calibration_result is None:
            raise RuntimeError("No calibration data available. Run 'calibrate()' first.")

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        ext = output_path.suffix.lower()

        if ext == ".npz":
            self.__save_npz(output_path)
        elif ext == ".json":
            self.__save_json(output_path)
        elif ext == ".txt":
            self.__save_txt(output_path)
        else:
            raise ValueError(f"Unsupported file extension: {ext}. Use .npz, .json or .txt")

    def __save_npz(self, path: Path) -> None:
        """Save result as compressed .npz file"""
        np.savez_compressed(path, **self._calibration_result)

    def __save_json(self, path: Path) -> None:
        """Save result as JSON file (supports basic types and numpy arrays/dtypes only)"""
        
        def serialize(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, np.generic):
                return obj.item()
            elif isinstance(obj, dict):
                return {key: serialize(value) for key, value in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [serialize(value) for value in obj]
            else:
                return obj

        serializable_data = serialize(self._calibration_result)

        with open(path, "w") as f:
            json.dump(serializable_data, f, indent=4)

    def __save_txt(self, path: Path) -> None:
        """Save result in human-readable text format"""
        with open(path, "w") as f:
            for key, value in self._calibration_result.items():
                f.write(f"{key.upper()}:\n")
                if isinstance(value, np.ndarray):
                    np.savetxt(f, value, fmt="%.10f")
                else:
                    f.write(f"{value}\n")
                f.write("\n")

    def load_calibration_result(self,
                                input_path: Union[str, Path]) -> Dict[str, Any]:
        """
        Load calibration results from file and store in self._calibration_result.
        Format is determined automatically by file extension.
        
        Supported formats:
            - .npz: Binary numpy format
            - .json: Human-readable JSON format
            - .txt: Simple text format with matrices
        
        Args:
            input_path: Path to load the result from
            
        Returns:
            Dictionary with loaded calibration data
            
        Raises:
            FileNotFoundError: If file doesn't exist
            ValueError: For unsupported formats or invalid data
            RuntimeError: If loaded data is invalid
        """
        input_path = Path(input_path)
        if not input_path.exists():
            raise FileNotFoundError(f"Calibration file not found: {input_path}")
        
        ext = input_path.suffix.lower()
        
        try:
            if ext == ".npz":
                result = self.__load_npz(input_path)
            elif ext == ".json":
                result = self.__load_json(input_path)
            elif ext == ".txt":
                result = self.__load_txt(input_path)
            else:
                raise ValueError(f"Unsupported file format: {ext}. Use .npz, .json or .txt")
            
            self.__validate_calibration_data(result)
            self._calibration_result = result
            return result
            
        except Exception as e:
            raise RuntimeError(f"Failed to load calibration data: {str(e)}") from e

    def __load_npz(self, path: Path) -> Dict[str, Any]:
        """Load result from .npz file with validation"""
        with np.load(path, allow_pickle=False) as data:
            result = {key: data[key] for key in data.files}
        
        # Convert numpy arrays to proper types
        for key in result:
            if isinstance(result[key], np.ndarray):
                result[key] = result[key].astype(np.float64)
        
        return result

    def __load_json(self, path: Path) -> Dict[str, Any]:
        """Load and validate JSON calibration data"""
        
        def _convert_arrays(obj):
            if isinstance(obj, list):
                arr = np.array(obj, dtype=np.float64)
                if arr.ndim == 0:  # Handle scalar values
                    return float(arr)
                return arr
            elif isinstance(obj, dict):
                return {k: _convert_arrays(v) for k, v in obj.items()}
            return obj
        
        with open(path, "r") as f:
            data = json.load(f)
        
        return _convert_arrays(data)

    def __load_txt(self, path: Path) -> Dict[str, Any]:
        """Load from text format with validation"""
        result = {}
        current_key = None
        current_data = []
        
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                
                if line.endswith(":"): 
                    if current_key is not None:
                        self.__store_txt_data(result, current_key, current_data)
                    current_key = line[:-1].lower()
                    current_data = []
                elif line and current_key is not None:
                    current_data.append([float(x) for x in line.split()])
        
        if current_key is not None:
            self.__store_txt_data(result, current_key, current_data)
        
        return result

    def __store_txt_data(self, result: dict, key: str, data: list) -> None:
        """Helper to store parsed text data with proper formatting"""
        if not data:
            return
            
        if len(data) == 1 and len(data[0]) == 1:
            result[key] = data[0][0]
        else:
            result[key] = np.array(data, dtype=np.float64)

    def __validate_calibration_data(self, data: dict) -> None:
        """Validate loaded calibration data structure"""
        required_keys = {'matrix', 'distortion'}
        if not required_keys.issubset(data.keys()):
            raise ValueError(f"Loaded data is missing required keys: {required_keys - set(data.keys())}")
        
        if not isinstance(data['matrix'], np.ndarray) or data['matrix'].shape != (3, 3):
            raise ValueError("Invalid camera matrix in loaded data")
        
        if not isinstance(data['distortion'], np.ndarray):
            raise ValueError("Invalid distortion coefficients in loaded data")
    
    def _euler_to_rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Convert Euler angles (roll, pitch, yaw) to a 3x3 rotation matrix.
        
        The rotation follows the Tait-Bryan convention (Z-Y-X) where:
        - Roll is rotation around X-axis (bank)
        - Pitch is rotation around Y-axis (attitude)
        - Yaw is rotation around Z-axis (heading)
        
        Args:
            roll: Rotation around X-axis in radians
            pitch: Rotation around Y-axis in radians
            yaw: Rotation around Z-axis in radians
            
        Returns:
            np.ndarray: 3x3 rotation matrix
            
        Raises:
            ValueError: If any input angle is not a float number
            TypeError: If any input is not a numerical type
        """
        
        if not isinstance(roll, (float, int)):
            raise TypeError(f"Roll angle must be numeric, got {type(roll)}")
        if not isinstance(pitch, (float, int)):
            raise TypeError(f"Pitch angle must be numeric, got {type(pitch)}")
        if not isinstance(yaw, (float, int)):
            raise TypeError(f"Yaw angle must be numeric, got {type(yaw)}")
        
        roll = float(roll)
        pitch = float(pitch)
        yaw = float(yaw)
        
        R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
        
        return R_z @ R_y @ R_x

    @staticmethod
    def _handeye_calibration_decorator(method):

        def wrapper(self, 
                    gripper_poses: List[Tuple], 
                    pattern: str="*",
                    calib_method: str="DANIILIDIS"):
            
            if self._calibration_result is None:
                raise ValueError("Camera calibration must be performed first")
            
            if not isinstance(gripper_poses, list):
                raise TypeError("gripper_poses must be a list")
                
            if not isinstance(pattern, str):
                raise TypeError("pattern must be a string")

            _, images = self._check_images_size(pattern=pattern)
            
            if len(images) != len(gripper_poses):
                raise ValueError("Number of images must match number of gripper poses")
            
            if self._calibration_result["matrix"].shape != (3, 3):
                raise ValueError("Invalid camera matrix in calibration result")

            R_gripper2base = []
            t_gripper2base = []
            R_target2cam = []
            t_target2cam = []
            
            for img, pose in zip(images, gripper_poses):
                if isinstance(img, (str, Path)):
                    img = cv2.imread(str(img))
                    if img is None:
                        print(f"Failed to load image: {img}")
                        continue

                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                target_pose = method(self, gray)
                if target_pose is None:
                    continue

                R_target, t_target = target_pose
                x, y, z, roll, pitch, yaw = pose
                t_gripper = np.array([x, y, z]).reshape(3, 1)
                R_gripper = self._euler_to_rotation_matrix(roll, pitch, yaw)

                R_target2cam.append(R_target)
                t_target2cam.append(t_target)
                R_gripper2base.append(R_gripper)
                t_gripper2base.append(t_gripper)

            if len(R_gripper2base) < 5:
                raise ValueError(f"Insufficient valid samples: {len(R_gripper2base)}/5")
            
            method_map = {
                "DANIILIDIS": "CALIB_HAND_EYE_DANIILIDIS",
                "PARK": "CALIB_HAND_EYE_PARK",
                "TSAI": "CALIB_HAND_EYE_TSAI",
                "ANDREFF": "CALIB_HAND_EYE_ANDREFF",
                "HORAUD": "CALIB_HAND_EYE_HORAUD"
            }

            method_upper = calib_method.upper()
            if method_upper not in method_map:
                raise ValueError(f"Unknown hand-eye calibration method '{method}'. "
                                f"Available: {list(method_map.keys())}")

            cv2_enum_name = method_map[method_upper]

            if not hasattr(cv2, cv2_enum_name):
                available = [k for k, v in method_map.items() if hasattr(cv2, v)]
                raise RuntimeError(
                    f"Your OpenCV version does not support method '{method}' "
                    f"(cv2.{cv2_enum_name} not found).\n"
                    f"Available in your build: {available}"
                )

            method_enum = getattr(cv2, cv2_enum_name)

            return cv2.calibrateHandEye(
                R_gripper2base=R_gripper2base,
                t_gripper2base=t_gripper2base,
                R_target2cam=R_target2cam,
                t_target2cam=t_target2cam,
                method=method_enum
            )
        
        return wrapper

    def _preprocess_images(self) -> Any:
        """
        Should be implemented by subclasses.
        """
        raise NotImplementedError("Calibration must be implemented in a derived class.")

    def calibrate(self) -> Any:
        """
        Should return calibration parameters such as matrix and distortion coefficients.
        """
        raise NotImplementedError("Calibration must be implemented in a derived class.")

    def handeye_calibrate(self) -> Any:
        """
        Should  return handeye calibration parametrs such as rotation matrix and translation vector (camera->gripper)
        """
        raise NotImplementedError("Calibration must be implemented in a derived class.")

    def find_corners(self) -> Any:
        """
        Should return detected corner coordinates.
        """
        raise NotImplementedError("Calibration must be implemented in a derived class.")
    