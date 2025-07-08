import os
import cv2
import numpy as np

from datetime import datetime
from controller import Camera, RangeFinder


class CameraRecognitionProcessor:
    def __init__(self, verbose: bool = False):
        self._verbose = verbose

    def process(self, cam: Camera) -> dict:
        w, h = cam.getWidth(), cam.getHeight()
        objs = cam.getRecognitionObjects()

        if self._verbose:
            print(f"[{cam.getName()}] objs={len(objs)}")

        img_rgb = np.frombuffer(cam.getImage(), dtype=np.uint8).reshape((h, w, 4))[:, :, :3]
        out = dict(camera=cam.getName(), image_rgb=img_rgb, objects=[])
        
        for o in objs:
            out["objects"].append({
                "position": list(o.getPosition()),
                "orientation": list(o.getOrientation()),
                "size": list(o.getSize()),
                "model": str(o.getModel()),
                "position_on_image": list(o.getPositionOnImage()),
            })
        
        return out


class CameraImageSaverComponent:
    def _make_filenames(self, camera_name: str, folder: str) -> dict:
        os.makedirs(folder, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        base = os.path.join(folder, f"{camera_name}_{timestamp}")
        return {
            "rgb": f"{base}_rgb.png",
            "depth": f"{base}_depth.png"
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