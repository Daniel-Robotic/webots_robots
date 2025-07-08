import cv2
import numpy as np
from typing import List, Dict, Optional
from controller import Robot, Display
from extensions.webots.communication import WebotsJsonComm


class CameraPublisherComponent:
    def __init__(self, comm: WebotsJsonComm, robot: Robot):
        self._comm = comm
        self._robot = robot

    def publish(self, results: List[Dict], image_pattern: Optional[Dict] = None):
        for res in results:
            self._comm.send({
                "source": "camera_node",
                "type": "recognized_objects",
                "data": {
                    "camera": res["camera"],
                    "objects": res.get("objects", [])
                }
            })

            # Показываем изображение от распознавания, если есть
            if "image" in res and "width" in res and "height" in res:
                self._display_image(
                    camera_name=res["camera"],
                    image=res["image"],
                    width=res["width"],
                    height=res["height"]
                )

        # Также показываем изображение от pattern_detector, если передано
        if image_pattern:
            self._display_image(
                camera_name=image_pattern["camera"],
                image=image_pattern["image"],
                width=image_pattern["image"].shape[1],
                height=image_pattern["image"].shape[0]
            )

    def _display_image(self, camera_name: str, image: np.ndarray, width: int, height: int):
        if image.ndim == 3 and image.shape[2] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGBA)

        image_bytes = image.tobytes()
        display_name = camera_name.replace("_rgb", "_display")

        try:
            display: Display = self._robot.getDevice(display_name)
            ir = display.imageNew(image_bytes, Display.BGRA, width, height)
            display.imagePaste(ir, 0, 0, False)
            display.imageDelete(ir)
        except Exception as e:
            print(f"[Display Error] Could not update {display_name}: {e}")
