from typing import List, Dict
from ..communication import WebotsJsonComm


class CameraPublisherComponent:
    def __init__(self, comm: WebotsJsonComm):
        self._comm = comm

    def publish(self, results: List[Dict]):
        for res in results:
            self._comm.send({
                "source": "camera_node",
                "type": "recognized_objects",
                "data": {
                    "camera": res["camera"],
                    "objects": res["objects"]
                }
            })