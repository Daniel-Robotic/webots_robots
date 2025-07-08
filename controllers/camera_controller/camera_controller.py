import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

from controller import Robot
from extensions.webots.communication import WebotsJsonComm
from extensions.webots.camera.sensors import CameraSensorComponent
from extensions.webots.camera.publisher import CameraPublisherComponent
from extensions.webots.camera.processor import (CameraRecognitionProcessor,
                                                CameraImageSaverComponent,
                                                PatternDetectorComponent)

robot = Robot()
ts = int(robot.getBasicTimeStep())
override_result = None

comm = WebotsJsonComm(robot.getDevice(f"{robot.getName()}_receiver"),
                      robot.getDevice(f"{robot.getName()}_emitter"))
comm.enable(ts)

sensors = CameraSensorComponent(robot, ts, verbose=False)
proc = CameraRecognitionProcessor()
pub = CameraPublisherComponent(comm, robot)
saver = CameraImageSaverComponent()
pattern_detector = PatternDetectorComponent(camera_sensors=sensors)

while robot.step(ts) != -1:
    image_pattern = None
    comm.receive()
    
    for msg in comm.messages:
        if msg.get("type") == "save_image":
            folder = msg.get("data", {}).get("folder", "./images")
            sensor_type = msg.get("data", {}).get("type", "rgb")
            for sensor in sensors.cameras + sensors.range_finders:
                saver.save(sensor, sensor_type, folder)

        elif msg.get("type") == "pattern_detection":
            pattern_type = msg["data"].get("pattern_type", "")
            params = msg["data"].get("params", {})
            image_pattern = pattern_detector.detect_and_overlay(pattern_type, params)

    results = [proc.process(c) for c in sensors.cameras]
    pub.publish(results, image_pattern=image_pattern)


sensors.disable_all()
comm.disable()

