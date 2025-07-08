import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append(str(Path(__file__).resolve().parents[2]))

from controller import Robot
from extensions.webots.communication import WebotsJsonComm
from extensions.webots.camera.sensors import CameraSensorComponent
from extensions.webots.camera.publisher import CameraPublisherComponent
from extensions.webots.camera.processor import (CameraRecognitionProcessor,
                                                CameraImageSaverComponent)

robot = Robot()
ts = int(robot.getBasicTimeStep())
comm = WebotsJsonComm(robot.getDevice(f"{robot.getName()}_receiver"),
                      robot.getDevice(f"{robot.getName()}_emitter"))
comm.enable(ts)


sensors = CameraSensorComponent(robot, ts, verbose=False)
proc = CameraRecognitionProcessor()
pub = CameraPublisherComponent(comm)
saver = CameraImageSaverComponent()

while robot.step(ts) != -1:
    comm.receive()
    for msg in comm.messages:
        if msg.get("type") == "save_image":
            folder = msg.get("data", {}).get("folder", "./images")
            sensor_type = msg.get("data", {}).get("type", "rgb")

            for sensor in sensors.cameras + sensors.range_finders:
                saver.save(sensor, sensor_type, folder)

    results = [proc.process(c) for c in sensors.cameras]
    pub.publish(results)


sensors.disable_all()
comm.disable()

