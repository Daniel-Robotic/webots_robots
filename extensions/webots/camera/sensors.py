from __future__ import annotations
from typing import List
from controller import Robot, Camera, RangeFinder
from ...utils.device_search import find_devices


class CameraSensorComponent:
    def __init__(self, robot: Robot, timestep: int, verbose: bool = False):
        self.robot = robot
        self._cams: List[Camera] = find_devices(robot, Camera)
        self._rfs: List[RangeFinder] = find_devices(robot, RangeFinder)
        self._verbose = verbose

        for c in self._cams:
            c.enable(timestep)
            c.recognitionEnable(timestep)

        for r in self._rfs:
            r.enable(timestep)

        if verbose:
            print(f"[CameraSensor] cams={len(self._cams)}, range={len(self._rfs)}")

    def __del__(self):
        self.disable_all()

    @property
    def cameras(self):
        return self._cams

    @property
    def range_finders(self):
        return self._rfs

    def disable_all(self):
        for c in self._cams:
            c.recognitionDisable()
            c.disable()

        for r in self._rfs:
            r.disable()