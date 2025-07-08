import roboticstoolbox as rtb
from typing import List, Dict


class RobotManipulatorStateComponent:
    def __init__(self,
                 robot_model: rtb.DHRobot):
        self._robot_model = robot_model
        self._current_position = self._robot_model.qz
        self._ee_position = None
        self._ee_orientation_rpy = None

    @property
    def current_position(self):
        return self._current_position

    @property
    def ee_position(self):
        return self._ee_position

    @property
    def ee_orientation_rpy(self):
        return self._ee_orientation_rpy
    
    def update(self, 
                source: str, 
                message_type: str, 
                messages: List[Dict]):
        
        for msg in messages:
            if msg["source"] == source and msg["type"] == message_type:
                self._current_position = list(msg["data"]["joints"].values())[:-1]
                self._solve_pzk()

    def _solve_pzk(self):
        T = self._robot_model.fkine(self._current_position)
        
        self._ee_position = T.t
        self._ee_orientation_rpy = T.rpy()
        