"""supervisor_controller controller."""
from controller import Robot, Supervisor

robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
    pass

