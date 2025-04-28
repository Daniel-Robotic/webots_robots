"""camera_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, RangeFinder

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

rgb_camera: Camera = robot.getDevice("IntelRealsenseD455_rgb")
rgb_camera.enable(timestep)

depth_camera: RangeFinder = robot.getDevice("IntelRealsenseD455_depth")
depth_camera.enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    pass

# Enter here exit cleanup code.

rgb_camera.disable()
depth_camera.disable()