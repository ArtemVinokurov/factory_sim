"""inverse_kinematics controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot

# timestep = int(robot.getBasicTimeStep())

import sys
import tempfile
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')


IKPY_MAX_ITERATIONS = 4

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
filename = "pr15.urdf"
# with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    # filename = file.name
    # file.write(supervisor.getUrdf().encode('utf-8'))
print(filename)
armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True])

# Initialize the arm motors and encoders.
motors = []

for link in armChain.links:
    if 'joint' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)

# Get the arm and target nodes.
target = supervisor.getFromDef('TARGET')
arm = supervisor.getSelf()
print(len(motors))
# Loop 1: Draw a circle on the paper sheet.
print('Draw a circle on the paper sheet...')
while supervisor.step(timeStep) != -1:
    t = supervisor.getTime()

    # Use the circle equation relatively to the arm base as an input of the IK algorithm.
    # x = 0.1 * math.cos(t) + 0.55
    # y = 0.1 * math.sin(t) - 0.0
    # z = 0.1
    x = 0.0
    y = -0.6
    z = 0.3
    # Call "ikpy" to compute the inverse kinematics of the arm.
    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors]
    ikResults = armChain.inverse_kinematics([x, y, z], [-1.0, 1.0, 0.0], "X")
    print(ikResults)
    
    # Actuate the 3 first arm motors with the IK results.
    for i in range(6):
        motors[i].setPosition(ikResults[i + 1])
    # Keep the hand orientation down.
    # motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
    # Keep the hand orientation perpendicular.
    # motors[5].setPosition(ikResults[1])


