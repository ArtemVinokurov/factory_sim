"""box_robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import numpy as np
import tempfile
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

from math import *
from controller import Supervisor
import time
from dataclasses import dataclass, field


# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

joint_str = '#WEBOTS_MOTION,V1.0,pr15_shoulder_pan_joint,pr15_shoulder_lift_joint,pr15_elbow_joint,pr15_wrist1_joint,pr15_wrist2_joint,pr15_wrist3_joint'



def move_point(point, time_move):
    init_pt_str = '00:00:000,Zero,0,0,0,0,0,0'
    with open('move_point.motion', 'w') as f:
        f.write
    

def main():
    filename = "pr15.urdf"
    armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True])
    
    motors = []
    
    for link in armChain.links:
        if 'joint' in link.name:
            motor = supervisor.getDevice(link.name)
            motor.setVelocity(3.14)
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(timeStep)
            motors.append(motor)


if __name__ == '__main__':
    main()


@dataclass
class Time():
    sec : int = 0
    ms : int = 0



class RobotControl():
    def __init__(self):
        urdf_filename = "pr15.urdf"
        self.armChain = Chain.from_urdf_file(urdf_filename, active_links_mask=[False, True, True, True, True, True, True])

        self.supervisor = Supervisor()
        self.timeStep = int(self.supervisor.getBasicTimeStep())
        self.motors = []

        self.currentPos = [0.0]*6

        self.motionFileData = '#WEBOTS_MOTION,V1.0,pr15_shoulder_pan_joint,pr15_shoulder_lift_joint,pr15_elbow_joint,pr15_wrist1_joint,pr15_wrist2_joint,pr15_wrist3_joint'


    def init_robot(self):
        for link in self.armChain.links:
            if 'joint' in link.name:
                motor = self.supervisor.getDevice(link.name)
                motor.setVelocity(3.14)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.timeStep)
                self.motors.append(motor)
        self.step()

    def step(self):
        self.supervisor.step(self.timeStep)
        self.read_sensors()

    def delay(self, delay_time):
        counter = int(delay_time) * 1000 / self.timeStep
        while counter > 0:
            counter -= 1
            self.step()


    def read_sensors(self):
        for motor, pos in zip(self.motors, self.current_pos):
            pos = motor.getPositionSensor().getValue()

    def move_point(self, position, orientation, move_time):
        self.step()
        ikResults = self.armChain.inverse_kinematics(target_position=position, target_orientation=orientation, initial_position=self.currentPos, orientation_mode="Z")
        init_pt_str = '00:00:000,Zero,' + ",".join(map(str, self.currentPos))
        target_pt_str = 
        with open('move_point.motion', 'w') as f:
            f.write




   