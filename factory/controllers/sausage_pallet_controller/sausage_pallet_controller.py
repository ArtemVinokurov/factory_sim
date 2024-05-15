"""sausage_pallet controller."""
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
from controller import Supervisor, Robot
import time
from controller import VacuumGripper


supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())
q_dot_max = 3.14
tool_offset = 0.14

box_size = 0.25

box_number = 12

def delay(time):
    counter = time * 1000 / timeStep
    while counter > 0:
        counter -= 1
        supervisor.step(timeStep)


def calc_time(q, q0):
    time_move = 0
    for i in range(6):
        time_move_new = 3 * (abs(q[i] - q0[i])) / (2 * q_dot_max)
        if time_move_new > time_move:
            time_move = time_move_new
    return time_move

def calc_coeff(q, q_ref, time_move):
    mat_coeff = []
    for i in range(6):
        a0 = q_ref[i]
        a1 = 0
        a2 = 3 * (q[i] - q_ref[i]) / pow(time_move, 2)
        a3 = -2 * (q[i] - q_ref[i]) / pow(time_move, 3)
        row = [a0, a1, a2, a3]
        mat_coeff.append(row)
    return mat_coeff





def move_point(motors, chain, position, orientation, rot=None):
    init_pos = [0] + [m.getPositionSensor().getValue() for m in motors]
    if orientation == "down":
        ikResults = chain.inverse_kinematics(target_position=position, target_orientation=[0.0, 0.0, 1.0], initial_position=init_pos, orientation_mode="Z")
    elif orientation == "side":
        ikResults = chain.inverse_kinematics(target_position=position, target_orientation=[0.0, 1.0, 0.0], initial_position=init_pos, orientation_mode="Z")
    elif orientation == "all":
        ikResults = chain.inverse_kinematics(target_position=position, target_orientation=rot, initial_position=init_pos, orientation_mode="all")
    q = ikResults[1:]
    q_ref = [0.0] * 6
    
    supervisor.step(timeStep)
    for i in range(6):
        q_ref[i] = motors[i].getPositionSensor().getValue()
 

    time_move = calc_time(q, q_ref)
    coeff_a = calc_coeff(q, q_ref, time_move)
    start_time = supervisor.getTime()
    
    execute = True
    while execute:
        ref_t = supervisor.getTime() - start_time
        for i in range(6):
            cmd = coeff_a[i][0] + coeff_a[i][1] * ref_t + coeff_a[i][2] * pow(ref_t, 2) + coeff_a[i][3] * pow(ref_t, 3)
            motors[i].setPosition(cmd)
        for i in range(6):
            supervisor.step(timeStep)
            current_pos = motors[i].getPositionSensor().getValue()
            if abs(current_pos - q[i]) > 0.0001:
                execute = True
                break
            else:
                execute = False
        if ref_t > time_move + 0.01*time_move:
            execute = False
        supervisor.step(timeStep)

def move_joints(joint_angles, motors):
    q_ref = [0.0] * 6
    for i in range(6):
        q_ref[i] = motors[i].getPositionSensor().getValue()
    q = joint_angles
    time_move = calc_time(q, q_ref)
    coeff_a = calc_coeff(q, q_ref, time_move)
    start_time = supervisor.getTime()
    
    execute = True
    while execute:
        ref_t = supervisor.getTime() - start_time
        for i in range(6):
            cmd = coeff_a[i][0] + coeff_a[i][1] * ref_t + coeff_a[i][2] * pow(ref_t, 2) + coeff_a[i][3] * pow(ref_t, 3)
            motors[i].setPosition(cmd)
        for i in range(6):
            supervisor.step(timeStep)
            current_pos = motors[i].getPositionSensor().getValue()
            if abs(current_pos - q[i]) > 0.001:
                execute = True
                break
            else:
                execute = False
        if ref_t > time_move + 0.01*time_move:
            execute = False
        supervisor.step(timeStep)    


def move_to_init_pos(motors):
    init_pos = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
    for i in range(6):
        motors[i].setPosition(init_pos[i])
        

def main():


    # place_box_pos_node = supervisor.getFromDef("place_box_pos")
    # place_box_pos_ref = place_box_pos_node.getField("translation")
    # place_pos_init = place_box_pos_ref.getSFVec3f()
    
    # targets_point = [place_pos_init] * 12
    
    # x = place_pos_init[0]
    # y = place_pos_init[1]
    # z = place_pos_init[2] + tool_offset + 0.05
    
    
    # targets_point = [[x,y,z], [x-0.25, y, z], [x-0.5,y,z],
                     # [x-0.5,y-0.27,z], [x-0.25, y-0.27, z], [x,y-0.27,z],
                     # [x,y,z], [x-0.25, y, z], [x-0.5,y,z],
                     # [x-0.5,y-0.25,z], [x-0.25, y-0.25, z], [x,y-0.25,z]]
    # iterator = 1

        
                
        
    
    

    filename = "pr15.urdf"
    armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True])
    
    motors = []
    
    for link in armChain.links:
        print(link.name)
        if 'joint' in link.name:
            motor = supervisor.getDevice(link.name)
            motor.setVelocity(3.14)
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(timeStep)
            motors.append(motor)

    ps_1 = supervisor.getDevice("sausage_detector")
    ps_1.enable(timeStep)
    
    vacuum = supervisor.getDevice('vacuum gripper')
    vacuum.enablePresence(timeStep)
    supervisor.step(timeStep)
    move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 0.0], motors)
    
    count_box = 0
    
    grab_box_point = [0.58, -0.03, 0.47]
    up_grab_point = [0.54, -0.03, 0.6]
    start_point = [0.58, -0.03, 0.51] 
    while supervisor.step(timeStep) != -1:
            continue
        # if ps_1.getValue() < 400:
            
            # aim_pt = targets_point[count_box]
            # rot = np.array([[0,-1,0], [-1,0,0], [0,0,1]])
            # move_point(motors, armChain, start_point, "all", rot)
            # delay(0.5)
            # move_point(motors, armChain, grab_box_point, "all", rot)
            
            # vacuum.turnOn()
            # delay(1.0)
            # move_point(motors, armChain, up_grab_point, "all", rot)
            # move_joints([1.57, 0.0, 1.57, 0.0, 1.57, 0.0], motors)
            # delay(0.5)
            # rot = np.array([[1,0,0], [0,-1,0], [0,0,1]])
            
            # middle_point = [aim_pt[0], aim_pt[1], aim_pt[2] + 0.3]
           # middle_point[2] += 0.1
            # move_point(motors, armChain, middle_point, "all", rot)
            # delay(1)
            # move_point(motors, armChain, aim_pt, "all", rot)
            # vacuum.turnOff()
            # delay(1)
            # move_joints([1.57, 0.0, 1.57, 0.0, 1.57, 0.0], motors)
            # move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 0.0], motors)
            # count_box += 1
        # move_point(motors, armChain, box_position, "down")
        # delay(2)
        # move_point(motors, armChain, place_pos_init, "down")
        # delay(2)
        # new_point = place_pos_init
        # new_point[0] -= 0.25
        # move_point(motors, armChain, new_point, "down")
        # delay(2)
        # move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 0.0], motors)
        
        # z = 0.485
        # move_point(motors, armChain, [x, y, z], "down")
        # delay(0.5)
        # z = 0.55
        # move_point(motors, armChain, [x, y, z], "down")
        # y = 0.25
        # move_point(motors, armChain, [x, y, z], "down")
        # z = 0.2
        # move_point(motors, armChain, [x, y, z], "side")
        # y = 0.23
        # move_point(motors, armChain, [x, y, z], "side")
        # delay(0.5)
        # supervisor.setCustomData("ready")
        # y = 0.25
        # move_point(motors, armChain, [x, y, z], "side")
        # z = 0.55
        # move_point(motors, armChain, [x, y, z], "side")
        # move_joints([0.0, 0.0, 1.57, 0.0, 1.57, 0.0], motors)
        
            

            
        
            
            
            
    
    

    # move_point(motors, armChain, [x, y, z], "side")

    # while supervisor.step(timeStep) != -1:

        # t = supervisor.getTime()
 
        # x = 0.6
        # y = 0.0
        # z = 0.5
        # initial_position = [0] + [m.getPositionSensor().getValue() for m in motors]
        # ikResults = armChain.inverse_kinematics([x, y, z], [0.0, 0.0, 1.0], "Z")
        # print(ikResults)
        
        # for i in range(6):
            # motors[i].setPosition(ikResults[i + 1])



if __name__ == '__main__':
    main()

