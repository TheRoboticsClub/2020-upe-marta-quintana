from HAL import HAL
from GUI import GUI

import numpy as np
import time
import math

# Car direction
carx = 0.0
cary = 0.0

# Obstacles direction
obsx = 0.0
obsy = 0.0

# Average direction
avgx = 0.0
avgy = 0.0

def check_target(self, posx, posy, targetx, targety):
    if(abs(targetx - posx) <= 6 and abs(targety - posy) <= 6):
        currentTarget.setReached(True)
        currentTarget = GUI.map.getNextTarget()

        if(currentTarget != None):
            return 1
        else:
            return 0

while True:
    currentTarget = GUI.map.getNextTarget()
    GUI.map.targetx = currentTarget.getPose().x
    GUI.map.targety = currentTarget.getPose().y
    posx = HAL.getPose3d().x
    posy = HAL.getPose3d().y
    targetid = currentTarget.getId()

    # TODO
    k_obstacle = 0.6
    k_car = 0.2
    k_target = -0.2
    unit_angle = (HAL.getLaserData().maxAngle - HAL.getLaserData().minAngle)/len(HAL.getLaserData().values)
    unit_range = 1/(HAL.getLaserData().maxRange - HAL.getLaserData().minRange)
    

    target_vector = [k_target * (GUI.map.targetx - posx), k_target * (GUI.map.targety - posy)]
    rot_angle = (-1) * (HAL.getPose3d().yaw + math.pi/2)
    target_vector = [target_vector[0] * math.cos(rot_angle) - target_vector[1] * math.sin(rot_angle), target_vector[0] * math.sin(rot_angle) + target_vector[1] * math.cos(rot_angle)]

    obstacle_vector = [0, 0]
    angle = 180


    for r in HAL.getLaserData().values:
        if r != 0:
            obstacle_vector[0] += k_obstacle * (10/r) *  math.cos(math.radians(angle))
        #Y component of Obstacle Vector remains unchanged

        #print(str(k_obstacle * (1/r) * (1/r) *  math.cos(math.radians(angle))) + " " + str(k_obstacle * (1/r) * (1/r) *   math.sin(math.radians(angle))) + " " + str(r))			
        angle -= 1

    if(abs(obstacle_vector[0]) > 12):
        obstacle_vector[0] = 12 *(abs(obstacle_vector[0])/obstacle_vector[0])
    
    car_vector = [0, 0]

    running = 0

    if(abs(GUI.map.targetx - posx) <= 6 and abs(GUI.map.targety - posy) <= 6):
        currentTarget.setReached(True)
        currentTarget = GUI.map.getNextTarget()

    if(currentTarget != None):
        running = 1

    if(running == 0):
        HAL.motors.sendV(0)
        HAL.motors.sendW(0)
        

    car_vector[0] = k_car * (target_vector[0] + obstacle_vector[0])
    car_vector[1] = k_car * (target_vector[1] + obstacle_vector[1])
    
    HAL.motors.sendV(3.5)
    #Add PID to angle actual - where it should be headed times a constant
    k_angle = -0.5
    HAL.motors.sendW(k_angle * (car_vector[0]))

    # Car direction <GREEN>
    GUI.map.carx = car_vector[0]
    GUI.map.cary = car_vector[1]

    # Obstacles direction <RED>
    GUI.map.obsx = obstacle_vector[0]
    GUI.map.obsy = obstacle_vector[1]

    # Target Vector <BLACK>
    GUI.map.avgx = target_vector[0]
    GUI.map.avgy = target_vector[1]
