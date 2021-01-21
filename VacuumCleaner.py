from HAL import HAL
from GUI import GUI
import random
import time
import math

state = 1
v = 0
w = 2

while True:
    #Generate the spiral for start
    if(state == 1 and HAL.bumper.getBumperData().state == 0):
        HAL.motors.sendV(v)
        HAL.motors.sendW(w)
        v += 0.0125

    #Point towards a random direction if collision takes place
    elif(HAL.bumper.getBumperData().state == 1):
        state = 2		
        v = 0
        w = 3
        HAL.motors.sendV(v)
        if(HAL.bumper.getBumperData().bumper == 0):
        	HAL.motors.sendW(w)
        else:
        	HAL.motors.sendW(-1 * w)
        time.sleep(random.random() * math.pi)
        HAL.motors.sendV(0)
        HAL.motors.sendW(0)
        time.sleep(0.8)
        state = 3
        
    #Straight Move
    elif(state == 3):		
        v = 5
        w = 0
        HAL.motors.sendV(v)
        HAL.motors.sendW(w)