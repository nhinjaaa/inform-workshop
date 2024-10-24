#!/usr/bin/env python3

# 1 _______ IMPORTS _______
import math
import time
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *
from pybricks.tools import wait
from pybricks.hubs import EV3Brick

# 2 _______ VARIABLEN _______
# Variablen: Farben
blau = Color.BLUE
gelb = Color.YELLOW
gruen = Color.GREEN
rot = Color.RED
pink = Color.RED
schwarz = Color.BLACK
weiss = Color.WHITE

# Variablen: Roboter-Komponenten
ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B)
left_motor = motorA
right_motor = motorB
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
color_sensor_in1 = ColorSensor(Port.S1)
obstacle_sensor_front = UltrasonicSensor(Port.S2)
obstacle_sensor_left = UltrasonicSensor(Port.S3)
obstacle_sensor_right = UltrasonicSensor(Port.S4)
gyro_sensor= GyroSensor(Port.S5)
motorC = Motor(Port.C)

# Variablen: Geschwindigkeit
v = 150

# 3 _______ METHODEN _______
# Geradeausfahren
def geradeaus(strecke):
    robot.reset() 
    
    PROPORTIONAL_GAIN = 2.5
    while robot.distance() < strecke:
        angle_correction = -1 * PROPORTIONAL_GAIN * gyro_sensor.angle()
        robot.drive(v, angle_correction) 
        wait(10)
    robot.stop()

# Rueckwaertsfahren
def zurueck(strecke):
    robot.reset() 
    
    PROPORTIONAL_GAIN = 2.5
    while robot.distance() > -strecke:
        angle_correction = -1 * PROPORTIONAL_GAIN * gyro_sensor.angle()
        robot.drive(-v, angle_correction) 
        wait(10)
    robot.stop()
    
# Rechtsabbiegen
def rechts(drehwinkel):
    while gyro_sensor.angle() < drehwinkel:
        motorA.dc(10)
        motorB.dc(-10)
    while gyro_sensor.angle() != 0:
        gyro_sensor.reset_angle(0)
    robot.stop()

# Linksabbiegen
def links(drehwinkel):
    while gyro_sensor.angle() > -drehwinkel:
        motorA.dc(-10)
        motorB.dc(10)
    while gyro_sensor.angle() != 0:
        gyro_sensor.reset_angle(0)
    robot.stop()
    
# Farbsensor
def farbsensor():
    return color_sensor_in1.color()

# Abstandssensor
def abstandssensor_vorne():
    return obstacle_sensor_front.distance()

def abstandssensor_links():
    return obstacle_sensor_left.distance()

def abstandssensor_rechts():
    return obstacle_sensor_right.distance()

################################

robot.settings(straight_speed=v, straight_acceleration=50, turn_rate=v)

# 4 _______ MAIN-METHODE _______
def labyrinth_2():
    ... # Hier kommt euer Code hin

def main():
    labyrinth_2()

if __name__ == '__main__':
    main()