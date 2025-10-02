#!/usr/bin/env python3
import variables as v
from ev3dev2.motor import SpeedDPS
from time import sleep
import math

v.link_1_motor.reset()
v.link_2_motor.reset()
while True:
    angle1 = v.link_1_motor.position
    angle2 = v.link_2_motor.position
    print("Angles: ", angle1, angle2)
    sleep(0.5)