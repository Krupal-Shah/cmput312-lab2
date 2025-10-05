#!/usr/bin/env python3
# import variables as v
# from ev3dev2.motor import SpeedDPS
# from time import sleep
import math

# v.link_1_motor.reset()
# v.link_2_motor.reset()
# while True:
#     angle1 = v.link_1_motor.position
#     angle2 = v.link_2_motor.position
#     print("Angles: ", -angle1, angle2)
#     sleep(0.5)

(x1, y1) = 8.1699, -7.1829
(x2, y2) = 8.6108, -11.6215
(x3, y3) = 5.1692, -12.65493

# theta = math.degrees(math.atan2(y3 - y2, x3 - x2) -
#                      math.atan2(y1 - y2, x1 - x2))

v1 = (x1 - x2, y1 - y2)
v2 = (x3 - x2, y3 - y2)

dot = v1[0]*v2[0] + v1[1]*v2[1]
det = v1[0]*v2[1] - v1[1]*v2[0]

theta = math.degrees(math.atan2(det, dot))

print("Angle between the two lines is: ", abs(theta))
