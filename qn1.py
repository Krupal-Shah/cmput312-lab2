#!/usr/bin/env python3
import variables as v
import utils as ut
from ev3dev2.motor import SpeedDPS
from time import sleep
import math

def evalRobot2D(theta):
    x = v.l1*math.cos(theta[0]) + v.l2*math.cos(theta[0]+theta[1])
    y = v.l1*math.sin(theta[0]) + v.l2*math.sin(theta[0]+theta[1])
    return [x, y]


# Part b
angle1 = 45    # degrees
angle2 = 90     # degrees
speed = 50      # degrees per second

# Reset motor positions
v.link_1_motor.reset()
v.link_2_motor.reset()

ut.move_to_angles(speed, [-angle1, angle2])

v.link_1_motor.off()
v.link_2_motor.off()

theta = [math.radians(angle1), math.radians(angle2)]
end_effector_pos = evalRobot2D(theta)

print("End Effector Position: ", end_effector_pos)

###################################################
# # Part c_a: Measure a line
# v.link_1_motor.reset()
# v.link_2_motor.reset()
# positions = []
# print("Press the touch sensor to record positions")
# while True:
#     if v.touch_sensor.is_pressed:
#         angle1 = -v.link_1_motor.position
#         angle2 = v.link_2_motor.position
#         print("Angles Recorded: ", angle1, angle2)
#         position = evalRobot2D(
#             [math.radians(angle1), math.radians(angle2)])
#         print("Position Recorded: ", position)
#         positions.append(position)

#     if len(positions) == 2:
#         break

#     sleep(0.2)

# x1, y1 = positions[0]
# x2, y2 = positions[1]

# # Using the manhattan distance formula
# print("Distance of line drawn: ", math.sqrt((x2 - x1)**2 + (y2 - y1)**2))

###################################################
# Part c_b: Measure an angle
# v.link_1_motor.reset()
# v.link_2_motor.reset()
# positions = []
# print("Press the touch sensor to record positions")
# while True:
#     if v.touch_sensor.is_pressed:
#         angle1 = -v.link_1_motor.position
#         angle2 = v.link_2_motor.position
#         position = evalRobot2D(
#             [math.radians(angle1), math.radians(angle2)])
#         print("Position Recorded: ", position)
#         positions.append(position)

#     if len(positions) == 3:
#         break

#     sleep(0.2)

# x1, y1 = positions[0]
# x2, y2 = positions[1]
# x3, y3 = positions[2]

# # https://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
# theta = math.degrees(math.atan2(y3 - y2, x3 - x2) -
#                      math.atan2(y1 - y2, x1 - x2))
# print("Angle between the two lines is: ", abs(theta))
