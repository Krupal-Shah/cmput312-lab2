#!/usr/bin/env python3
import variables as v
from ev3dev2.motor import SpeedDPS
from time import sleep
import math


def get_end_effector_position(angle1, angle2):
    h_matrix = [
        [math.cos(math.radians(angle1 + angle2)), -math.sin(math.radians(angle1 + angle2)),
         v.l1 * math.cos(math.radians(angle1)) + v.l2 * math.cos(math.radians(angle1 + angle2))],
        [math.sin(math.radians(angle1 + angle2)), math.cos(math.radians(angle1 + angle2)), v.l1 *
         math.sin(math.radians(angle1)) + v.l2 * math.sin(math.radians(angle1 + angle2))],
        [0, 0, 1]
    ]
    return (h_matrix[0][2], h_matrix[1][2])


# Part b
angle1 = 10  # Define in degrees
angle2 = 20  # Define in degrees
speed = 50

v.link_1_motor.reset()
v.link_2_motor.reset()

v.link_1_motor.on_for_degrees(SpeedDPS(speed), -angle1)
v.link_2_motor.on_for_degrees(SpeedDPS(speed), angle2)

sleep(1)
v.link_1_motor.off()
v.link_2_motor.off()

end_effector_pos = get_end_effector_position(angle1, angle2)
print("End Effector Position: ", end_effector_pos)

###################################################
# # Part c_a: Measure a line
# v.link_1_motor.reset()
# v.link_2_motor.reset()
# start_position = (0, 0)
# end_position = (0, 0)
# while v.touch_sensor.is_pressed:
#     starting_angle1 = v.link_1_motor.position
#     starting_angle2 = v.link_2_motor.position
#     start_position = get_end_effector_position(
#         starting_angle1, starting_angle2)
#     print("Start Position: ", start_position)
#     sleep(0.1)

# while not v.touch_sensor.is_pressed:
#     ending_angle1 = v.link_1_motor.position
#     ending_angle2 = v.link_2_motor.position
#     end_position = get_end_effector_position(ending_angle1, ending_angle2)
#     print("End Position: ", end_position)
#     sleep(0.1)

# # Using the manhattan distance formula
# print("Distance of line drawn: ", math.sqrt(
#     (end_position[0] - start_position[0]) ** 2 + (end_position[1] - start_position[1]) ** 2))

###################################################
# # Part c_b: Measure an angle
# v.link_1_motor.reset()
# v.link_2_motor.reset()
# start_position = (0, 0)
# mid_position = (0, 0)
# end_position = (0, 0)
# while v.touch_sensor.is_pressed:
#     starting_angle1 = v.link_1_motor.position
#     starting_angle2 = v.link_2_motor.position
#     start_position = get_end_effector_position(
#         starting_angle1, starting_angle2)
#     print("Start Position: ", start_position)
#     sleep(0.1)

# while not v.touch_sensor.is_pressed:
#     mid_angle1 = v.link_1_motor.position
#     mid_angle2 = v.link_2_motor.position
#     mid_position = get_end_effector_position(mid_angle1, mid_angle2)
#     print("Mid Position: ", mid_position)
#     sleep(0.1)

# while v.touch_sensor.is_pressed:
#     ending_angle1 = v.link_1_motor.position
#     ending_angle2 = v.link_2_motor.position
#     end_position = get_end_effector_position(ending_angle1, ending_angle2)
#     print("End Position: ", end_position)
#     sleep(0.1)

# # https://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
# theta = math.atan2(end_position[1] - mid_position[1], end_position[0], mid_position[0]) - \
#     math.atan2(start_position[1] - mid_position[1],
#                start_position[0], mid_position[0])
# print("Angle between the two lines is: ", theta)
