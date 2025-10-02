#!/usr/bin/env python3
import variables as v
from ev3dev2.motor import SpeedDPS
from time import sleep
from qn1 import get_end_effector_position
import math


def alg_inv_kine(x, y):
    if x == 0 or y == 0:
        return 0, 0

    theta2 = math.acos((x**2 + y**2 - v.l1**2 - v.l2**2) / (2 * v.l1 * v.l2))
    k1 = v.l1 + v.l2 * math.cos(theta2)
    k2 = v.l2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return theta1, theta2


def newton_inv_kine(x, y, max_iters=10):
    if x == 0 or y == 0:
        return 0, 0

    link1_pos = v.link_1_motor.position 
    link2_pos = v.link_2_motor.position
    theta1 = math.radians(-link1_pos)
    theta2 = math.radians(link2_pos)

    for _ in range(max_iters):
        # Current end effector position
        x_curr, y_curr = get_end_effector_position(
            math.degrees(theta1), math.degrees(theta2))

        # Compute the error
        error_x = x - x_curr
        error_y = y - y_curr
        
        if abs(x - x_curr) < 0.01 and abs(y - y_curr) < 0.01:
            break

        # Compute the Jacobian matrix
        J = [
            [-v.l1 * math.sin(theta1) - v.l2 * math.sin(theta1 + theta2),
             -v.l2 * math.sin(theta1 + theta2)],
            [v.l1 * math.cos(theta1) + v.l2 * math.cos(theta1 + theta2),
             v.l2 * math.cos(theta1 + theta2)]
        ]

        # Compute the determinant of the Jacobian
        det_J = J[0][0] * J[1][1] - J[0][1] * J[1][0]
        if det_J == 0:
            break  # Singular matrix, cannot proceed

        # Compute the inverse of the Jacobian
        J_inv = [
            [J[1][1] / det_J, -J[0][1] / det_J],
            [-J[1][0] / det_J, J[0][0] / det_J]
        ]

        # Update the angles
        theta1 += J_inv[0][0] * error_x + J_inv[0][1] * error_y
        theta2 += J_inv[1][0] * error_x + J_inv[1][1] * error_y
        
    return theta1, theta2

x, y = 2.5, 6.0
speed = 50
theta1, theta2 = alg_inv_kine(x, y)
theta1 = -math.degrees(theta1)
theta2 = math.degrees(theta2)

print("For position %.2f %.2f, moving link1 %.2f and link2 %.2f" %
      (x, y, theta1, theta2))

v.link_1_motor.on_for_degrees(SpeedDPS(speed), theta1)
v.link_2_motor.on_for_degrees(SpeedDPS(speed), theta2)
