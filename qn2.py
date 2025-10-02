#!/usr/bin/env python3
import variables as v
from ev3dev2.motor import SpeedDPS
from time import sleep
import math
import random
import utils as ut

# ------------------ Robot functions ------------------ #


def evalRobot2D(theta):
    x = v.l1*math.cos(theta[0]) + v.l2*math.cos(theta[0]+theta[1])
    y = v.l1*math.sin(theta[0]) + v.l2*math.sin(theta[0]+theta[1])
    return [x, y]


def fdJacob(theta, alpha=1e-6):
    J = [[0, 0], [0, 0]]
    f1 = evalRobot2D([theta[0]+alpha, theta[1]])
    f2 = evalRobot2D([theta[0]-alpha, theta[1]])
    f3 = evalRobot2D([theta[0], theta[1]+alpha])
    f4 = evalRobot2D([theta[0], theta[1]-alpha])

    J[0][0] = (f1[0]-f2[0])/(2*alpha)
    J[1][0] = (f1[1]-f2[1])/(2*alpha)
    J[0][1] = (f3[0]-f4[0])/(2*alpha)
    J[1][1] = (f3[1]-f4[1])/(2*alpha)
    return J

# ------------------ Inverse Kinematics ------------------ #


def invKin2D(l, theta0, pos, n, mode):
    tol = 1e-2
    alpha = 0.3  # damping factor for stability

    # Workspace check
    if ut.norm(pos) > sum(l):
        print("Target unreachable: outside workspace")
        return None

    if mode == 0:  # Newton
        for k in range(n):
            J = fdJacob(l, theta0)
            xk = evalRobot2D(theta0)
            fk = ut.vec_sub(pos, xk)

            if ut.norm(fk) < tol:
                print(
                    "Newton converged in %d steps, error" % (ut.norm(fk)))
                break

            invJ = ut.mat_inv(J)
            sk = ut.mat_vec_mul(invJ, fk)
            theta0 = ut.vec_add(theta0, ut.vec_scale(
                sk, alpha))  # damped update
        return theta0

    elif mode == 1:  # Algebraic
        if pos == [0, 0]:
            pos = [1e-6, 0]  # avoid singularity

        x, y = pos[0], pos[1]
        theta2 = math.acos(
            (x**2 + y**2 - l[0]**2 - l[1]**2) / (2 * l[0] * l[1]))
        k1 = l[0] + l[1] * math.cos(theta2)
        k2 = l[1] * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        return [theta1, theta2]


# ------------------ Main Code ------------------ #
pos = [x, y] = 0, -18.5
speed = 50
theta0 = [math.radians(10), math.radians(-10)]

v.link_1_motor.reset()
v.link_2_motor.reset()

# curr_x, curr_y = evalRobot2D(
#     [math.radians(v.link_1_motor.position), math.radians(v.link_2_motor.position)])
# print("Current Position: ", curr_x, curr_y)

# theta0 = [math.radians(v.link_1_motor.position),
#           math.radians(v.link_2_motor.position)]
mode = 1    # 0: Newton, 1: Broyden
n = 25      # max iterations

# Inverse Kinematics (Newton)
theta1, theta2 = invKin2D([v.l1, v.l2], theta0, pos, n, 0)
# Need to negate theta1 for motor direction
theta1, theta2 = -math.degrees(theta1), math.degrees(theta2)

print("For position %.2f %.2f, moving link1 %.2f and link2 %.2f" %
      (x, y, theta1, theta2))

x_check, y_check = evalRobot2D(
    [math.radians(-theta1), math.radians(theta2)])
print("Check: x=%.2f, y=%.2f" % (x_check, y_check))

# Move the motors
v.link_1_motor.on_to_position(SpeedDPS(speed), theta1)
v.link_2_motor.on_to_position(SpeedDPS(speed), theta2)

sleep(1)
v.link_1_motor.off()
v.link_2_motor.off()


# Inverse Kinematics (Algebraic)
# pos = [x, y] = 18.5, 0
# theta0 = [math.radians(-theta1), math.radians(theta2)]
# theta1_, theta2_ = invKin2D([v.l1, v.l2], theta0, pos, n, 0)
# theta1_ = -math.degrees(theta1_)
# theta2_ = math.degrees(theta2_)

# print("For position %.2f %.2f, moving link1 %.2f and link2 %.2f" %
#       (x, y, theta1_, theta2_))


# x_check_, y_check_ = evalRobot2D(
#     [math.radians(-theta1_), math.radians(theta2_)])
# print("Check: x=%.2f, y=%.2f" % (x_check_, y_check_))

# v.link_1_motor.on_to_position(SpeedDPS(speed), theta1_)
# v.link_2_motor.on_to_position(SpeedDPS(speed), theta2_)

# Move to midpoint
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
# midpoint = [(x1 + x2)/2, (y1 + y2)/2]
# print("Midpoint: ", midpoint)

# # Using Algebraic method to go to midpoint
# theta0 = [math.radians(v.link_1_motor.position),
#           math.radians(v.link_2_motor.position)]
# theta1_m, theta2_m = invKin2D([v.l1, v.l2], theta0, midpoint, n, 1)
# theta1_m = -math.degrees(theta1_m)
# theta2_m = math.degrees(theta2_m)
# print("Moving to midpoint, link1: %.2f, link2: %.2f" %
#       (theta1_m, theta2_m))
# v.link_1_motor.on_for_degrees(SpeedDPS(speed), theta1_m)
# v.link_2_motor.on_for_degrees(SpeedDPS(speed), theta2_m)

# Using Newton method to go to midpoint
# theta0 = [math.radians(v.link_1_motor.position),
#           math.radians(v.link_2_motor.position)]
# theta1_m, theta2_m = invKin2D([v.l1, v.l2], theta0, midpoint, n, 0)
# theta1_m = -math.degrees(theta1_m)
# theta2_m = math.degrees(theta2_m)
# print("Moving to midpoint, link1: %.2f, link2: %.2f" %
#       (theta1_m, theta2_m))
# v.link_1_motor.on_for_degrees(SpeedDPS(speed), theta1_m)
# v.link_2_motor.on_for_degrees(SpeedDPS(speed), theta2_m

# Final cleanup
# sleep(1)
# v.link_1_motor.off()
# v.link_2_motor.off()
