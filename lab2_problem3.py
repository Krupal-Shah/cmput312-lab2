#!/usr/bin/env python3

"""
Group Members: Krupal Shah & Jaspreet Singh Chhabra

Date: October 8th 2025
 
Brick Number: G20

Lab Number: 2

Problem Number: 3
 
Brief Program/Problem Description:
    - Implement inverse kinematics for a 2-link robotic arm using both Newton's
      method and an analytical approach.
    - Write a program that receives as input a (x,y) location inside the robot 
      working space and moves the robot end effector to the input location.
    - Write a program that finds the midpoint between two points sampled by the
      robot and moves the robot end effector to that midpoint.

Brief Solution Summary:
    - Implemented inverse kinematics using lecture slides.
    - Reused functions from problem 2 for selecting points and moving the robot.

Used Resources/Collaborators:
    - Lecture slides and notes from CMPUT 312.
    - Textbook: "Introduction to Robotics: Mechanics and Control" by John J. Craig.
    - OpenAI ChatGPT

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

import math
import utils as ut
import variables as v

from time import sleep
from inv_kine_newtons import move_to_xy

SPEED = 50  # degrees per second


def to_mm(pos):
    return (pos[0] * 10, pos[1] * 10)


def invKin2D(pos, mode):
    if mode == 0:  # Newton Method
        pos = to_mm(pos)
        print(pos)
        move_to_xy(pos[0], pos[1])

    elif mode == 1:  # Analytical Method
        try:
            if pos == [0, 0]:
                pos = [1e-6, 0]  # avoid singularity

            x, y = pos[0], pos[1]
            cos_theta2 = max(-1.0, min(1.0, (x**2 + y**2 -
                             v.l1**2 - v.l2**2) / (2 * v.l1 * v.l2)))

            # There are two possible solutions for theta2 (elbow up/down)
            theta2_elbow_up = math.acos(cos_theta2)
            theta2_elbow_down = -theta2_elbow_up

            # Corresponding theta1 values
            # Elbow up configuration
            k1_up = v.l1 + v.l2 * math.cos(theta2_elbow_up)
            k2_up = v.l2 * math.sin(theta2_elbow_up)
            theta1_elbow_up = math.atan2(y, x) - math.atan2(k2_up, k1_up)

            # Elbow down configuration
            k1_down = v.l1 + v.l2 * math.cos(theta2_elbow_down)
            k2_down = v.l2 * math.sin(theta2_elbow_down)
            theta1_elbow_down = math.atan2(y, x) - math.atan2(k2_down, k1_down)

            # Join the angles into pairs
            theta_elbow_up = ut.to_degrees([theta1_elbow_up, theta2_elbow_up])
            theta_elbow_down = ut.to_degrees(
                [theta1_elbow_down, theta2_elbow_down])

            clamped1, _ = ut.clamp_to_workspace(theta_elbow_up)
            clamped2, _ = ut.clamp_to_workspace(theta_elbow_down)

            if not clamped1:
                print("Elbow Up Solution chosen")
                print("Angles (degrees):", theta_elbow_up)
                ut.move_to_angles(SPEED, theta_elbow_up)
                return theta_elbow_up
            elif not clamped2:
                print("Elbow Down Solution chosen")
                print("Angles (degrees):", theta_elbow_down)
                ut.move_to_angles(SPEED, theta_elbow_down)
                return theta_elbow_down
            else:
                print("Both solutions out of workspace")
                print("Elbow Up Angles (degrees):", theta_elbow_up)
                print("Elbow Down Angles (degrees):", theta_elbow_down)
                return None
        except ValueError as e:
            print("Math domain error: %.2f" % e)
            print("No solution exists for position (%.2f, %.2f)" % (x, y))
            return None
    else:
        print("Invalid mode for inverse kinematics")
        return None


def main():
    mode = 1                        # 0: Moving to positon, 1: Midpoint Calculation
    inv_mode = 1                    # 0: Newton, 1: Analytical
    pos = [x, y] = 5.41, -10.77     # target position in cm

    v.link_1_motor.reset()
    v.link_2_motor.reset()
    sleep(1)  # wait for reset to complete

    if mode == 0:
        theta = invKin2D(pos, inv_mode)

        if inv_mode == 1:
            print("For position %.2f %.2f, moving link1 %.2f and link2 %.2f" %
                  (x, y, theta[0], theta[1]))

            x_check, y_check = ut.fk_2r(
                math.radians(theta[0]), math.radians(theta[1]))
            print("Check: x=%.2f, y=%.2f" % (x_check, y_check))
        else:
            pos = ut.get_current_joint_angles()
            x_check, y_check = ut.fk_2r(
                math.radians(pos[0]), math.radians(pos[1]))
            print("Check: x=%.2f, y=%.2f" % (x_check, y_check))

    elif mode == 1:
        print("Press the touch sensor to record positions")
        (x1, y1) = ut.sample_point()
        (x2, y2) = ut.sample_point()
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        print("Midpoint is: (%.2f, %.2f)" % (mid_x, mid_y))
        theta = invKin2D([mid_x, mid_y], inv_mode)
        print("Moving to midpoint, angles (degrees):", theta)

    else:
        print("Invalid mode selected")
        exit(1)

    v.link_1_motor.off()
    v.link_2_motor.off()


if __name__ == "__main__":
    main()
