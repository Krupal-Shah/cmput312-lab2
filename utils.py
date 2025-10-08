"""
Group Members: Krupal Shah & Jaspreet Singh Chhabra

Date: October 8th 2025
 
Brick Number: G20

Lab Number: 2

Problem Number: - (Helper file to store common functions)
 
Brief Program/Problem Description: -
Brief Solution Summary: -

Used Resources/Collaborators: Lecture slides, EV3Dev2 documentation,
     OpenAI ChatGPT

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

import math
import variables as v
from time import sleep
from ev3dev2.motor import SpeedDPS


### Math Matrix Utility Functions ###
def norm(v):
    return math.sqrt(v[0]**2 + v[1]**2)


def mat_mul_vec(M, v):
    return [M[0][0]*v[0] + M[0][1]*v[1],
            M[1][0]*v[0] + M[1][1]*v[1]]


def det(M):
    return M[0][0]*M[1][1] - M[0][1]*M[1][0]


def mat_inv(M, damping=1e-6):
    det_ = det(M)
    if abs(det_) < 1e-9:
        det_ += damping  # regularize
    return [[M[1][1]/det_, -M[0][1]/det_],
            [-M[1][0]/det_,  M[0][0]/det_]]


def to_degrees(angle):
    return [math.degrees(angle[0]), math.degrees(angle[1])]


def to_radians(angle):
    return [math.radians(angle[0]), math.radians(angle[1])]


### Robot Kinematics Functions ###
def fk_2r(l, theta1, theta2):
    """Forward kinematics for 2R planar robot."""
    x = l[0] * math.cos(theta1) + l[1] * math.cos(theta1 + theta2)
    y = l[0] * math.sin(theta1) + l[1] * math.sin(theta1 + theta2)
    return x, y


def get_current_joint_angles():
    l1_deg = v.link_1_motor.position
    l2_deg = v.link_2_motor.position
    return [l1_deg, l2_deg]


def move_to_angles(speed, angle):
    _, angle = clamp_to_workspace(angle)
    print("Moving to angles (deg): ", angle)

    v.link_1_motor.on_to_position(SpeedDPS(speed), -angle[0])
    v.link_2_motor.on_to_position(SpeedDPS(speed), angle[1])


def clamp_to_workspace(angle):
    clamped = False
    angle1 = angle[0]
    angle2 = angle[1]

    if angle1 > 65:
        angle1 = 65
        clamped = True
    elif angle1 < -80:
        angle1 = -80
        clamped = True

    if angle2 < -140:
        angle2 = -140
        clamped = True
    elif angle2 > 160:
        angle2 = 160
        clamped = True

    return clamped, [angle1, angle2]


def wait_for_press_and_release(ts):
    while not ts.is_pressed:
        sleep(0.02)
    sleep(0.05)
    while ts.is_pressed:
        sleep(0.02)
    sleep(0.05)


def sample_point():
    """Wait for press, read encoders, compute and return end-effector position."""
    wait_for_press_and_release(v.touch_sensor)
    theta1_deg = v.link_1_motor.position
    theta2_deg = v.link_2_motor.position

    theta1 = math.radians(-theta1_deg)  # negated motor 1 as it is mirrored
    theta2 = math.radians(theta2_deg)

    x, y = fk_2r(theta1, theta2)

    print("\n--- Forward Kinematics ---")
    print("m1 (theta1):", theta1_deg, "deg")
    print("m2 (theta2):", theta2_deg, "deg")
    print("End Effector: x =", x, "cm, y =", y, "cm")

    return (x, y)
