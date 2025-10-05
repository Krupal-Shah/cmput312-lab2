#!/usr/bin/env python3
import math
from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sensor import INPUT_1

import inv_kine_analytical
import inv_kine_newtons

MODE = 2
# 1 for analytical and 2 for newton's 

L1_MM = 115.0   # upper arm length
L2_MM = 70.0    # forearm length

M1_PORT = OUTPUT_A   
M2_PORT = OUTPUT_B   

# Touch sensor port
TOUCH_PORT = INPUT_1

# ----------------------------
# Setup
# ----------------------------
m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)
touch = TouchSensor(TOUCH_PORT)

# Reset encoders at "home" (0 deg)
m1.reset()
m2.reset()

print("Encoders zeroed at home.")
print("Move the robot to any pose using your program/controls.")
print("Press and release the touch sensor to sample encoder angles and compute (x, y).")


def fk_2r(theta1, theta2, l1_mm, l2_mm):
    x = l1_mm * math.cos(theta1) + l2_mm * math.cos(theta1 + theta2)
    y = l1_mm * math.sin(theta1) + l2_mm * math.sin(theta1 + theta2)
    return x, y

def wait_for_press_and_release(ts):
    while not ts.is_pressed:
        sleep(0.02)
    sleep(0.05)
    while ts.is_pressed:
        sleep(0.02)
    sleep(0.05)


def sample_point():
    """Wait for press, read encoders, compute and return end-effector position."""
    wait_for_press_and_release(touch)
    theta1_deg = m1.position
    theta2_deg = m2.position

    # Apply sign convention (negated motor 1 if mirrored)
    theta1 = math.radians(-theta1_deg)
    theta2 = math.radians(theta2_deg)

    x_mm, y_mm = fk_2r(theta1, theta2, L1_MM, L2_MM)
    print("\n--- fwd kinematics ---")
    print("m1 (theta1):", theta1_deg, "deg")
    print("m2 (theta2):", theta2_deg, "deg")
    print("End Effector: x =", x_mm, "mm, y =", y_mm, "mm")

    return (x_mm, y_mm)


def main():
    print("Move robot to first pose, press and release touch sensor...")
    p1 = sample_point()

    print("Move robot to second pose, press and release touch sensor...")
    p2 = sample_point()

    mid_x = (p1[0] + p2[0]) / 2.0
    mid_y = (p1[1] + p2[1]) / 2.0

    print("\n--- Results ---")
    print("Point 1:", p1)
    print("Point 2:", p2)
    print("Midpoint:", mid_x,"mm", mid_y, "mm")

    if MODE == 1:
        # analytical method
        # todo add the final pos to analytical
        inv_kine_analytical.move_to_xy(mid_x,mid_y)
        
    if MODE == 2:
        # newton's method
        inv_kine_newtons.move_to_xy(mid_x,mid_y, p2[0], p2[1])

if __name__ == "__main__":
    main()
