#!/usr/bin/env python3

"""
Group Members: Krupal Shah & Jaspreet Singh Chhabra

Date: October 8th 2025
 
Brick Number: G20

Lab Number: 2

Problem Number: 3 (Separate File for Newton's Method but called from lab2_problem3.py)
 
Brief Program/Problem Description:
    - Implement inverse kinematics for a 2-link robotic arm using both Newton's
      method.

Brief Solution Summary:
    - Different seed values considered for elbow up and elbow down positions.
    - Waypoints generated in a straight line to the target to converge to a solution. (5mm step)
    - Jacobian matrix calculated and inverted to find change in joint angles.
    - Clamping of joint angle changes to avoid large swings.

Used Resources/Collaborators:
    - Lecture slides and notes from CMPUT 312.
    - Textbook: "Introduction to Robotics: Mechanics and Control" by John J. Craig.
    - Textbook: "Modeling, Motion Planning, and Control of Manipulators and Mobile Robots"
        by Akshit Lunia et al.
    - OpenAI ChatGPT

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

import math
import utils as ut
import variables as v

L1, L2 = 115, 70        # link lengths in mm

# default speed
SPEED_DPS = 60
MAX_ITER = 50

# initial/home position
X_HOME = L1+L2
Y_HOME = 0

DEG = math.pi/180.0

v.link_1_motor.reset()
v.link_2_motor.reset()

def jacobian_2r(theta1, theta2):
    s1, c1 = math.sin(theta1), math.cos(theta1)
    s12, c12 = math.sin(theta1+theta2), math.cos(theta1+theta2)
    return [[-L1*s1 - L2*s12, -L2*s12],
            [L1*c1 + L2*c12,  L2*c12]]

def line_waypoints(start, goal, max_step=20.0):  # mm per step
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    dist = math.hypot(dx, dy)
    if dist <= max_step:
        return [goal]
    n = int(math.ceil(dist/max_step))
    return [(start[0] + (i/n)*dx, start[1] + (i/n)*dy) for i in range(1, n+1)]


def choose_newton_seed_for_target(x_goal, y_goal, last_seed=None):
    """
    Pure-Newton seeding: try two tiny elbow bends (up vs down) at home,
    pick the one with smaller initial task-space error to the GOAL.
    """
    # Two tiny-bend candidates around the singular home pose
    candidates = ([+5*DEG, -5*DEG],   # elbow-up-ish
                  [-5*DEG, +5*DEG])   # elbow-down-ish
    
    # Exclude last used seed if provided
    if last_seed is not None:
        candidates = [t for t in candidates if t != last_seed]

    best_t = None
    best_err = None
    for t in candidates:
        _, xy = ut.fk_2r((L1, L2), t[0], t[1])
        e = [x_goal - xy[0], y_goal - xy[1]]

        en = ut.norm(e)
        # do not pick the last chosen candidate
        if (best_err is None) or (en < best_err):
            best_err = en
            best_t = [t[0], t[1]]
    return best_t

def clamp_step(dtheta, max_step_deg=10.0):
    """Limit per-iteration joint change to avoid huge swings."""
    max_rad = max_step_deg * DEG
    m = max(abs(dtheta[0]), abs(dtheta[1]))
    if m > max_rad:
        s = max_rad / m
        dtheta[0] *= s
        dtheta[1] *= s
    return dtheta

def move_to_xy(x_target, y_target):
    print("Inverse Kinematics - move arm to (x,y) target (mm)", x_target, y_target)
    
    theta = choose_newton_seed_for_target(x_target, y_target)
    last_seed = theta
    count = 0
    
    xy_start = (X_HOME, Y_HOME)

    print("Starting point", xy_start)

    # 5 mm waypoints
    waypoints = line_waypoints(xy_start, (x_target, y_target), max_step=5.0)

    # Newton per waypoint (1 mm tol) + step clamp
    for wp in waypoints:
        for _ in range(MAX_ITER):
            _, xy = ut.fk_2r((L1, L2), theta[0], theta[1])          # end-effector in mm
            e = [wp[0] - xy[0], wp[1] - xy[1]]        # mm
            if ut.norm(e) < 1.0:                     # 1 mm tolerance
                break

            J = jacobian_2r(theta[0], theta[1])
            J_inv = ut.mat_inv(J)
            if J_inv is None:
                print("Singular Jacobian at waypoint", wp)
                break

            dtheta = ut.mat_mul_vec(J_inv, e)           # rad
            dtheta = clamp_step(dtheta, max_step_deg=10.0)

            theta[0] += dtheta[0]
            theta[1] += dtheta[1]
            # theta = clamp_joint_limits(theta[0], theta[1])

            print("Waypoint:", wp, " Error (mm):", e, " dtheta (rad):", dtheta)

        ut.move_to_angles(SPEED_DPS, ut.to_degrees(theta))
        clamped, _ = ut.clamp_to_workspace(ut.to_degrees(theta))
        if clamped:
            print("\nWarning: joint limits reached at waypoint", wp)

            theta = choose_newton_seed_for_target(x_target, y_target, last_seed)
            last_seed = theta
            print("Trying again with alternative seed:", [t/DEG for t in theta])

            theta = [t * 2 for t in theta]
            ut.move_to_angles(SPEED_DPS, ut.to_degrees(theta))
            
            # generate new waypoints from current position to target
            _, xy = ut.fk_2r((L1, L2), theta[0], theta[1])          # end-effector in mm
            waypoints = line_waypoints(xy, (x_target, y_target), max_step=5.0)
            count += 1
            if count > 2:
                print("Failed to reach target after 2 reseed attempts.")
                break
            

def main():
    print("")
    # test case:

    # move_to_xy(X_HOME, Y_HOME)
    # move_to_xy(87, -185)
    # move_to_xy(115,  80)
    # move_to_xy(115, -80)
    # move_to_xy(135, -40)
    # move_to_xy(135, 40)

    # move_to_xy(140, -60)
    # move_to_xy(180,  10)
    # move_to_xy( 60, -30)
    # move_to_xy(40, -150)
    # move_to_xy(40, 150)
    # move_to_xy(60, 60)
    # move_to_xy(0,0)

if __name__ == "__main__":
    main()
