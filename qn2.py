#!/usr/bin/env python3
import variables as v
from ev3dev2.motor import SpeedDPS
from time import sleep
import math
import utils as ut
from typing import Union
from inv_kine_newtons import move_to_xy

SPEED = 50  # degrees per second

### Helper Functions ###


# max step size
def line_waypoints(start: Union[list, tuple], goal: Union[list, tuple], max_step=20.0):
    """Generate waypoints from start to goal with max step size."""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    dist = math.hypot(dx, dy)
    if dist <= max_step:
        return [goal]
    n = int(math.ceil(dist/max_step))
    return [(start[0] + (i/n)*dx, start[1] + (i/n)*dy) for i in range(1, n+1)]


def choose_starting_point(x_target, y_target):
    """Choose a good starting point for inverse kinematics."""
    candidates = ([+5, -5],   # elbow-up-ish
                  [-5, +5])   # elbow-down-ish
    best_t = None
    best_err = None
    for t in candidates:
        xy = ut.fk_2r(t[0], t[1])
        e = [x_target - xy[0], y_target - xy[1]]

        en = ut.norm(e)
        if (best_err is None) or (en < best_err):
            best_err = en
            best_t = [t[0], t[1]]
    return best_t


def to_mm(pos):
    return (pos[0] * 10, pos[1] * 10)
# ------------------ Inverse Kinematics ------------------ #


def invKin2D(pos, n, mode, start_x=None, start_y=None):
    tol = 0.1       # tolerance in cm
    alpha = 0.3     # damping factor for stability

    # Workspace check
    # if ut.norm(pos) > (v.l1 + v.l2) or ut.norm(pos) < abs(v.l1 - v.l2):
    #     print("Position out of workspace")
    #     print("Position (cm):", ut.norm(pos),
    #           " Workspace radius (cm):", v.l1 + v.l2)
    #     return None

    if mode == 0:  # Newton Method
        # # Keep everything in cm for consistency with fk_2r and jacobian
        # theta0 = choose_starting_point(pos[0], pos[1])
        # theta0 = ut.to_radians(theta0)
        # xy_start = [v.l1+v.l2, 0]   # Initial position (cm)

        # if start_x and start_y:
        #     xy_start = [start_x, start_y]

        # waypoints = line_waypoints(
        #     xy_start, pos, max_step=0.5)  # 2 cm per step

        # for wp in waypoints:
        #     for iteration in range(n):
        #         # Jacobian expects a list [theta1, theta2]
        #         J = ut.jacobian_2r(theta0)
        #         xy = ut.fk_2r(theta0[0], theta0[1])  # Forward kinematics
        #         e = ut.vec_sub(wp, xy)     # error in cm

        #         if ut.norm(e) < tol/10.0:  # tolerance in cm (0.1 cm)
        #             print("Newton converged in %d steps, error %.3f cm" %
        #                   (iteration, ut.norm(e)))
        #             break

        #         invJ = ut.mat_inv(J)

        #         if invJ is None:
        #             print("Singular Jacobian at waypoint", wp)
        #             break

        #         dtheta = ut.mat_mul_vec(invJ, e)
        #         theta0 = ut.vec_add(theta0, ut.vec_scale(
        #             dtheta, alpha))  # damped update

        #     print("Waypoint: [%.2f, %.2f] cm, Error: %.3f cm, Angles: [%.1f, %.1f] deg" %
        #           (wp[0], wp[1], ut.norm(e), math.degrees(theta0[0]), math.degrees(theta0[1])))
        #     ut.move_to_angles(SPEED, ut.to_degrees(theta0))

        # return ut.to_degrees(theta0)
        pos = to_mm(pos)
        move_to_xy(pos[0], pos[1], start_x, start_y)

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

# End point
# 8.7, 18.5
# 3.5, -18.5
# 18.5, 0

### Main Program ###


def main():
    mode = 0        # 0: Moving to positon, 1: Midpoint Calculation
    inv_mode = 0    # 0: Newton, 1: Analytical
    n = 25          # max iterations
    pos = [x, y] = 8.7, 18.5     # target position in cm

    v.link_1_motor.reset()
    v.link_2_motor.reset()
    sleep(1)  # wait for reset to complete

    if mode == 0:
        theta = invKin2D(pos, n, inv_mode)

        print("For position %.2f %.2f, moving link1 %.2f and link2 %.2f" %
              (x, y, theta[0], theta[1]))

        x_check, y_check = ut.fk_2r(
            math.radians(theta[0]), math.radians(theta[1]))
        print("Check: x=%.2f, y=%.2f" % (x_check, y_check))

    elif mode == 1:
        print("Press the touch sensor to record positions")
        (x1, y1) = ut.sample_point()
        (x2, y2) = ut.sample_point()
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        print("Midpoint is: (%.2f, %.2f)" % (mid_x, mid_y))
        theta = invKin2D([mid_x, mid_y], n, inv_mode, start_x=x2, start_y=y2)
        print("Moving to midpoint, angles (degrees):", theta)

    else:
        print("Invalid mode selected")
        exit(1)

    v.link_1_motor.off()
    v.link_2_motor.off()


if __name__ == "__main__":
    main()
