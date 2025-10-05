#!/usr/bin/env python3
import variables as v
from ev3dev2.motor import SpeedDPS
from time import sleep
import math
import utils as ut

SPEED = 50  # degrees per second

### Helper Functions ###
def line_waypoints(start, goal, max_step=20.0):  # mm per step
    """Generate waypoints from start to goal with max step size."""
    dx, dy = goal - start[0], goal - start[1]
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
        _, xy = ut.fk_2r(t[0], t[1])
        e = [x_target - xy[0], y_target - xy[1]]

        en = ut.norm(e)
        if (best_err is None) or (en < best_err):
            best_err = en
            best_t = [t[0], t[1]]
    return best_t


def to_mm(pos):
    return [pos[0]*10, pos[1]*10]

# ------------------ Inverse Kinematics ------------------ #
def invKin2D(pos, n, mode, start_x=None, start_y=None):
    tol = 1.0       # tolerance in mm
    alpha = 0.3     # damping factor for stability

    # Workspace check
    if ut.norm(pos) > (v.l1 + v.l2) or ut.norm(pos) < abs(v.l1 - v.l2):
        print("Position out of workspace")
        return None

    if mode == 0:  # Newton Method
        pos = to_mm(pos)  # convert to mm
        theta0 = choose_starting_point(pos[0], pos[1])
        theta0 = ut.to_radians(theta0)
        xy_start = to_mm(v.l1+v.l2, 0)   # Initial position (mm)

        if start_x and start_y:
            xy_start = to_mm(start_x, start_y)

        waypoints = line_waypoints(xy_start, pos, max_step=20.0)

        for wp in waypoints:
            for _ in range(n):
                J = ut.jacobian_2r(theta0)
                xy = ut.fk_2r(theta0[0], theta0[1])
                e = ut.vec_sub(wp, xy)     # error in mm

                if ut.norm(e) < tol:
                    print(
                        "Newton converged in %d steps, error" % (ut.norm(e)))
                    break

                invJ = ut.mat_inv(J)
                if invJ is None:
                    print("Singular Jacobian at waypoint", wp)
                    break

                dtheta = ut.mat_vec_mul(invJ, e)
                theta0 = ut.vec_add(theta0, ut.vec_scale(
                    dtheta, alpha))  # damped update

            print("Waypoint:", wp, " Error (mm):", e,
                  " theta0:", ut.to_degrees(theta0))
            ut.move_to_angles(SPEED, ut.to_degrees(theta0))

        return ut.to_degrees(theta0)

    elif mode == 1:  # Analytical Method
        if pos == [0, 0]:
            pos = [1e-6, 0]  # avoid singularity

        x, y = pos[0], pos[1]
        cos_theta2 = (x**2 + y**2 - v.l1**2 - v.l2**2) / (2 * v.l1 * v.l2)

        # There are two possible solutions for theta2 (elbow up/down)
        theta2_elbow_up = math.acos(cos_theta2)
        theta2_elbow_down = -theta2_elbow_up

        # Corresponding theta1 values
        # Elbow up configuration
        k1_up = v.l1 + v.l2 * math.cos(theta2_elbow_up)
        k2_up = v.l2 * math.sin(theta2_elbow_up)
        theta1_elbow_up = ut.to_degrees(
            math.atan2(y, x) - math.atan2(k2_up, k1_up))

        # Elbow down configuration
        k1_down = v.l1 + v.l2 * math.cos(theta2_elbow_down)
        k2_down = v.l2 * math.sin(theta2_elbow_down)
        theta1_elbow_down = ut.to_degrees(
            math.atan2(y, x) - math.atan2(k2_down, k1_down))

        # Join the angles into pairs
        theta_elbow_up = [theta1_elbow_up, ut.to_degrees(theta2_elbow_up)]
        theta_elbow_down = [theta1_elbow_down,
                            ut.to_degrees(theta2_elbow_down)]

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
    else:
        print("Invalid mode for inverse kinematics")
        return None


def main():
    mode = 0        # 0: Moving to positon, 1: Midpoint Calculation
    inv_mode = 0    # 0: Newton, 1: Analytical
    n = 25          # max iterations

    v.link_1_motor.reset()
    v.link_2_motor.reset()
    sleep(1)  # wait for reset to complete

    if mode == 0:
        pos = [x, y] = 0, -18.5     # target position in cm
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
