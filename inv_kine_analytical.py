#!/usr/bin/env python3
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS



L1, L2 = 115.0, 70.0                  # link lengths (mm)
X_HOME, Y_HOME = L1 + L2, 0.0         # home in mm -> (185, 0)

SPEED_DPS = 60                        # motor speed (deg/s)
MAX_ITER  = 50                        # Newton iterations per waypoint

DEG = math.pi / 180.0

m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)

m1.reset()
m2.reset()
print("Encoders after reset (deg):", m1.position, m2.position)

# ===========
# Kinematics
# ===========
def fk_2r(theta1, theta2):
    """
    Forward kinematics: returns end-effector [x, y] in mm.
    (Elbow position not needed for control; simpler API avoids tuple/list mix-ups.)
    """
    x = L1*math.cos(theta1) + L2*math.cos(theta1 + theta2)
    y = L1*math.sin(theta1) + L2*math.sin(theta1 + theta2)
    return [x, y]

def jacobian_2r(theta1, theta2):
    """
    Geometric Jacobian (2x2) mapping joint rates (rad/s) to tip velocity (mm/s).
    """
    s1, c1   = math.sin(theta1), math.cos(theta1)
    s12, c12 = math.sin(theta1 + theta2), math.cos(theta1 + theta2)
    return [[-L1*s1 - L2*s12, -L2*s12],
            [ L1*c1 + L2*c12,  L2*c12]]

# =================
# Tiny 2x2 helpers
# =================
def mat2_inv(M):
    det = M[0][0]*M[1][1] - M[0][1]*M[1][0]
    if abs(det) < 1e-9:
        return None
    inv = 1.0/det
    return [[ M[1][1]*inv, -M[0][1]*inv],
            [-M[1][0]*inv,  M[0][0]*inv]]

def mat2_mul_vec2(M, v):
    return [M[0][0]*v[0] + M[0][1]*v[1],
            M[1][0]*v[0] + M[1][1]*v[1]]

def vec2_norm(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1])


def clamp_to_workspace(x, y):
    """
    Project (x,y) into the reachable annulus [|L1-L2|, L1+L2] with tiny buffers.
    """
    r    = math.hypot(x, y)
    rmin = abs(L1 - L2) + 0.1    # 0.1 mm inside the inner boundary
    rmax = (L1 + L2) - 0.1       # 0.1 mm inside the outer boundary
    if r < 1e-6:
        return (rmin, 0.0)
    if r < rmin:
        s = rmin / r
        return (x*s, y*s)
    if r > rmax:
        s = rmax / r
        return (x*s, y*s)
    return (x, y)

def line_waypoints(start, goal, max_step=20.0):
    """
    Straight-line interpolation in mm. Default step = 20 mm.
    """
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    dist = math.hypot(dx, dy)
    if dist <= max_step:
        return [goal]
    n = int(math.ceil(dist / max_step))
    return [(start[0] + (i/n)*dx, start[1] + (i/n)*dy) for i in range(1, n+1)]


def ik2r_seed(x, y, elbow_up=True):
    """
    Closed-form IK seed for target (x,y) in mm.
    Returns (theta1, theta2) in radians.
    elbow_up=True => +sqrt branch; elbow_up=False => -sqrt branch.
    """
    r2 = x*x + y*y
    c2 = (r2 - L1*L1 - L2*L2) / (2.0*L1*L2)
    # Clamp for numerical safety
    if c2 >  1.0: c2 = 1.0
    if c2 < -1.0: c2 = -1.0
    s2_abs = math.sqrt(max(0.0, 1.0 - c2*c2))
    s2 = +s2_abs if elbow_up else -s2_abs

    theta2 = math.atan2(s2, c2)
    k1 = L1 + L2*c2
    k2 = L2*s2
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return (theta1, theta2)

def pick_elbow_branch_for_target(x, y):
    
    return True if y >= 0.0 else False


def clamp_step(dtheta, max_step_deg=10.0):
    """
    Limit per-iteration joint update magnitude to avoid huge Newton jumps.
    """
    max_rad = max_step_deg * DEG
    m = max(abs(dtheta[0]), abs(dtheta[1]))
    if m > max_rad:
        s = max_rad / m
        dtheta[0] *= s
        dtheta[1] *= s
    return dtheta


def get_current_joint_angles():
    j1_deg = m1.position    # EV3 reports degrees
    j2_deg = m2.position
    return [j1_deg*DEG, j2_deg*DEG]

def goto_joint_angles(theta1, theta2, speed_dps=SPEED_DPS):
    j1_deg = theta1/DEG
    j2_deg = theta2/DEG
    # Keep your m1 sign flip (mirrored mounting)
    m1.on_to_position(SpeedDPS(speed_dps), -j1_deg)
    m2.on_to_position(SpeedDPS(speed_dps),  j2_deg)


def move_to_xy(x_target_mm, y_target_mm):
    print("IK move to target (mm):", x_target_mm, y_target_mm)

    # clamp target in mm
    x_target, y_target = clamp_to_workspace(x_target_mm, y_target_mm)
    print("clamped target (mm):", x_target, y_target)

    # start waypoints from HOME (as requested)
    xy_start = (X_HOME, Y_HOME)    # (185, 0) mm

    # seed Newton with analytic IK on the correct elbow branch
    elbow_up = pick_elbow_branch_for_target(x_target, y_target)
    theta = list(ik2r_seed(x_target, y_target, elbow_up=elbow_up))

    # build small waypoints (default 20 mm)
    waypoints = line_waypoints(xy_start, (x_target, y_target), max_step=20.0)
    print("waypoints (mm):", waypoints)

    # refine each waypoint with Newton + step clamp (1 mm tolerance)
    for wp in waypoints:
        for _ in range(MAX_ITER):
            xy = fk_2r(theta[0], theta[1])         # [x,y] in mm
            e  = [wp[0] - xy[0], wp[1] - xy[1]]    # mm
            if vec2_norm(e) < 1.0:                 # 1 mm tol
                break

            J = jacobian_2r(theta[0], theta[1])    # mm/rad
            J_inv = mat2_inv(J)
            if J_inv is None:
                print("Singular Jacobian at waypoint (mm):", wp)
                break

            dtheta = mat2_mul_vec2(J_inv, e)       # rad
            dtheta = clamp_step(dtheta, max_step_deg=10.0)
            theta[0] += dtheta[0]
            theta[1] += dtheta[1]

        print("Reached wp (mm):", wp,
              " err(mm):", e,
              " theta(deg):", round(theta[0]/DEG,2), round(theta[1]/DEG,2))
        goto_joint_angles(theta[0], theta[1])


if __name__ == "__main__":
    # Try a few (all in mm):
    # move_to_xy(115.0,  80.0)
    # move_to_xy(115.0, -80.0)
    # move_to_xy(135.0, -40.0)
    # move_to_xy(140.0, -60.0)

    # move_to_xy(120,  80)   # Up-left of home
    # move_to_xy(100, 100)   # Diagonal upward
    # move_to_xy( 80, 150)   # Near vertical

    # move_to_xy(120, -80)   # Down-left of home
    move_to_xy(100,-100)   # Diagonal downward
    # move_to_xy(140, -60)   # Your earlier test
