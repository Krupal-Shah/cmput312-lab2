#!/usr/bin/env python3
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS
# import matplotlib.pyplot as plt
# link lengths (mm)
L1, L2 = 115, 70 

# default speed
SPEED_DPS = 60
MAX_ITER = 50

# initial/home position
X_HOME = L1+L2
Y_HOME = 0

m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)

DEG = math.pi/180.0

# T1_MIN = 80.0 * DEG
# T1_MAX =  -60.0 * DEG
# T2_MIN = -140.0 * DEG
# T2_MAX =   160.0 * DEG

# def clamp_joint_limits(t1, t2):
#     if t1 < T1_MIN: t1 = T1_MIN
#     if t1 > T1_MAX: t1 = T1_MAX
#     if t2 < T2_MIN: t2 = T2_MIN
#     if t2 > T2_MAX: t2 = T2_MAX
#     return [t1, t2]


m1.reset()
m2.reset()

print("postion after reset", m1.position, m2.position) # inital positions

def fk_2r(theta1, theta2):
    # elbow
    x1 = L1*math.cos(theta1)
    y1 = L1*math.sin(theta1)
    # end effector
    x2 = x1 + L2*math.cos(theta1+theta2)
    y2 = y1 + L2*math.sin(theta1+theta2)
    return (x1,y1),(x2,y2)

def jacobian_2r(theta1, theta2):
    s1, c1 = math.sin(theta1), math.cos(theta1)
    s12, c12 = math.sin(theta1+theta2), math.cos(theta1+theta2)
    return [[-L1*s1 - L2*s12, -L2*s12],
            [ L1*c1 + L2*c12,  L2*c12]]

def mat2_T(M):
    return [[M[0][0], M[1][0]],
            [M[0][1], M[1][1]]]

def mat2_mul_vec2(M, v):
    return [M[0][0]*v[0] + M[0][1]*v[1],
            M[1][0]*v[0] + M[1][1]*v[1]]

def mat2_mul_mat2(A, B):
    return [[A[0][0]*B[0][0] + A[0][1]*B[1][0],
             A[0][0]*B[0][1] + A[0][1]*B[1][1]],
            [A[1][0]*B[0][0] + A[1][1]*B[1][0],
             A[1][0]*B[0][1] + A[1][1]*B[1][1]]]

def mat2_inv(M):
    det = M[0][0]*M[1][1] - M[0][1]*M[1][0]
    if abs(det) < 1e-9:
        return None
    inv_det = 1.0/det
    return [[ M[1][1]*inv_det, -M[0][1]*inv_det],
            [-M[1][0]*inv_det,  M[0][0]*inv_det]]

def vec2_norm(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1])

def get_current_joint_angles():
    j1_deg = m1.position 
    j2_deg = m2.position 
    return [j1_deg*DEG, j2_deg*DEG]

def goto_joint_angles(theta1, theta2, speed_dps=SPEED_DPS):
    j1_deg = theta1/DEG
    j2_deg = theta2/DEG
    m1_deg = j1_deg 
    m2_deg = j2_deg 
    m1.on_to_position(SpeedDPS(speed_dps), -m1_deg) # added negative since motor is mirrored
    m2.on_to_position(SpeedDPS(speed_dps), m2_deg)

def clamp_to_workspace(x, y):
    r = math.hypot(x, y)
    rmin = abs(L1 - L2) + 1e-4
    rmax = (L1 + L2) - 1e-4
    if r < 1e-9: 
        return (rmin, 0.0)
    if r < rmin:
        s = rmin / r
        return (x*s, y*s)
    if r > rmax:
        s = rmax / r
        return (x*s, y*s)
    return (x, y)

def line_waypoints(start, goal, max_step=20.0):  # mm per step
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    dist = math.hypot(dx, dy)
    if dist <= max_step:
        return [goal]
    n = int(math.ceil(dist/max_step))
    return [(start[0] + (i/n)*dx, start[1] + (i/n)*dy) for i in range(1,n+1)]

def choose_newton_seed_for_target(x_goal, y_goal):
    """
    Pure-Newton seeding: try two tiny elbow bends (up vs down) at home,
    pick the one with smaller initial task-space error to the GOAL.
    """
    # Two tiny-bend candidates around the singular home pose
    candidates = ([+5*DEG, -5*DEG],   # elbow-up-ish
                  [-5*DEG, +5*DEG])   # elbow-down-ish
    best_t = None
    best_err = None
    for t in candidates:
        _, xy = fk_2r(t[0], t[1])
        e = [x_goal - xy[0], y_goal - xy[1]]

        en = vec2_norm(e)
        if (best_err is None) or (en < best_err):
            best_err = en
            best_t = [t[0], t[1]]
    return best_t

# def choose_newton_seed_for_target(x_goal, y_goal, bend_deg=5.0):
#     """
#     Try two tiny bends at home (elbow-up-ish vs elbow-down-ish).
#     Prefer seeds inside joint limits; if both outside, clamp both and
#     pick the one with smaller task-space error to the GOAL.
#     Returns [t1, t2] in radians.
#     """
#     seeds = ([+bend_deg*DEG, -bend_deg*DEG],
#              [-bend_deg*DEG, +bend_deg*DEG])

#     best_t = None
#     best_err = None
#     best_inside = -1  # 1 if inside, 0 if clamped/outside

#     for t in seeds:
#         # check if inside limits
#         inside = (T1_MIN <= t[0] <= T1_MAX) and (T2_MIN <= t[1] <= T2_MAX)
#         # clamp to limits if necessary
#         tc = clamp_joint_limits(t[0], t[1])
#         # evaluate error at (possibly clamped) seed
#         _, xy = fk_2r(tc[0], tc[1])
#         e = [x_goal - xy[0], y_goal - xy[1]]
#         en = vec2_norm(e)

#         # prefer inside-limits seeds; break ties by smaller error
#         rank_inside = 1 if inside else 0
#         if (rank_inside > best_inside) or (rank_inside == best_inside and (best_err is None or en < best_err)):
#             best_inside = rank_inside
#             best_err = en
#             best_t = [tc[0], tc[1]]

#     return best_t


def clamp_step(dtheta, max_step_deg=10.0):
    """Limit per-iteration joint change to avoid huge swings."""
    max_rad = max_step_deg * DEG
    m = max(abs(dtheta[0]), abs(dtheta[1]))
    if m > max_rad:
        s = max_rad / m
        dtheta[0] *= s
        dtheta[1] *= s
    return dtheta

def to_mm(x, y):
    if abs(x) < 0.5 and abs(y) < 0.5:
        return (x*1000.0, y*1000.0)
    return (x, y)

def move_to_xy(x_target, y_target, start_x=None, start_y=None):
    # normalize units & clamp
    x_target, y_target = to_mm(x_target, y_target)
    print("Inverse Kinematics - move arm to (x,y) target (mm)", x_target, y_target)
    x_target, y_target = clamp_to_workspace(x_target, y_target)
    print("(x,y) after clamping (mm)", x_target, y_target)

    # start from HOME in workspace; seed elbow branch by goal
    theta = choose_newton_seed_for_target(x_target, y_target)
    xy_start = (X_HOME, Y_HOME)
    
    # start_x and start_y will be defined from read_arm 2nd position
    if start_x and start_y:
        xy_start = (start_x,start_y)

    # 5 mm waypoints
    waypoints = line_waypoints(xy_start, (x_target, y_target), max_step=5.0)
    print("way points generated:", waypoints)

    # Newton per waypoint (1 mm tol) + step clamp
    for wp in waypoints:
        for _ in range(MAX_ITER):
            _, xy = fk_2r(theta[0], theta[1])          # end-effector in mm
            e  = [wp[0] - xy[0], wp[1] - xy[1]]        # mm
            if vec2_norm(e) < 1.0:                     # 1 mm tolerance
                break

            J = jacobian_2r(theta[0], theta[1])
            J_inv = mat2_inv(J)
            if J_inv is None:
                print("Singular Jacobian at waypoint", wp)
                break

            dtheta = mat2_mul_vec2(J_inv, e)           # rad
            dtheta = clamp_step(dtheta, max_step_deg=10.0)
            

            theta[0] += dtheta[0]
            theta[1] += dtheta[1]
            # theta = clamp_joint_limits(theta[0], theta[1])

            print("Waypoint:", wp, " Error (mm):", e, " dtheta (rad):", dtheta)

        goto_joint_angles(theta[0], theta[1])

# def simulate_to_xy(x_target, y_target):
#     # 0) clamp target
#     x_target,y_target = clamp_to_workspace(x_target,y_target)

#     # 1) start at home (straight arm along +x)
#     theta=[5*DEG, -5*DEG]
#     (_, _), (xe,ye) = fk_2r(theta[0],theta[1])
#     # xy_start=(xe,ye)
#     xy_start = (X_HOME, Y_HOME)

#     # 2) build waypoints
#     waypoints=line_waypoints(xy_start,(x_target,y_target),max_step=0.02)

#     # 3) prepare plot
#     fig,ax=plt.subplots()
#     ax.set_aspect('equal')
#     ax.set_xlim(-0.2,0.2); ax.set_ylim(-0.05,0.2)

#     traj=[xy_start]

#     # 4) solve for each waypoint (just like your loop)
#     for wp in waypoints:
#         for _ in range(MAX_ITER):
#             (_, _),(xe,ye)=fk_2r(theta[0],theta[1])
#             e=[wp[0]-xe, wp[1]-ye]
#             if vec2_norm(e)<1e-3: break
#             J=jacobian_2r(theta[0],theta[1]); J_inv=mat2_inv(J)
#             if J_inv is None:
#                 print("Singular Jacobian at",wp)
#                 break
#             dtheta=mat2_mul_vec2(J_inv,e)
#             theta[0]+=dtheta[0]; theta[1]+=dtheta[1]
#         # after solving this waypoint, draw arm
#         (x1,y1),(xe,ye)=fk_2r(theta[0],theta[1])
#         ax.plot([0,x1,xe],[0,y1,ye],'o-',alpha=0.5)
#         traj.append((xe,ye))

#     # plot full end-effector trajectory
#     xs=[p[0] for p in traj]; ys=[p[1] for p in traj]
#     ax.plot(xs,ys,'r--',label="end effector path")
#     ax.legend(); plt.show()

# Example
def main():
    # simulate_to_xy(0.0,0.20)
    print("")
    # test case:
    
    # move_to_xy(X_HOME, Y_HOME)  
   
    # move_to_xy(115,  80)
    # move_to_xy(115, -80)
    # move_to_xy(135, -40)
    # move_to_xy(135, 40)

    # move_to_xy(140, -60)
    # move_to_xy(180,  10)
    # move_to_xy( 60, -30)
    # move_to_xy(40, -150)
    # move_to_xy(40, 150)
    move_to_xy(60,60)


if __name__ == "__main__":
    main()