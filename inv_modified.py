#!/usr/bin/env python3
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedDPS
import matplotlib.pyplot as plt
# link lengths (meters)
L1, L2 = 0.115, 0.070

# default speed
SPEED_DPS = 60
MAX_ITER = 50

# initial/home position
X_HOME = L1+L2
Y_HOME = 0

m1 = LargeMotor(OUTPUT_A)
m2 = LargeMotor(OUTPUT_B)

DEG = math.pi/180.0

m1.reset()
m2.reset()

def fk_2r(theta):
    x = L1*math.cos(theta[0]) + L2*math.cos(theta[0]+theta[1])
    y = L1*math.sin(theta[0]) + L2*math.sin(theta[0]+theta[1])
    return [x, y]


def fdJacob(theta, alpha=1e-6):
    J = [[0, 0], [0, 0]]
    f1 = fk_2r([theta[0]+alpha, theta[1]])
    f2 = fk_2r([theta[0]-alpha, theta[1]])
    f3 = fk_2r([theta[0], theta[1]+alpha])
    f4 = fk_2r([theta[0], theta[1]-alpha])

    J[0][0] = (f1[0]-f2[0])/(2*alpha)
    J[1][0] = (f1[1]-f2[1])/(2*alpha)
    J[0][1] = (f3[0]-f4[0])/(2*alpha)
    J[1][1] = (f3[1]-f4[1])/(2*alpha)
    return J


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
    return [[M[1][1]*inv_det, -M[0][1]*inv_det],
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
    m1.on_to_position(SpeedDPS(speed_dps), -m1_deg)
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


def line_waypoints(start, goal, max_step=0.02):
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    dist = math.hypot(dx, dy)
    if dist <= max_step:
        return [goal]
    n = int(math.ceil(dist/max_step))
    return [(start[0] + (i/n)*dx, start[1] + (i/n)*dy) for i in range(1, n+1)]


def move_to_xy(x_target, y_target):
    # 0) clamp target to workspace
    x_target, y_target = clamp_to_workspace(x_target, y_target)

    # 1) compute current end effector (start point)
    theta = get_current_joint_angles()
    xy_start = (X_HOME, Y_HOME)

    # xy_start = fk_2r(theta[0], theta[1])

    if abs(theta[0]) < 1e-6 and abs(theta[1]) < 1e-6:
        theta = [5*DEG, -5*DEG]   # small bend to avoid singularity

    # 2) build waypoints from current position -> goal
    waypoints = line_waypoints(xy_start, (x_target, y_target), max_step=0.02)
    # 3) loop over waypoints
    for wp in waypoints:
        for _ in range(MAX_ITER):
            xy = fk_2r(theta[0], theta[1])
            e = [wp[0] - xy[0], wp[1] - xy[1]]
            if vec2_norm(e) < 1e-3:
                break

            J = fdJacob(theta)
            J_inv = mat2_inv(J)
            if J_inv is None:
                print("Singular Jacobian at waypoint", wp)
                break

            dtheta = mat2_mul_vec2(J_inv, e)
            theta[0] += dtheta[0]
            theta[1] += dtheta[1]
            print("Waypoint:", wp, " Error:", vec2_norm(e), " dtheta:", dtheta)
        # send robot to this waypoint solution
        goto_joint_angles(theta[0], theta[1])

# def simulate_to_xy(x_target, y_target):
#     # 0) clamp target
#     x_target,y_target = clamp_to_workspace(x_target,y_target)

#     # 1) start at home (straight arm along +x)
#     theta=[5*DEG, -5*DEG]
#     [xe,ye]=fk_2r(theta[0],theta[1])
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
#             (xe,ye)=fk_2r(theta[0],theta[1])
#             e=[wp[0]-xe, wp[1]-ye]
#             if vec2_norm(e)<1e-3: break
#             J=jacobian_2r(theta[0],theta[1]); J_inv=mat2_inv(J)
#             if J_inv is None:
#                 print("Singular Jacobian at",wp)
#                 break
#             dtheta=mat2_mul_vec2(J_inv,e)
#             theta[0]+=dtheta[0]; theta[1]+=dtheta[1]

            ## This will not work as I changed fk_2r to take a single argument
            ## instead of two arguments. Please fix it.
            ## Try just printing the values instead.
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

    move_to_xy(0., 0)


if __name__ == "__main__":
    main()
