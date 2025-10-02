#!/usr/bin/env python3
import variables as v
from ev3dev2.motor import SpeedDPS
from time import sleep
import math
import random

# ------------------ Helpers ------------------ #


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1]]


def vec_scale(a, s):
    return [a[0]*s, a[1]*s]


def norm(v):
    return math.sqrt(v[0]**2 + v[1]**2)


def mat_vec_mul(M, v):
    return [M[0][0]*v[0] + M[0][1]*v[1],
            M[1][0]*v[0] + M[1][1]*v[1]]


def mat_add(A, B):
    return [[A[0][0]+B[0][0], A[0][1]+B[0][1]],
            [A[1][0]+B[1][0], A[1][1]+B[1][1]]]


def outer(u, v):
    return [[u[0]*v[0], u[0]*v[1]],
            [u[1]*v[0], u[1]*v[1]]]

# Inverse of 2x2 matrix


def mat_inv(M, damping=1e-6):
    det = M[0][0]*M[1][1] - M[0][1]*M[1][0]
    if abs(det) < 1e-9:
        det += damping  # regularize
    return [[M[1][1]/det, -M[0][1]/det],
            [-M[1][0]/det,  M[0][0]/det]]

# ------------------ Robot functions ------------------ #


def evalRobot2D(l, theta):
    x = l[0]*math.cos(theta[0]) + l[1]*math.cos(theta[0]+theta[1])
    y = l[0]*math.sin(theta[0]) + l[1]*math.sin(theta[0]+theta[1])
    return [x, y]


def fdJacob(l, theta, alpha=1e-6):
    J = [[0, 0], [0, 0]]
    f1 = evalRobot2D(l, [theta[0]+alpha, theta[1]])
    f2 = evalRobot2D(l, [theta[0]-alpha, theta[1]])
    J[0][0] = (f1[0]-f2[0])/(2*alpha)
    J[1][0] = (f1[1]-f2[1])/(2*alpha)
    f3 = evalRobot2D(l, [theta[0], theta[1]+alpha])
    f4 = evalRobot2D(l, [theta[0], theta[1]-alpha])
    J[0][1] = (f3[0]-f4[0])/(2*alpha)
    J[1][1] = (f3[1]-f4[1])/(2*alpha)
    return J

# ------------------ Inverse Kinematics ------------------ #


def alg_inv_kin(x, y):
    if x == 0 or y == 0:
        return 0, 0

    theta2 = math.acos((x**2 + y**2 - v.l1**2 - v.l2**2) / (2 * v.l1 * v.l2))
    k1 = v.l1 + v.l2 * math.cos(theta2)
    k2 = v.l2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    return theta1, theta2


def invKin2D(l, theta0, pos, n, mode):
    tol = 1e-2
    alpha = 0.3  # damping factor for stability

    # Workspace check
    if norm(pos) > sum(l):
        print("Target unreachable: outside workspace")
        return None

    if mode == 0:  # Newton
        for k in range(n):
            J = fdJacob(l, theta0)
            xk = evalRobot2D(l, theta0)
            fk = vec_sub(pos, xk)

            if norm(fk) < tol:
                print(f"Newton converged in {k} steps, error {norm(fk):.4f}")
                break

            invJ = mat_inv(J)
            sk = mat_vec_mul(invJ, fk)
            theta0 = vec_add(theta0, vec_scale(sk, alpha))  # damped update
        return theta0

    elif mode == 1:  # Broyden
        B = fdJacob(l, theta0)
        xk = evalRobot2D(l, theta0)
        fk = vec_sub(pos, xk)

        for k in range(n):
            if norm(fk) < tol:
                print(f"Broyden converged in {k} steps, error {norm(fk):.4f}")
                break

            invB = mat_inv(B)
            sk = mat_vec_mul(invB, fk)
            theta_new = vec_add(theta0, vec_scale(sk, alpha))
            x_new = evalRobot2D(l, theta_new)
            fk_new = vec_sub(pos, x_new)

            yk = vec_sub(fk_new, fk)
            denom = sk[0]*sk[0] + sk[1]*sk[1]
            if abs(denom) > 1e-9:
                upd = outer(vec_sub(yk, mat_vec_mul(B, sk)), sk)
                B = mat_add(B, [[upd[i][j]/denom for j in range(2)]
                            for i in range(2)])

            theta0 = theta_new
            fk = fk_new
        return theta0


pos = [x, y] = 11.5, -7.0
speed = 50
theta0 = [math.radians(10), math.radians(10)]
mode = 1  # 0: Newton, 1: Broyden
n = 25  # max iterations

theta1, theta2 = alg_inv_kin(x, y)
theta1_, theta2_ = invKin2D([v.l1, v.l2], theta0, pos, n, mode)

theta1 = -math.degrees(theta1)
theta2 = math.degrees(theta2)
theta1_ = -math.degrees(theta1_)
theta2_ = math.degrees(theta2_)

print("For position %.2f %.2f, moving link1 %.2f and link2 %.2f" %
      (x, y, theta1, theta2))
print("For position %.2f %.2f, moving link1 %.2f and link2 %.2f" %
      (x, y, theta1_, theta2_))

# v.link_1_motor.on_for_degrees(SpeedDPS(speed), theta1)
# v.link_2_motor.on_for_degrees(SpeedDPS(speed), theta2)
