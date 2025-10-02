import math
import variables as v
from time import sleep
from ev3dev2.motor import SpeedDPS


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


def det(M):
    return M[0][0]*M[1][1] - M[0][1]*M[1][0]


def mat_inv(M, damping=1e-6):
    det_ = det(M)
    if abs(det_) < 1e-9:
        det_ += damping  # regularize
    return [[M[1][1]/det_, -M[0][1]/det_],
            [-M[1][0]/det_,  M[0][0]/det_]]


def move_to_angles(speed, angle):
    v.link_1_motor.on_to_position(SpeedDPS(speed), -angle[0])
    v.link_2_motor.on_to_position(SpeedDPS(speed), angle[1])
    sleep(2)

def to_degrees(angle):
    return [math.degrees(angle[0]), math.degrees(angle[1])]

def to_radians(angle):
    return [math.radians(angle[0]), math.radians(angle[1])]