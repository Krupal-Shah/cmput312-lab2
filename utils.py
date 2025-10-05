import math
import variables as v
from time import sleep
from ev3dev2.motor import SpeedDPS


### Math Matrix Utility Functions ###
def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1]]


def vec_scale(a, s):
    return [a[0]*s, a[1]*s]


def norm(v):
    return math.sqrt(v[0]**2 + v[1]**2)


def mat_mul_vec(M, v):
    return [M[0][0]*v[0] + M[0][1]*v[1],
            M[1][0]*v[0] + M[1][1]*v[1]]


def mat_add(A, B):
    return [[A[0][0]+B[0][0], A[0][1]+B[0][1]],
            [A[1][0]+B[1][0], A[1][1]+B[1][1]]]

def mat_mul_mat(A, B):
    return [[A[0][0]*B[0][0] + A[0][1]*B[1][0],
             A[0][0]*B[0][1] + A[0][1]*B[1][1]],
            [A[1][0]*B[0][0] + A[1][1]*B[1][0],
             A[1][0]*B[0][1] + A[1][1]*B[1][1]]]

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

def mat_transpose(M):
    return [[M[0][0], M[1][0]],
            [M[0][1], M[1][1]]]


def to_degrees(angle):
    return [math.degrees(angle[0]), math.degrees(angle[1])]


def to_radians(angle):
    return [math.radians(angle[0]), math.radians(angle[1])]


### Robot Kinematics Functions ###
def fk_2r(theta1, theta2):
    """Forward kinematics for 2R planar robot."""
    x = v.l1 * math.cos(theta1) + v.v.l2 * math.cos(theta1 + theta2)
    y = v.l1 * math.sin(theta1) + v.v.l2 * math.sin(theta1 + theta2)
    return x, y


def get_current_joint_angles():
    l1_deg = v.link_1_motor.position 
    v.l2_deg = v.link_2_motor.position 
    return [l1_deg, v.l2_deg]


def move_to_angles(speed, angle):
    _, angle = clamp_to_workspace(angle)
    print("Moving to angles (deg): ", angle)

    v.link_1_motor.on_to_position(SpeedDPS(speed), -angle[0])
    v.link_2_motor.on_to_position(SpeedDPS(speed), angle[1])
    sleep(2)


def clamp_to_workspace(angle):
    clamped = False
    angle1 = angle[0]
    angle2 = angle[1]
    
    if angle1 > 60:
        angle1 = 60
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

    # Apply sign convention (negated motor 1 if mirrored)
    theta1 = math.radians(-theta1_deg)
    theta2 = math.radians(theta2_deg)

    x, y = fk_2r(theta1, theta2)

    print("\n--- Forward Kinematics ---")
    print("m1 (theta1):", theta1_deg, "deg")
    print("m2 (theta2):", theta2_deg, "deg")
    print("End Effector: x =", x, "cm, y =", y, "cm")

    return (x, y)

def jacobian_2r(theta):
    s1, c1 = math.sin(theta[0]), math.cos(theta[0])
    s12, c12 = math.sin(theta[0] + theta[1]), math.cos(theta[0] + theta[1])
    return [[-v.l1*s1 - v.l2*s12, -v.l2*s12],
            [ v.l1*c1 + v.l2*c12,  v.l2*c12]]