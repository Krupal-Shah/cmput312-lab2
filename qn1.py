#!/usr/bin/env python3
import variables as v
import utils as ut
import math

mode = 0  # 0: Move to angles, 1: Measure a line, 2: Measure an angle
speed = 50  # degrees per second

# Reset motor positions
v.link_1_motor.reset()
v.link_2_motor.reset()

if mode == 0:
    print("Mode: Move to angles")
    angle1 = 10
    angle2 = 10

    [angle1, angle2] = ut.clamp_to_workspace(angle1, angle2)
    ut.move_to_angles(speed, [angle1, angle2])

    theta = [math.radians(angle1), math.radians(angle2)]
    end_effector_pos = ut.fk_2r(theta)
    print("End Effector Position: ", end_effector_pos)

elif mode == 1:
    print("Mode: Measure a line")
    print("Press the touch sensor to record positions")
    (x1, y1) = ut.sample_point()
    (x2, y2) = ut.sample_point()
    print("Distance of line drawn: ", math.sqrt((x2 - x1)**2 + (y2 - y1)**2))

elif mode == 2:
    print("Mode: Measure an angle")
    print("Press the touch sensor to record positions")
    (x1, y1) = ut.sample_point()
    (x2, y2) = ut.sample_point()
    (x3, y3) = ut.sample_point()

    # https://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
    v1 = (x1 - x2, y1 - y2)
    v2 = (x3 - x2, y3 - y2)

    dot = v1[0]*v2[0] + v1[1]*v2[1]
    det = v1[0]*v2[1] - v1[1]*v2[0]
    theta = math.degrees(math.atan2(det, dot))
    print("Angle between the two lines is: ", abs(theta))
    
else:
    print("Invalid mode selected")
    exit(1)

v.link_1_motor.off()
v.link_2_motor.off()