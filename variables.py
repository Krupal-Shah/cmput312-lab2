from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sensor import INPUT_1

# Note: Link 1 will always have negated angle compared to link 2
# because of the way the motors are mounted on the robot arm
# lengths of the two links

l1 = 11.5   # in cm
l2 = 7.0    # in cm

LINK_1 = OUTPUT_A
LINK_2 = OUTPUT_B

link_1_motor = LargeMotor(OUTPUT_A)
link_2_motor = LargeMotor(OUTPUT_B)

touch_sensor = TouchSensor(INPUT_1)  # on port 1
