import brickpi3
import math
import time
import numpy as np

BP = brickpi3.BrickPi3()

ENCODER_MULTIPLIER_LINEAR = 27
ENCODER_MULTIPLIER_ANGULAR = 27
LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_A

def init_motors():
    BP.reset_all()

# Reset motor encoders to 0
def reset_encoders():
    BP.offset_motor_encoder(RIGHT_MOTOR, BP.get_motor_encoder(RIGHT_MOTOR))
    BP.offset_motor_encoder(LEFT_MOTOR, BP.get_motor_encoder(LEFT_MOTOR))

# Get current encoding
def get_encoding():
    encoder_right = BP.get_motor_encoder(RIGHT_MOTOR)
    encoder_left = BP.get_motor_encoder(LEFT_MOTOR)

    return (encoder_right, encoder_left)

# Drive in straight line
def drive_straight(distance_cm):
    time.sleep(0.5)
    reset_encoders()
    encoder_right, encoder_left = get_encoding()

    BP.set_motor_dps(LEFT_MOTOR, 300)
    BP.set_motor_dps(RIGHT_MOTOR, 300)
    encoded = distance_cm * ENCODER_MULTIPLIER_LINEAR
    while encoder_right <= encoded and encoder_left <= encoded:
        encoder_right, encoder_left = get_encoding()
        #time.sleep(0.02) #Sleep to reduce load on pi

    BP.set_motor_dps(RIGHT_MOTOR, 0)
    BP.set_motor_dps(LEFT_MOTOR, 0)

    time.sleep(0.5)

def convert_angle_to_distance(angle_deg):
    return 7 * math.pi / 180 * angle_deg

def turn_anticlockwise(angle_deg):
    time.sleep(0.5)
    reset_encoders()
    encoder_right, encoder_left = get_encoding()

    BP.set_motor_dps(LEFT_MOTOR, -100)
    BP.set_motor_dps(RIGHT_MOTOR, 100)

    encoded = convert_angle_to_distance(angle_deg) * ENCODER_MULTIPLIER_ANGULAR

    while encoder_left >= (-1 * encoded) or encoder_right <= encoded:
        encoder_right, encoder_left = get_encoding()
        #time.sleep(0.02) Sleep to reduce load on pi

    BP.set_motor_dps(RIGHT_MOTOR, 0)
    BP.set_motor_dps(LEFT_MOTOR, 0)
    time.sleep(0.5)

def turn_clockwise(angle_deg):
    time.sleep(0.5)
    reset_encoders()
    encoder_right, encoder_left = get_encoding()

    BP.set_motor_dps(LEFT_MOTOR, 100)
    BP.set_motor_dps(RIGHT_MOTOR, -100)

    encoded = convert_angle_to_distance(angle_deg) * ENCODER_MULTIPLIER_ANGULAR

    while encoder_right >= (-1 * encoded) or encoder_left <= encoded:
        encoder_right, encoder_left = get_encoding()
        #time.sleep(0.02) Sleep to reduce load on pi

    BP.set_motor_dps(RIGHT_MOTOR, 0)
    BP.set_motor_dps(LEFT_MOTOR, 0)
    time.sleep(0.5)

def turn(angle_deg):
    if(angle_deg == 0):
        return

    if(angle_deg > 0):
        turn_anticlockwise(angle_deg)

    if(angle_deg < 0):
        turn_clockwise(-1*angle_deg)
