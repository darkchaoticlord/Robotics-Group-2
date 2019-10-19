import brickpi3
import math

BP = brickpi3.BrickPi3()

ENCODER_VAL_FOR_ONE_CM = 27.25
LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_A


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
    encoder_right, encoder_left = get_encoding()

    BP.set_motor_power(LEFT_MOTOR, 39)
    BP.set_motor_power(RIGHT_MOTOR, 35)
    encoded = distance_cm * ENCODER_VAL_FOR_ONE_CM 
    while encoder_right < encoded and encoder_left < encoded:
        encoder_right, encoder_left = get_encoding()

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)

def convert_angle_to_distance(angle_deg):
    return 7 * math.pi / 180 * angle_deg


def turn_clockwise(angle_deg):
    encoder_right, encoder_left = get_encoding()

    BP.set_motor_power(LEFT_MOTOR, 40)
    BP.set_motor_power(RIGHT_MOTOR, -36)
    encoded = convert_angle_to_distance(angle_deg) * ENCODER_VAL_FOR_ONE_CM
    while encoder_right > (-1 * encoded) and encoder_left < encoded:
        encoder_right, encoder_left = get_encoding()

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)


def main():
    for _ in range(10):
        for _ in range(4):
            reset_encoders()
            drive_straight(40)
            reset_encoders()
            turn_clockwise(90)


if __name__ == "__main__":
    main()
