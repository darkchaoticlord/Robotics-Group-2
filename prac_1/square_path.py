import brickpi3
import math
import time

BP = brickpi3.BrickPi3()

ENCODER_VAL_FOR_ONE_CM = 27
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

   # BP.set_motor_power(LEFT_MOTOR, 39)
   # BP.set_motor_power(RIGHT_MOTOR, 35)
    BP.set_motor_dps(LEFT_MOTOR, 300)
    BP.set_motor_dps(RIGHT_MOTOR, 300)
    encoded = distance_cm * ENCODER_VAL_FOR_ONE_CM 
    while encoder_right <= encoded and encoder_left <= encoded:
        encoder_right, encoder_left = get_encoding()
        #time.sleep(0.02) #Sleep to reduce load on pi

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)

def convert_angle_to_distance(angle_deg):
    return 7 * math.pi / 180 * angle_deg


def turn_clockwise(angle_deg):
    encoder_right, encoder_left = get_encoding()

    #BP.set_motor_power(LEFT_MOTOR, 40)
    #BP.set_motor_power(RIGHT_MOTOR, -36)
    BP.set_motor_dps(LEFT_MOTOR, 100)
    BP.set_motor_dps(RIGHT_MOTOR, -100)
    #encoded = convert_angle_to_distance(angle_deg) * ENCODER_VAL_FOR_ONE_CM
    encoded = 280
    while encoder_right >= (-1 * encoded) or encoder_left <= encoded:
        encoder_right, encoder_left = get_encoding()
        print("right: ", encoder_right)
        print("left: ", encoder_left)
        #time.sleep(0.02) Sleep to reduce load on pi

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)


def main():
    try:
        for _ in range(4):

            # Theres a bug that occurs if we dont sleep between commands to the BrickPi. It doesnt register the commands we send it and block. Hence the sleeps in between the commands

            reset_encoders()
            time.sleep(0.02)
            drive_straight(40)
            reset_encoders()
            time.sleep(0.02)
            turn_clockwise(90)

    except KeyboardInterrupt:
        BP.reset_all() # This will prevent the robot moving if the program is interrupted or exited


if __name__ == "__main__":
    main()
