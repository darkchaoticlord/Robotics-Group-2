import brickpi3
import math
import time
import numpy as np

BP = brickpi3.BrickPi3()

ENCODER_MULTIPLIER_LINEAR = 27
ENCODER_MULTIPLIER_ANGULAR = 25
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
    reset_encoders()
    encoder_right, encoder_left = get_encoding()

    BP.set_motor_dps(LEFT_MOTOR, 300)
    BP.set_motor_dps(RIGHT_MOTOR, 300)
    encoded = distance_cm * ENCODER_MULTIPLIER_LINEAR
    while encoder_right <= encoded and encoder_left <= encoded:
        encoder_right, encoder_left = get_encoding()
        #time.sleep(0.02) #Sleep to reduce load on pi

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)

def convert_angle_to_distance(angle_deg):
    return 7 * math.pi / 180 * angle_deg

def turn_anticlockwise(angle_deg):
    reset_encoders()
    encoder_right, encoder_left = get_encoding()

    BP.set_motor_dps(LEFT_MOTOR, -100)
    BP.set_motor_dps(RIGHT_MOTOR, 100)

    #TODO: Properly calibrate the ENCODER_VAL (one for straight line and one for rotation?)
    encoded = convert_angle_to_distance(angle_deg) * ENCODER_MULTIPLIER_ANGULAR

    while encoder_left >= (-1 * encoded) or encoder_right <= encoded:
        encoder_right, encoder_left = get_encoding()
        #time.sleep(0.02) Sleep to reduce load on pi

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)

def turn_clockwise(angle_deg):
    reset_encoders()
    encoder_right, encoder_left = get_encoding()

    BP.set_motor_dps(LEFT_MOTOR, 100)
    BP.set_motor_dps(RIGHT_MOTOR, -100)

    encoded = convert_angle_to_distance(angle_deg) * ENCODER_MULTIPLIER_ANGULAR

    while encoder_right >= (-1 * encoded) or encoder_left <= encoded:
        encoder_right, encoder_left = get_encoding()
        #time.sleep(0.02) Sleep to reduce load on pi

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)

def turn(angle_deg):
    if(angle_deg == 0):
        return

    if(angle_deg > 0):
        turn_anticlockwise(angle_deg)

    if(angle_deg < 0):
        turn_clockwise(-1*angle_deg)
    


class State:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    # Keep any angle we move through between -pi and pi
    def normalizePi(self, val):
        while val <= -math.pi:
            val += 2*math.pi

        while val >= math.pi:
            val -= 2*math.pi

        return val

    # Uses the position based path planning equations from lecture 2 to calculate the motion required to reach positon (world_x, world_y)
    # Returns 3 values: beta = angle to rotate through to point towards desired point, 
    #                   d = distance to drive to reach desired point, 
    def calculate_motion(self, world_x, world_y):
        dx = world_x - self.x
        dy = world_y - self.y

        alpha = math.atan2(dy, dx)

        beta = self.normalizePi(alpha - self.theta)

        print(dx)
        print(dy)
        print(math.degrees(alpha))
        print(math.degrees(beta))

        d = math.sqrt(dx**2 + dy**2)

        return (beta, d)

    def navigate_to_waypoint(self, world_x, world_y):
        angle_to_rotate, distance = self.calculate_motion(world_x, world_y)

        print("turning: " + str(angle_to_rotate * 180/math.pi))
        print("moving: " + str(distance))
        
        turn(angle_to_rotate * 180 / math.pi)
        time.sleep(0.02)
        drive_straight(distance)

        # Update our current state
        self.x = world_x
        self.y = world_y
        self.theta += angle_to_rotate

        print(self.x, self.y, math.degrees(self.theta))
        

def main():
    try:
        current_state = State(0,0,0)
        while(True):
            x = input("Enter x coordinate: ")
            y = input("Enter y coordinate: ")

            print('Moving to: ', x, ', ', y)

            current_state.navigate_to_waypoint(x, y)        

    except KeyboardInterrupt:
        BP.reset_all() # This will prevent the robot moving if the program is interrupted or exited


if __name__ == "__main__":
    main()
