
# Just testing cyberduck
#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for reading a touch sensor connected to PORT_1 of the BrickPi3
# 
# Hardware: Connect an EV3 or NXT touch sensor to BrickPi3 Port 1.
# 
# Results:  When you run this program, you should see a 0 when the touch sensor is not pressed, and a 1 when the touch sensor is pressed.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import numpy as np

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

# Configure for a touch sensor.
# If an EV3 touch sensor is connected, it will be configured for EV3 touch, otherwise it's configured for NXT touch.
# BP.set_sensor_type configures the BrickPi3 for a specific sensor.
# BP.PORT_1 specifies that the sensor will be on sensor port 1.
# BP.SENSOR_TYPE.TOUCH specifies that the sensor will be a touch sensor.
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.TOUCH)
BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.TOUCH)

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
    BP.set_motor_dps(LEFT_MOTOR, np.sign(distance_cm) * 300)
    BP.set_motor_dps(RIGHT_MOTOR, np.sign(distance_cm) * 300)
    encoded = distance_cm * ENCODER_VAL_FOR_ONE_CM 
    while encoder_right <= encoded and encoder_left <= encoded:
        encoder_right, encoder_left = get_encoding()
        #time.sleep(0.02) #Sleep to reduce load on pi

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)

def convert_angle_to_distance(angle_deg):
    return 7 * math.pi / 180 * angle_deg


def turn(angle_deg):
    encoder_right, encoder_left = get_encoding()

    #BP.set_motor_power(LEFT_MOTOR, 40)
    #BP.set_motor_power(RIGHT_MOTOR, -36)
    BP.set_motor_dps(LEFT_MOTOR, np.sign(angle_deg) * 100)
    BP.set_motor_dps(RIGHT_MOTOR, - np.sign(angle_deg) * 100)
    #encoded = convert_angle_to_distance(angle_deg) * ENCODER_VAL_FOR_ONE_CM
    encoded = 280
    while encoder_right >= (-1 * encoded) or encoder_left <= encoded:
        encoder_right, encoder_left = get_encoding()
        print("right: ", encoder_right)
        print("left: ", encoder_left)
        #time.sleep(0.02) Sleep to reduce load on pi

    BP.set_motor_power(RIGHT_MOTOR, 0)
    BP.set_motor_power(LEFT_MOTOR, 0)

# clockwise == 1 if the robot needs to go right, clockwise == -1 otherwise
def back_and_turn(clockwise):
    time.sleep(0.02)

    BP.set_motor_power(LEFT_MOTOR, 0)
    BP.set_motor_power(RIGHT_MOTOR, 0)

    drive_straight( -20 )

    time.sleep(0.02)
    
    turn( clockwise * 90 )

    time.sleep(0.02)


try:
    while True:
        # read and display the sensor value
        # BP.get_sensor retrieves a sensor value.
        # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
        # BP.get_sensor returns the sensor value (what we want to display).
        
        BP.set_motor_dps(LEFT_MOTOR, 300)
        BP.set_motor_dps(RIGHT_MOTOR, 300)
        try:
            valueLeft = BP.get_sensor(BP.PORT_1)
            valueRight = BP.get_sensor(BP.PORT_2)
            
            if ( valueLeft == 1 and valueRight == 0 ):
                back_and_turn(1)
            elif ( valueright = 1 ):
                back_and_turn( -1 )
                

            

        except brickpi3.SensorError as error:
            print(error)
        
        time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
