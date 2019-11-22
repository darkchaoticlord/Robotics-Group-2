import brickpi3
import time
import numpy as np

BP = brickpi3.BrickPi3()

SPIN_CLOCKWISE = True


class Sensor:
    clockwise = True
    first_sensor_reading = True

    def __init__(self, motor_port, sensor_port):
        self.sensor_port = sensor_port
        self.motor_port = motor_port
        self.get_sensor_reading() # Sets up the sensor
        #BP.set_motor_limits(motor_port, 80, 150)

    def reset_encoder(self):
        BP.offset_motor_encoder(self.motor_port, BP.get_motor_encoder(self.motor_port))

    def get_encoding(self):
        return BP.get_motor_encoder(self.motor_port)

    def spin_anticlockwise(self):
        time.sleep(0.02)
        self.reset_encoder()

        values = [(0,self.get_sensor_reading())]

        BP.set_motor_dps(self.motor_port, 150)
        pos = self.get_encoding()
        target = 1

        while pos < 360:
            if pos == target:
                values.append((target,self.get_sensor_reading()))
                target += 1

            pos = self.get_encoding()
        
        BP.set_motor_dps(self.motor_port, 0)

        return values

    def spin_clockwise(self):
        time.sleep(0.02)
        self.reset_encoder()

        values = [(0,self.get_sensor_reading())]

        BP.set_motor_dps(self.motor_port, -150)
        pos = self.get_encoding()
        target = 1

        while pos > -360:
            if -pos == target:
                values.append((target,self.get_sensor_reading()))
                target += 1

            pos = self.get_encoding()

        BP.set_motor_dps(self.motor_port, 0)

        return values

    def spin_and_measure(self):

        if self.clockwise:
            self.clockwise = False
            return self.spin_clockwise()

        else:
            self.clockwise = True
            return self.spin_anticlockwise()


    def get_sensor_reading(self):
        if(self.first_sensor_reading):
            BP.set_sensor_type(self.sensor_port, BP.SENSOR_TYPE.NXT_ULTRASONIC)
            time.sleep(1)
            self.first_sensor_reading = False

        # read and display the sensor value
        # BP.get_sensor retrieves a sensor value.
        # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
        # BP.get_sensor returns the sensor value (what we want to display).

        try:
            return BP.get_sensor(self.sensor_port)

        except brickpi3.SensorError as error:
            print(error)

        time.sleep(0.02)

try:
    BP.reset_all()
    s = Sensor(BP.PORT_C, BP.PORT_1)

    print(s.spin_and_measure())
    print(s.spin_and_measure())

except KeyboardInterrupt:
    BP.reset_all() # This will prevent the robot moving if the program is interrupted or exited

BP.reset_all()
