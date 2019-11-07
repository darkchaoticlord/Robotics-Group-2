import brickpi3
import math
import time
import random
import numpy as np
import copy
import motion_commands as mc

BP = brickpi3.BrickPi3()

ENCODER_VAL_FOR_ONE_CM = 27
LEFT_MOTOR = BP.PORT_B
RIGHT_MOTOR = BP.PORT_A
NUM_OF_PARTICLES = 100

# Class representing the Particle in our particle filter
class Particle:

    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

    #NOTE: THETA has not been capped between 0 and 360 -> will cause problems

    def update_linear_motion(self, D, std_dev_e, std_dev_f):
        self.x += (D + self.get_gaussian_error(std_dev_e)) * np.cos(math.radians(self.theta))
        self.y += (D + self.get_gaussian_error(std_dev_e)) * np.sin(math.radians(self.theta))
        self.theta = (self.theta + self.get_gaussian_error(std_dev_f)) % 360

    def update_rotation_motion(self, alpha, std_dev_g):
        self.theta = (self.theta + alpha + self.get_gaussian_error(std_dev_g)) % 360
    
    def get_gaussian_error(self, std_dev):
        return random.gauss(0, std_dev)

    def return_tuple(self):
        return self.x, self.y, self.theta

class ParticleSet:

    def __init__(self, particles):
        self.particles = particles

    def update_linear_motions(self, D, std_dev_e, std_dev_f):
        for particle in self.particles:
            particle.update_linear_motion(D, std_dev_e, std_dev_f)

    def update_rotation_motions(self, alpha, std_dev_g):
        for particle in self.particles:
            particle.update_rotation_motion(alpha, std_dev_g)

    def __str__(self):
        return str([particle.return_tuple() for particle in self.particles])

# class Line:
#
#     def __init__(self, start, end):
#         self.start = start
#         self.end = end
#
#     def __str__(self):
#         return "({}, {}, {}, {})".format(self.start[0], self.start[1],
#                                          self.end[0], self.end[1])

def main():
    try:
        mc.init_motors()
        particle_set = ParticleSet([Particle(100,500,0,1/NUM_OF_PARTICLES) for _ in range(NUM_OF_PARTICLES)])

        e, f, g = 2, 1, 2

        # draw grid
        for x in range(9):
            value = x * 50 + 100
            print("drawLine: ({}, 100, {}, 500)".format(value, value))

        for y in range(9):
            value = y * 50 + 100
            print("drawLine: (100, {}, 500, {})".format(value, value))

        for _ in range(4):
            # Theres a bug that occurs if we dont sleep between 
            # commands to the BrickPi. It doesnt register the 
            # commands we send it and block. Hence the sleeps 
            # in between the commands
            for _ in range(4):
                D = 10
                mc.drive_straight(D)
                particle_set.update_linear_motions(10 * D, e, f)

                #for ps in updated_values:
                print("drawParticles: " + str(particle_set))


            mc.turn_anticlockwise(90)
            particle_set.update_rotation_motions(-90, g)

    except KeyboardInterrupt:
        BP.reset_all() # This will prevent the robot moving if the program is interrupted or exited


if __name__ == "__main__":
    main()
