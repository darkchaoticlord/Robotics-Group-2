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

    def resample():
        new_particle_set = []
        sum_of_weights = 0
        for particle in self.particles:
            #calculate_likelihood(particle, z)
            sum_of_weights += particle.weight

        # We can skip the normalization stage and instead sample from 0 to sum_of_weights

        # This runs in O(n^2), but we can use the Alias algorithm to make it in O(n)
        for i in range(NUM_OF_PARTICLES):
            prob = random.uniform(0, sum_of_weights)
            running_total = 0

            for particle in self.particles:
                running_total += particle.weight

                if prob <= running_total:
                    new_particle_set.append(particle)
                    break

                else
                    running_total += particle.weight
            


def calculate_likelihood(particle, z):
    pass

def main():
    # Seed our Random numbers
    random.seed()

    try:
        mc.init_motors()
        particle_set = ParticleSet([Particle(100,500,0,1/NUM_OF_PARTICLES) for _ in range(NUM_OF_PARTICLES)])

        e, f, g = 2, 1, 2

        # TODO: Draw map

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
