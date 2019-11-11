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

mymap=[] # list of lines

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

class RobotsPosition:
 
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    # sets self.x, self.y, self.theta to the  weighted average of the locations in particles, weighted by their corresponding weights
    def robots_Position(self, particleSet):
        xs = np.array([ particle.x for particle in particleSet.particles ])
        ys = np.array([ particle.y for particle in particleSet.particles ])
        thetas =  np.array([ particle.theta for particle in particleSet.particles ])

        weights = np.array([ particle.weight for particle in particleSet.particles  ])
        
        self.x = np.sum( xs * weights ) / xs.shape
        self.y = np.sum( ys * weights ) / ys.shape
        self.theta = np.sum( thetas * weights ) / thetas.shape

class Line:
    
    # start and end are tuples (x, y) representing coordinates on the map
    def __init__(self, start, end):
         self.start = start
         self.end = end

    def distance_from_robot(RobotsPosition):

        Ax = self.start[0]
        Ay = self.start[1]
        Bx = self.end[0]
        By = self.end[1]


        x = RobotsPosition.x
        y = RobotsPosition.y
        cos = np.cos(math.radians(RobotsPosition.theta))
        sin = np.sin(math.radians(RobotsPosition.theta))

        # m is the distance formula from lecture slides
        m = ((By-Ay)*(Ax - x) - (Bx-Ax))/((By-Ay)*cos - (Bx-Ax)*sin)

        return m

    # returns boolean
    def line_valid(self, RobotsPosition):
        x1 = self.start[0]
        y1 = self.start[1]

        x2 = self.end[0]
        y2 = self.end[1]

        m1 = np.tan(RobotsPosition.theta * np.pi / 180)
        m2 = ( x1 - x2 ) / ( y1 - y2 )

        if m1 == m2:
            return False

        x = ( RobotsPosition.x * m1 - x1 * m2 - RobotsPosition.y + y1 ) / ( m1 - m2)
        # y = m1 * x - RobotsPosition.x * m1 + RobotsPosition.y
        print(RobotsPosition.theta)
        print(m2)
        if abs(x - x1) < abs(x1 - x2) and abs(x - x2) < abs(x1-x2):
            return True
        
        return False
    
    def __str__(self):
         return "({}, {}, {}, {})".format(self.start[0], self.start[1],
                                          self.end[0], self.end[1])

# map is a list of lines
def distance_to_shortest_valid_line(robotsPosition):
    shortest = 500 # higher than what the sensor can measure

    for line in mymap:
        if (line.line_valid(robotsPosition) and line.distance_from_robot(robotsPosition) < shortest ):
            shortest = line.distance_from_robot(robotsPosition)

    return shortest

def calculate_likelihood(particle, z, robotsPosition):

    sigma = 2#cm

    m = distance_to_shortest_valid_line(robotsPosition)
    return np.exp( - ( ( z - m  ) ** 2 ) / ( 2 * sigma ** 2  )  )

def main():

    #The letters correspond to the lines on the map
    lineA = ((0,0),(0,168))
    lineB = ((0,168),(84,168))
    lineC = ((84,126),(84,210))
    lineD = ((84,210),(168,210))
    lineE = ((168,210),(168,84))
    lineF = ((168,84),(210,84))
    lineG = ((210,84),(210,0))
    lineH = ((210,0),(0,0))
    
    mymap = [lineA, lineB, lineC, lineD, lineE, lineF, lineG, lineH]

    #Note: just for reference, delete after

    #mymap.add_wall((0,0,0,168));        # a
    #mymap.add_wall((0,168,84,168));     # b
    #mymap.add_wall((84,126,84,210));    # c
    #mymap.add_wall((84,210,168,210));   # d
    #mymap.add_wall((168,210,168,84));   # e
    #mymap.add_wall((168,84,210,84));    # f
    #mymap.add_wall((210,84,210,0));     # g
    #mymap.add_wall((210,0,0,0));        # h

    particle1 = Particle(0, 0, 0, 0.8)
    particle2 = Particle(100, 90, 180, 0.1)
    particle3 = Particle(90, 100, 180, 0.1)
    set = ParticleSet([particle1, particle2, particle3])
    r = RobotsPosition(10, 20, 45)

    r.robots_Position(set)

    line1 = Line( (50, 50), (90, 10) )
    print(line1.line_valid(r))
    '''
    try:
        mc.init_motors()
        particle_set = ParticleSet([Particle(10,10,0,1/NUM_OF_PARTICLES) for _ in range(NUM_OF_PARTICLES)])

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
    '''

if __name__ == "__main__":
    main()
