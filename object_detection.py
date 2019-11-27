# import brickpi3
import math
import time
import random
import numpy as np
import copy
import motion_commands as mc
from sensor_rotate import Sensor

BP = brickpi3.BrickPi3()
first_time = True
scale_factor = 3

NUM_OF_PARTICLES = 100

mymap=[] # list of lines

# Keep any angle we move through between -pi and pi
def normalizePi(val):
    while val <= -math.pi:
        val += 2*math.pi

    while val >= math.pi:
        val -= 2*math.pi

    return val

# Class representing the Particle in our particle filter
class Particle:

    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta # In radians
        self.weight = weight

    def update_linear_motion(self, D, std_dev_e, std_dev_f):
        self.x += (D + self.get_gaussian_error(std_dev_e)) * np.cos(self.theta)
        self.y += (D + self.get_gaussian_error(std_dev_e)) * np.sin(self.theta)
        self.theta = normalizePi((self.theta + self.get_gaussian_error(std_dev_f)))

    def update_rotation_motion(self, alpha, std_dev_g):
        self.theta = normalizePi((self.theta + alpha + self.get_gaussian_error(std_dev_g)))

    def get_gaussian_error(self, std_dev):
        return random.gauss(0, std_dev)

    def return_tuple(self):
        return (self.x * scale_factor, self.y * scale_factor, math.degrees(self.theta + math.pi))

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

    def update(self, z, pose):

        for particle in self.particles:
            t = calculate_likelihood(particle, z)
            particle.weight *= t

    def resample(self):
        new_particle_set = []
        sum_of_weights = 0

        for particle in self.particles:
            sum_of_weights += particle.weight

        # We can skip the normalization stage and instead sample from 0 to sum_of_weights

        # This runs in O(n^2), but we can use the Alias algorithm to make it in O(n)
        for i in range(NUM_OF_PARTICLES):
            prob = random.uniform(0, sum_of_weights)
            running_total = 0

            for particle in self.particles:
                running_total += particle.weight

                if prob <= running_total:
                    new_particle_set.append(Particle(particle.x, particle.y, particle.theta, 1.0/NUM_OF_PARTICLES))
                    break

                else:
                    running_total += particle.weight


        self.particles = new_particle_set

class RobotsPosition:

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta # In radians


    # Uses the position based path planning equations from lecture 2 to calculate the motion required to reach positon (world_x, world_y)
    # Returns 3 values: beta = angle to rotate through to point towards desired point,
    #                   d = distance to drive to reach desired point,
    def calculate_motion(self, world_x, world_y):
        dx = world_x - self.x
        dy = world_y - self.y

        alpha = math.atan2(dy, dx)

        beta = normalizePi(alpha - self.theta)

        d = math.sqrt(dx**2 + dy**2)

        return (beta, d)

    def turn_to(self, world_x, world_y, particle_set):

        angle_to_rotate, distance = self.calculate_motion(world_x, world_y)
        print("turning: " + str(math.degrees(angle_to_rotate)))

        g = math.radians(2)

        mc.turn(math.degrees(angle_to_rotate))
        particle_set.update_rotation_motions(angle_to_rotate, g)
        time.sleep(0.02)

    def move_to(self, world_x, world_y, particle_set):
        angle_to_rotate, distance = self.calculate_motion(world_x, world_y)

        e, f, g = 2, math.radians(1), math.radians(2)

        #print("turning: " + str(math.degrees(angle_to_rotate)))

        #mc.turn(math.degrees(angle_to_rotate))
        #particle_set.update_rotation_motions(angle_to_rotate, g)
        #time.sleep(0.02)

        if(distance < 20):
            mc.drive_straight(distance)
            print("moving: " + str(distance))
            particle_set.update_linear_motions(distance, e, f)
        else:
            mc.drive_straight(20)
            print("moving: " + str(20))
            particle_set.update_linear_motions(20, e, f)

        #print(self.x, self.y, math.degrees(self.theta))

    # sets self.x, self.y, self.theta to the  weighted average of the locations in particles, weighted by their corresponding weights
    def update_position(self, particleSet):
        xs = [ particle.x * particle.weight for particle in particleSet.particles ]
        ys = [ particle.y * particle.weight for particle in particleSet.particles ]
        theta_sin =  [ math.sin(particle.theta) for particle in particleSet.particles ]
        theta_cos =  [ math.cos(particle.theta) for particle in particleSet.particles ]

        self.x = sum(xs)
        self.y = sum(ys)
        self.theta = math.atan2(float(sum(theta_sin))/float(NUM_OF_PARTICLES), float(sum(theta_cos))/float(NUM_OF_PARTICLES))

    def reached_waypoint(self, x, y, tolerance):
        if(self.x >= x - tolerance and self.x <= x + tolerance and self.y >= y - tolerance and self.y <= y + tolerance):
            return True

        return False

class Line:

    # start and end are tuples (x, y) representing coordinates on the map
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def distance_from_robot(self, particle):

        Ax = self.start[0]
        Ay = self.start[1]
        Bx = self.end[0]
        By = self.end[1]

        x = particle.x
        y = particle.y
        cos = np.cos(particle.theta)
        sin = np.sin(particle.theta)

        # m is the distance formula from lecture slides
        m = ((By-Ay)*(Ax - x) - (Bx-Ax)*(Ay-y))/((By-Ay)*cos - (Bx-Ax)*sin)
        return m

    # returns boolean
    def line_valid(self, particle):
        x1 = self.start[0]
        y1 = self.start[1]

        x2 = self.end[0]
        y2 = self.end[1]

        m1 = np.tan(particle.theta)

        #a Handles vertical lines
        if ( x1 == x2 ):
            x = x1
            y = (m1 * x) - (m1 * particle.x) + particle.y

            segment_distance = abs(y1 - y2)
            if ( abs( y - y1 ) < segment_distance and abs( y - y2 ) < segment_distance ):
                return True
            else:
                return False

        m2 = ( y1 - y2 ) / ( x1 - x2 )

        if m1 == m2 :
            return False

        x = ( (particle.x * m1) - (x1 * m2) - particle.y + y1 ) / ( m1 - m2)

        if abs(x - x1) < abs(x1 - x2) and abs(x - x2) < abs(x1-x2):
            return True

        return False

    def __str__(self):
        return "({}, {}, {}, {})".format(self.start[0] * scale_factor, self.start[1] * scale_factor,
                self.end[0] * scale_factor, self.end[1] * scale_factor)

def distance_to_shortest_valid_line(particle):
    shortest = 500 # higher than what the sensor can measure

    for line in mymap:
        if (line.line_valid(particle) and line.distance_from_robot(particle) > 0 and line.distance_from_robot(particle) < shortest):
            shortest = line.distance_from_robot(particle)

    return shortest

def calculate_likelihood(particle, z):

    sigma = 0.5 #cm

    m = distance_to_shortest_valid_line(particle)
    return np.exp( - ( ( z - m  ) ** 2 ) / ( 2 * sigma ** 2  )  )

def getExpectedDistanceFromWall( particle ):
    return distance_to_shortest_valid_line( particle )

def environmentAnomaly( robotsPos, measurement ):
    return getExpectedDistanceFromWall( Particle( robotsPos.x, robotsPos.y, measurement[1], 0 ) ) - measurement[ 0 ] > 10

def addVectors( vec1, vec2 ):
    if len( vec1 ) != len( vec2 ):
        return None
    if vec1 == ():
        return ()

    return ( vec1[0] + vec2[0], ) + addVectors( vec1[ 1 : ], vec2[ 1 : ] )

def fromPolar( polarCoord ):
    d = polarCoord[ 0 ]
    theta = polarCoord[ 1 ]

    return ( d * np.cos( theta ), d * np.sin( theta ) )

def ValidArea( bottlePos, area ):
    bottlePos = ( int(bottlePos[0]), int(bottlePos[1]) )
    # Range of the three areas
    A_x = range(120, 210) #I have added 120 approximately
    A_y = range(0, 84)
    B_x = range(84, 168)
    B_y = range(84, 210)
    C_x = range(0, 84)
    C_y = range(40, 168) #I have added 40 approximately

    if area == 'A' and bottlePos[0] in A_x and bottlePos[1] in A_y:
        return True
    elif area == 'B' and bottlePos[0] in B_x and bottlePos[1] in B_y:
        return True
    elif area == 'C' and bottlePos[0] in C_x and bottlePos[1] in C_y:
        return True
    else:
        return False

# getBottleCoords :: ( [measurement], RobotsPosition, char ) -> position
# where:
# measurement = (distance, theta)
# position = (x, y)
# distance = float
# x = float
# y = float
# theta = float, 0 <= theta < 2 * pi
# continuous_count = int -> if continuous anomaly then return vector, default is zero

# NOTE:
    # The orientation of the robot is not taken into account in this function
    # Only the orientation of the measurements are used.
    # The angle of the measurements is expexted to be the angle from the x-axis
    # (not relative to the orientation of the robot)

def getBottleCoords( measurements, robotsPos, area, continuous_count=0 ):
    if len( measurements ) == 0:
        return ()
    if environmentAnomaly( robotsPos, measurements[ 0 ] ):
        continuous_count += 1
        if continuous_count == 3:
            bottlePos = addVectors( fromPolar( measurements[ 0 ] ), ( float(robotsPos.x), float(robotsPos.y) ) )
            print(fromPolar( measurements[ 0 ] ))
            print(( float(robotsPos.x), float(robotsPos.y) ))
            if ValidArea( bottlePos, area ):
                return bottlePos
            else:
                return getBottleCoords( measurements[ 1 : ], robotsPos, area )
        else:
            return getBottleCoords( measurements[ 1 : ], robotsPos, area, continuous_count )
    else:
        return getBottleCoords( measurements[ 1 : ], robotsPos, area )

def mcl_navigate(target_x, target_y, pose, sensor, particle_set):

        while (not pose.reached_waypoint(target_x, target_y, 5)):
            # Turn towards goal
            pose.turn_to_waypoint(x,y, particle_set)

            # Update where we think we are (based off motion uncertainty)
            pose.update_position(particle_set)

            # Get a sensor reading
            distances = sensor.spin_and_measure()

            # Update our particle set using the measurement
            particle_set.update(distance_reading, pose)

            #print("drawParticles: " + str(particle_set))

            # Resample the particles
            particle_set.resample()

            # Update where we think we are (based off the observation + resampling)
            pose.update_position(particle_set)

            #print("drawParticles: " + str(particle_set))

            # Move straight
            pose.move_to_waypoint(x, y, particle_set)

            # Update where we think we are (based off motion uncertainty)
            pose.update_position(particle_set)

            # Get a sensor reading
            distances = sensor.spin_and_measure()

            # Update the particle set using the measurement
            particle_set.update(distance_reading, pose)

            #print("drawParticles: " + str(particle_set))

            # Resample the particles
            particle_set.resample()

            # Update where we think we are (based off observation + resampling)
            pose.update_position(particle_set)

            #print("drawParticles: " + str(particle_set))


            # Print where the robot thinks it is after a single rotation + motion cycle
            print(pose.x, pose.y, math.degrees(pose.theta))
            print()

def bottleNavigation(pose, sensor, particleSet):

    areas = [ 'A', 'B', 'C' ]
    outsideAreasCoords = [ ( 120, 30 ), (120, 84), (84, 84) ]

    mcl_navigate( outsideAreasCoords[0][0], outsideAreasCoords[0][1], pose, sensor, particleSet )

    measurements = sensor.spin_and_measure()[ 90 : 270]

    bottleCoords = getBottleCoords( measurements, pose, 'A' )
    print(bottleCoords)

    mcl_navigate( bottleCoords[0], bottleCoords[1], pose, sensor, particleSet )

def main():
    # Seed our Random numbers
    random.seed()


    #The letters correspond to the lines on the map
    lineA = Line((0,0),(0,168))
    lineB = Line((0,168),(84,168))
    lineC = Line((84,126),(84,210))
    lineD = Line((84,210),(168,210))
    lineE = Line((168,210),(168,84))
    lineF = Line((168,84),(210,84))
    lineG = Line((210,84),(210,0))
    lineH = Line((210,0),(0,0))


    global mymap
    mymap = [lineA, lineB, lineC, lineD, lineE, lineF, lineG, lineH]

    # print(distance_to_shortest_valid_line(Particle(160,74, -0.674, 0)))
    # def getBottleCoords( measurements, robotsPos, area ):
    # print( getBottleCoords( [(80, -0.674), (80, -0.674), (40, -0.674), (40, -0.674), (40, -0.674) ], RobotsPosition(160,74, None ), 'A' ) )

    starting_x = 84
    starting_y = 30


    Draw the map
    for i in mymap:
        print("drawLine:" + str(i))

    # Setup all the necessary components
    pose = RobotsPosition(starting_x, starting_y, 0)
    sensor = Sensor(BP.PORT_C, BP.PORT_1)
    particle_set = ParticleSet([Particle(starting_x, starting_y,0,1.0/NUM_OF_PARTICLES) for _ in range(NUM_OF_PARTICLES)])
    mc.init_motors()

    try:
        bottleNavigation(pose, sensor, particle_set)

    except KeyboardInterrupt:
        BP.reset_all() # This will prevent the robot moving if the program is interrupted or exited

if __name__ == "__main__":
    main()
