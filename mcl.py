import brickpi3
import math
import time
import random
import numpy as np
import copy
import motion_commands as mc

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)

NUM_OF_PARTICLES = 100

mymap=[] # list of lines

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

    def update(z, pose):

        for particle in self.particles:
            particle.weight *= calculateLikelihood(particle, z, pose)     


    def resample():
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
                    new_particle_set.append(Particle(particle.x, particle.y, 1/NUM_OF_PARTICLES))
                    break

                else:
                    running_total += particle.weight


        self.particles = new_particle_set


class RobotsPosition:

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

        d = math.sqrt(dx**2 + dy**2)

        return (beta, d)

    def navigate_to_waypoint(self, world_x, world_y, particle_set):
        angle_to_rotate, distance = self.calculate_motion(world_x, world_y)

        e, f, g = 2, 1, 2

        #print("turning: " + str(angle_to_rotate * 180/math.pi))
        #print("moving: " + str(distance))

        mc.turn(math.degrees(angle_to_rotate))
        particle_set.update_rotation_motions(math.degrees(angle_to_rotate), g)
        time.sleep(0.02)

        if(distance < 20):
            mc.drive_straight(distance)
            particle_set.update_linear_motions(distance, e, f)
        else:
            mc.drive_straight(20)
            particle_set.update_linear_motions(20, e, f)



        #print(self.x, self.y, math.degrees(self.theta))

    # sets self.x, self.y, self.theta to the  weighted average of the locations in particles, weighted by their corresponding weights
    def robots_Position(self, particleSet):
        xs = np.array([ particle.x for particle in particleSet.particles ])
        ys = np.array([ particle.y for particle in particleSet.particles ])
        thetas =  np.array([ particle.theta for particle in particleSet.particles ])

        weights = np.array([ particle.weight for particle in particleSet.particles  ])

        self.x = np.sum( xs * weights ) / xs.shape
        self.y = np.sum( ys * weights ) / ys.shape
        self.theta = np.sum( thetas * weights ) / thetas.shape

    def reached_waypoint(self, waypoint, tolerance):
        if(self.x >= waypoint[0]-tolerance and self.x <= waypoint[0]+tolerance and self.y >= waypoint[1]-tolerance and self.y <= waypoint[1]+tolerance):
            return True

        return False


class Line:

    # start and end are tuples (x, y) representing coordinates on the map
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def distance_from_robot(self, RobotsPosition):

        Ax = self.start[0]
        Ay = self.start[1]
        Bx = self.end[0]
        By = self.end[1]

        x = RobotsPosition.x
        y = RobotsPosition.y
        cos = np.cos(math.radians(RobotsPosition.theta))
        sin = np.sin(math.radians(RobotsPosition.theta))

        # m is the distance formula from lecture slides
        m = ((By-Ay)*(Ax - x) - (Bx-Ax)*(Ay-y))/((By-Ay)*cos - (Bx-Ax)*sin)
        return m

    # returns boolean
    def line_valid(self, RobotsPosition):
        x1 = self.start[0]
        y1 = self.start[1]

        x2 = self.end[0]
        y2 = self.end[1]

        m1 = np.tan((RobotsPosition.theta * np.pi) / 180)

        #a Handles vertical lines
        if ( x1 == x2 ):
            x = x1
            y = (m1 * x) - (m1 * RobotsPosition.x) + RobotsPosition.y

            segment_distance = abs(y1 - y2)
            if ( abs( y - y1 ) < segment_distance and abs( y - y2 ) < segment_distance ):
                return True
            else:
                return False

        m2 = ( y1 - y2 ) / ( x1 - x2 )

        if m1 == m2 :
            return False

        x = ( (RobotsPosition.x * m1) - (x1 * m2) - RobotsPosition.y + y1 ) / ( m1 - m2)

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
        #print(str(line)+' '+str(line.line_valid(robotsPosition)))
        if (line.line_valid(robotsPosition) and line.distance_from_robot(robotsPosition) < shortest and line.distance_from_robot(robotsPosition) > 0 ):
            shortest = line.distance_from_robot(robotsPosition)

    return shortest

def calculate_likelihood(particle, z, robotsPosition):

    sigma = 2#cm

    m = distance_to_shortest_valid_line(robotsPosition)
    return np.exp( - ( ( z - m  ) ** 2 ) / ( 2 * sigma ** 2  )  )

def get_sensor_reading():
    # read and display the sensor value
    # BP.get_sensor retrieves a sensor value.
    # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
    # BP.get_sensor returns the sensor value (what we want to display).
    try:
        value = BP.get_sensor(BP.PORT_1)
        print(value)                         # print the distance in CM

    except brickpi3.SensorError as error:
        print(error)

    return value


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


    waypoints = [(84,30), (180,30), (180,54), (138,54), (138,168), (114,168),(114,84),(84,84),(84,30)]


    starting_x = waypoints[0][0]
    starting_y = waypoints[0][1]


    # Draw the map
    pose = RobotsPosition(starting_x, starting_y, 0)


    for i in mymap:
        print("drawLine:" + str(i))

    particle_set = ParticleSet([Particle(starting_x, starting_y,0,1/NUM_OF_PARTICLES) for _ in range(NUM_OF_PARTICLES)])

    try:
        mc.init_motors()


        for i in waypoints[1:]:
            # Theres a bug that occurs if we dont sleep between
            # commands to the BrickPi. It doesnt register the
            # commands we send it and block. Hence the sleeps
            # in between the commands

            while (not pose.reached_waypoint(i, 5)):
                pose.navigate_to_waypoint(i[0], i[1], particle_set)
                distance_reading = get_sensor_reading() 
                particle_set.update(distance_reading, pose)
                particle_set.resample()
                pose.robots_Position(particle_set)

                print("drawParticles: " + str(particle_set))


    except KeyboardInterrupt:
        BP.reset_all() # This will prevent the robot moving if the program is interrupted or exited

if __name__ == "__main__":
    main()
