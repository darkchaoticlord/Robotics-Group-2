import brickpi3
import math
import time
import random
import numpy as np
import copy
import motion_commands as mc

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

    def turn_to_waypoint(self, world_x, world_y, particle_set):

        angle_to_rotate, distance = self.calculate_motion(world_x, world_y)
        print("turning: " + str(math.degrees(angle_to_rotate)))

        g = math.radians(2)

        mc.turn(math.degrees(angle_to_rotate))
        particle_set.update_rotation_motions(angle_to_rotate, g)
        time.sleep(0.02)

    def move_to_waypoint(self, world_x, world_y, particle_set):
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

    def distance_from_robot(self, RobotsPosition):

        Ax = self.start[0]
        Ay = self.start[1]
        Bx = self.end[0]
        By = self.end[1]

        x = RobotsPosition.x
        y = RobotsPosition.y
        cos = np.cos(RobotsPosition.theta)
        sin = np.sin(RobotsPosition.theta)

        # m is the distance formula from lecture slides
        m = ((By-Ay)*(Ax - x) - (Bx-Ax)*(Ay-y))/((By-Ay)*cos - (Bx-Ax)*sin)
        return m

    # returns boolean
    def line_valid(self, RobotsPosition):
        x1 = self.start[0]
        y1 = self.start[1]

        x2 = self.end[0]
        y2 = self.end[1]

        m1 = np.tan(RobotsPosition.theta)

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
        return "({}, {}, {}, {})".format(self.start[0] * scale_factor, self.start[1] * scale_factor,
                self.end[0] * scale_factor, self.end[1] * scale_factor)

        # map is a list of lines
def distance_to_shortest_valid_line(robotsPosition):
    shortest = 500 # higher than what the sensor can measure
    
    for line in mymap:
        #print(str(line)+' '+str(line.line_valid(robotsPosition)))
        if (line.line_valid(robotsPosition) and line.distance_from_robot(robotsPosition) < shortest and line.distance_from_robot(robotsPosition) > 0 ):
            shortest = line.distance_from_robot(robotsPosition)

    return shortest

def calculate_likelihood(particle, z):

    sigma = 0.5 #cm

    m = distance_to_shortest_valid_line(particle)
    return np.exp( - ( ( z - m  ) ** 2 ) / ( 2 * sigma ** 2  )  )

def get_sensor_reading():
    global first_time
    if(first_time):
        BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_ULTRASONIC)
        time.sleep(1)
        first_time = False
    # read and display the sensor value
    # BP.get_sensor retrieves a sensor value.
    # BP.PORT_1 specifies that we are looking for the value of sensor port 1.
    # BP.get_sensor returns the sensor value (what we want to display).
    values = []
    for i in range(10):
        try:
            values.append(BP.get_sensor(BP.PORT_1))

        except brickpi3.SensorError as error:
            print(error)

        time.sleep(0.02)
    
    
    return np.median(values) + 10


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

    particle_set = ParticleSet([Particle(starting_x, starting_y,0,1.0/NUM_OF_PARTICLES) for _ in range(NUM_OF_PARTICLES)])

    try:
        mc.init_motors()

        for num, (x,y) in enumerate(waypoints[1:]):
            # Theres a bug that occurs if we dont sleep between
            # commands to the BrickPi. It doesnt register the
            # commands we send it and block. Hence the sleeps
            # in between the commands

            print("Navigating to {}, {}".format(x,y))

            while (not pose.reached_waypoint(x,y,5)):
                print("Attempting nav to waypoint: " + str(num+2))


                # Turn and update
                pose.turn_to_waypoint(x,y, particle_set)
                pose.update_position(particle_set)
                distance_reading = get_sensor_reading()
                print("Sensor reading=" + str(distance_reading))
                    
                particle_set.update(distance_reading, pose)

                print("drawParticles: " + str(particle_set))
                particle_set.resample()
                pose.update_position(particle_set) # Update after observation+resampling
                time.sleep(1)

                print("drawParticles: " + str(particle_set))
                time.sleep(1)


                # Move straight and update
                pose.move_to_waypoint(x, y, particle_set)
                pose.update_position(particle_set) # Update using only motion uncertainty
                distance_reading = get_sensor_reading()
                print("Sensor reading=" + str(distance_reading))
                    
                particle_set.update(distance_reading, pose)

                print("drawParticles: " + str(particle_set))
                particle_set.resample()
                pose.update_position(particle_set) # Update after observation+resampling
                time.sleep(1) # Let particles get drawn

                print("drawParticles: " + str(particle_set))
                time.sleep(1)

                print(pose.x, pose.y, math.degrees(pose.theta))
                print()



    except KeyboardInterrupt:
        BP.reset_all() # This will prevent the robot moving if the program is interrupted or exited

if __name__ == "__main__":
    main()
