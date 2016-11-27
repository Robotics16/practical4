import brickpi
import time
import motor_util
from motor_util import motors
import sys
import random
import math
import numpy as np
from particleDataStructuresExample import mymap, canvas

startPos = 0.0
objectBackMove = -10
max_dis_err = 20**2
particleConcensus = 60
max_sonar = 120

def drawParticles(particles):
    # offset = 300   # Offset used to draw on a sensible location on the screen
    # print "drawParticles:" + str([(x + offset, y + offset, th, w) for ((x, y, th), w) in particles])
    canvas.drawParticles([(x, y, th, w) for ((x, y, th), w) in particles])

def updateParticlesForward(particles, d):
    """Update all particles with d cm forward motion,
    based on current theta for each."""

    for i in range(0, len(particles)):
        updatedParticle = updateOneParticleForward(particles[i], d)
        particles[i] = updatedParticle

    return particles

def updateParticlesRotate(particles, angle):
    """Update all particles with a certain angle"""

    for i in range(0, len(particles)):
        updatedParticle = updateOneParticleRotate(particles[i], angle)
        particles[i] = updatedParticle

    return particles

def updateOneParticleForward(particle, d):
    """Update particle triple with d cm forward motion,
    use gaussian distribution with mu = 0 and estimated sigma"""
    ((old_x, old_y, old_theta), w) = particle

    sigma_offset   = 0.004 #0.004 #0.02   #0.02**2        # estimated sigma for 10 cms (scaled with variance for longer distances) TODO
    sigma_rotation = 0.1*math.pi/256 #0.01 #0.100    # about 15 degrees for 114 cms
    sigma_rotation_scaled = sigma_rotation #math.sqrt((sigma_rotation ** 2) * d / 114)
    mu = 0

    e = random.gauss(mu, sigma_offset) # error term for coordinate offset
    f = random.gauss(mu, sigma_rotation) # error term for rotation

    particle = ((old_x + (d + e) * math.cos(old_theta),
                 old_y + (d + e) * math.sin(old_theta),
                 old_theta + f),
                w)

    return particle

def updateOneParticleRotate(particle, angle):
    """Update particle triple with angle radians rotation,
    use gaussian distribution with mu=0 and estimated sigma"""
    ((old_x, old_y, old_theta), w) = particle

    sigma_rotation_90 = 0.008 # estimated sigma for 90 degrees
    mu = 0

    sigma_rotation = sigma_rotation_90 #math.sqrt((sigma_rotation_90 ** 2) * abs(angle) / (math.pi / 2.0)) # scaled sigma based on the estimation and the actual angle
    g = random.gauss(mu, sigma_rotation) # error term for pure rotation

    particle = ((old_x, old_y, old_theta + angle + g), w)
    return particle


def getCurrentLocation(particles):
    """Given all particles returns an estimate of the
    current position (x, y, theta)"""
    estimates = [(x * weight, y * weight, theta * weight) for ((x, y, theta), weight) in particles]

    total_weight = sum([weight for ((x, y, theta), weight) in particles])

    x_estimate     = sum([e[0] for e in estimates]) / total_weight
    y_estimate     = sum([e[1] for e in estimates]) / total_weight
    theta_estimate = sum([e[2] for e in estimates]) / total_weight

    return (x_estimate, y_estimate, theta_estimate)

def navigateToWaypoint(w_x, w_y, particles, interface, motors):
    return navigateToWaypointCrawl(w_x, w_y, particles, interface, motors, -1)

# Returns (particles, hit)
def navigateToWaypointCrawl(w_x, w_y, particles, interface, motors, max_d):
    """Using the current location returned by getCurrentLocation()
    navigates the robot to (w_x, w_y) (coordinates in the Wold coordinate system)"""
    (x, y, theta) = getCurrentLocation(particles)

    # Get vector between current and next position
    (d_x, d_y) = (w_x - x, w_y - y)

    # Turn on the spot in the direction of the waypoint
    alpha = math.atan2(d_y, d_x) # absolute orientation needed (using atan2 so that the result is between -pi and pi)
    beta  = (alpha - theta) # angle to turn
    if (abs(beta) > math.pi) :
        if (beta > 0) :
            beta = beta - 2 * math.pi
        else :
            beta = beta + 2 * math.pi

    motor_util.rotate(-beta, interface, motors)
    particles = updateParticlesRotate(particles, beta)
    drawParticles(particles)

    sonar_value = getSonarValue(interface)
    (particles, objectConsensus) = updateparticleswithsonar(particles, sonar_value, mymap)
    drawParticles(particles)


    # Move straight forward to waypoint
    distance_to_move = math.sqrt(d_x ** 2 + d_y ** 2) # distance to move using the Pythagorean theorem
    move_more = False
    
    if max_d != -1 and distance_to_move > max_d:
        distance_to_move = max_d
        move_more = True
    
    (hit, actual_distance) = motor_util.forward(distance_to_move, interface, motors)
    if hit:
        motor_util.forward(objectBackMove, interface, motors)
        actual_distance += objectBackMove
    particles = updateParticlesForward(particles, actual_distance)
    drawParticles(particles)

    if not hit:
        # take sonar measurements (take 5 get median)
        sonar_value = getSonarValue(interface)

        # update probabilities with sonar distance
        (particles, objectConsensus) = updateparticleswithsonar(particles, sonar_value, mymap)
        drawParticles(particles)

    ret_value = (particles, hit)
    if move_more and not hit:
        ret_value = navigateToWaypointCrawl(w_x, w_y, particles, interface, motors, max_d)

    return ret_value


def getSonarValue(interface):
    port = 3 # port which ultrasoic sensor is plugged in to
    interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)
    isReading = interface.getSensorValue(port)
    
    while not isReading:
        isReading = interface.getSensorValue(port)

    data_list = []
    length = 10

    for _ in range(0, length):
        data_list.append(isReading[0])

    data_list.sort()
    
    sonar_value = (data_list[4] + data_list[5]) / 2

    print "SONAR: " + str(sonar_value)

    return sonar_value


def updateparticleswithsonar(particles, measured_distance, map_geometry):
    """Updates all particle weights given the sonar measurement.
    Also normalizes and resamples the particle set"""
    numberParticlesWithEvidence = 0
    updatedParticles = [] 
    objectConsensus = False

    for i in range(0, len(particles)):
        (updatedParticle, objectEvidence) = updateOneParticleWithSonar(particles[i], measured_distance, map_geometry)
        updatedParticles.append(updatedParticle)
        
        if objectEvidence:
            numberParticlesWithEvidence += 1
            
    
    if numberParticlesWithEvidence > particleConcensus:
        print "EVIDENCE OF OBJECT with consensus: " + str(numberParticlesWithEvidence)
        objectConsensus = True
    else:
        particles = updatedParticles
        particles = normalize_particles(particles)
        particles = resample_particles(particles)
    
    

    return (particles, objectConsensus)

def updateOneParticleWithSonar(particle, measured_distance, map_geometry):
    """Updates a single particle weight based on the sonar measurement"""
    (pos, w) = particle

    if (measured_distance > max_sonar):
        return (particle, False)
    
    likelihood = calculate_likelihood(pos, measured_distance, map_geometry)
    w = w * likelihood
    particle = (pos, w)
    objectEvidence = likelihood == 1
    return (particle, objectEvidence)

def calculate_likelihood(position, measured_distance, map_geometry):
    """Returns the probability of measuring `measured_distance` given the current
    position in the map"""

    (x, y, theta) = position
    walls = map_geometry.walls
    distances_wall = []

    for wall in walls:
        (a_x, a_y, b_x, b_y) = wall
        m = ((b_y - a_y) * (a_x - x) - (b_x - a_x) * (a_y - y)) / ((b_y - a_y) * math.cos(theta) - (b_x - a_x) * math.sin(theta))
        (intersection_x, intersection_y) = (x + m*math.cos(theta), y + m*math.sin(theta))

        is_between_x = (a_x <= intersection_x and intersection_x <= b_x) or (b_x <= intersection_x and intersection_x <= a_x)
        is_between_y = (a_y <= intersection_y and intersection_y <= b_y) or (b_y <= intersection_y and intersection_y <= a_y)

        if (is_between_x and is_between_y):
            distances_wall.append(m)


    greater_than_0_distances = [x for x in distances_wall if x >= 0]

    if not greater_than_0_distances:
        return 0

    minimum_distance = min(greater_than_0_distances)
    sigma = 2 # 0.3 # 0.3 sonar error
    k = 0.001 # const to "lift" bell curve with

    dis_err = (measured_distance-minimum_distance)**2
    if dis_err > max_dis_err:
        probability = 1
    else:
        probability = math.exp(-dis_err/(2*sigma**2)) + k # p(z|m) + k

    return probability

def normalize_particles(particles):
    """Normalizes particle set, such that the weights of all particles add up to 1"""
    total_weight = sum([w for (pos, w) in particles])
    particles = [(pos, w / total_weight) for (pos, w) in particles]
    return particles

def resample_particles(particles):
    """Resamples normalized particle set: samples len(particles) number of particles from
    the provided particle set, where the chance of picking a particle during a sampling is the
    weight of the particle"""

    pdf = [w for (p,w) in particles]
    indices = list(np.random.choice(len(particles), len(particles), p=pdf))

    out = []

    for i in indices:
        out.append(particles[i])

    particles = out
    return out

def main():
    interface = brickpi.Interface()
    #interface.startLogging('logfile.txt')
    interface.initialize()

    # Setup motors
    motor_util.setUpInterface(interface)

    # Setup initial state of particles
    numberOfParticles = 100
    waypoint1_x = 84
    waypoint1_y = 30
    particles = [((waypoint1_x, waypoint1_y, 0), 1 / float(numberOfParticles)) for i in range(numberOfParticles)]

    drawParticles(particles)

    # Assume robot at waypoint 0 : (84, 30)
    # Assume no obstacles on the course
    (particles, _) = navigateToWaypointCrawl(180, 30, particles, interface, motors, 20)  # 1
    (particles, _) = navigateToWaypointCrawl(180, 54, particles, interface, motors, 20)  # 2
    (particles, _) = navigateToWaypointCrawl(138, 54, particles, interface, motors, 20)  # 3
    (particles, _) = navigateToWaypointCrawl(138, 168, particles, interface, motors,20) # 4
    (particles, _) = navigateToWaypointCrawl(114, 168, particles, interface, motors,20) # 5
    (particles, _) = navigateToWaypointCrawl(114, 84, particles, interface, motors ,20)  # 6
    (particles, _) = navigateToWaypointCrawl(84, 84, particles, interface, motors  ,20)   # 7
    (particles, _) = navigateToWaypointCrawl(84, 30, particles, interface, motors  ,20)   # 8

    #interface.stopLogging('logfile.txt')
    interface.terminate()


if __name__ == "__main__":
    main()
