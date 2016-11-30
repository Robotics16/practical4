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
max_dis_err = 10
particleConsensus = 0.87
max_sonar = 140
sonar_object_allowance = -2   # difference between bumper and sonar aswell as error allowance (cm)
scan_rotations = 10  # Scan for 180 degrees (advised to be divisible by 2)
scan_radius = 190


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

    sigma_offset   = 1                      # estimated sigma for 20 cms
    sigma_rotation = 5 * (math.pi / 180)   # about 15 degrees for 40 cms
    sigma_rotation_scaled = math.sqrt((sigma_rotation ** 2) * abs(d) / 40)
    sigma_offset_scaled   = math.sqrt((sigma_offset ** 2) * abs(d) / 20)
    mu = 0

    e = random.gauss(mu, sigma_offset) # error term for coordinate offset
    f = random.gauss(mu, sigma_rotation_scaled) # error term for rotation

    particle = ((old_x + (d + e) * math.cos(old_theta),
                 old_y + (d + e) * math.sin(old_theta),
                 old_theta + f),
                w)

    return particle

def updateOneParticleRotate(particle, angle):
    """Update particle triple with angle radians rotation,
    use gaussian distribution with mu=0 and estimated sigma"""
    ((old_x, old_y, old_theta), w) = particle

    sigma_rotation_90 = 6 * (math.pi / 180) # 10 estimated sigma for 90 degrees
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

# Returns (particles, hit)
def navigateToWaypoint(w_x, w_y, particles, interface, motors, max_d = -1):
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

    motor_util.rotate(beta, interface, motors)
    particles = updateParticlesRotate(particles, beta)
    #(nx, ny, ntheta) = getCurrentLocation(particles)
    #print " ACTUAL : " + str((beta+theta)*(180/math.pi)) + " ESTIMATE : " + str(ntheta*(180/math.pi))
    drawParticles(particles)

    sonar_value = getSonarValue(interface)
    (particles, _) = updateparticleswithsonar(particles, sonar_value, mymap)
    drawParticles(particles)

    # Move straight forward to waypoint
    distance_to_move = math.sqrt(d_x ** 2 + d_y ** 2) # distance to move using the Pythagorean theorem
    move_more = False
    
    if max_d != -1 and distance_to_move > max_d:
        distance_to_move = max_d
        move_more = True
    
    (hit, actual_distance) = motor_util.forward(distance_to_move, interface, motors)
    particles = updateParticlesForward(particles, actual_distance)
    if hit:
        motor_util.forward(objectBackMove, interface, motors)
        particles = updateParticlesForward(particles, objectBackMove)
    drawParticles(particles)

    if not hit:
        (particles, hit, _) = mcl_attack_if_evidence(particles, interface, motors)
            

    ret_value = (particles, hit)
    if move_more and not hit:
        ret_value = navigateToWaypoint(w_x, w_y, particles, interface, motors, max_d)

    return ret_value


def getSonarValue(interface):
    port = 3 # port which ultrasonic sensor is plugged in to
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


def updateparticleswithsonar(particles, measured_distance, map_geometry, sensor_angle=0, allow_update = True):
    """Updates all particle weights given the sonar measurement.
    Also normalizes and resamples the particle set"""
    updatedParticles = [] 
    objectConsensus = False

    for i in range(0, len(particles)):
        (updatedParticle, objectEvidence) = updateOneParticleWithSonar(particles[i], measured_distance, map_geometry, sensor_angle)
        updatedParticles.append((updatedParticle, objectEvidence))
              
    total_weight = 0
    total_evidence_weight = 0
    for ((_, w), ev) in updatedParticles:
        total_weight += w
        if ev:
            total_evidence_weight += w
    #print "EVW = " + str(w) + "   TW = " + str(w)
    consensus = total_evidence_weight / total_weight
    if consensus > particleConsensus:
        print "EVIDENCE OF OBJECT with consensus value: " + str(total_evidence_weight*100/total_weight) + "%"
        objectConsensus = True
    elif allow_update :
        particles = [p for (p, ev) in updatedParticles]
        particles = normalize_particles(particles)
        particles = resample_particles(particles)  

    return (particles, objectConsensus)

def updateOneParticleWithSonar(particle, measured_distance, map_geometry, sensor_angle = 0):
    """Updates a single particle weight based on the sonar measurement"""
    (pos, w) = particle

    if (measured_distance > max_sonar):
        return ((pos, w), False)
    
    (likelihood, objectEvidence) = calculate_likelihood(pos, measured_distance, map_geometry, sensor_angle)
    w = w * likelihood
    ret = (pos, w)
    return (ret, objectEvidence)

def calculate_likelihood(position, measured_distance, map_geometry, sensor_angle = 0):
    """Returns the probability of measuring `measured_distance` given the current
    position in the map"""

    (x, y, theta) = position
    theta += sensor_angle * (math.pi/180)
    walls = map_geometry.walls
    distances_wall = []
    objectEvidence = False

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
        return (0, False)

    minimum_distance = min(greater_than_0_distances)
    sigma = 2 # sonar error
    k = 0.001 # const to "lift" bell curve with

    dis_err = minimum_distance - measured_distance
    # print " M = " + str(measured_distance)
    if (dis_err > max_dis_err):
        objectEvidence = True
        
    probability = math.exp(-dis_err**2/(2*sigma**2)) + k # p(z|m) + k

    return (probability, objectEvidence)

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

def scan_area(particles, interface, motors):
    #(x, y, t) = getCurrentLocation(particles)
    #print "ORIENTATION BEFORE SCAN: " + str(t * (180/math.pi)) + " X: " + str(x) + "  Y: " + str(y)
    alpha = scan_radius / scan_rotations
    hit = False
    
    curr_sensor_angle = scan_radius/2 # IN DEGREES
    motor_util.rotate_sensor_deg(interface, curr_sensor_angle)
    
    for i in range(scan_rotations):
        (particles, hit, objectConsensus) = mcl_attack_if_evidence(particles, interface, motors, curr_sensor_angle, False)
        
        if hit:
            return (particles, True)
        elif objectConsensus:
            return scan_area(particles, interface, motors)
        
        motor_util.rotate_sensor_deg(interface, -alpha)
        curr_sensor_angle -= alpha
    
    print "SCAN FAILEDDDDDDDDDDDDDDD :(" 
    motor_util.rotate_sensor_deg(interface, scan_radius/2)
    #(x, y, t) = getCurrentLocation(particles)
    #print "ORIENTATION AFTER SCAN: " + str(t * (180/math.pi)) + " X: " + str(x) + "  Y: " + str(y)
    
    return (particles, False)

def mcl_attack_if_evidence(particles, interface, motors, alpha = 0 , allow_update = True):
    # take sonar measurements (take 5 get median)
    sonar_value = getSonarValue(interface)
    hit = False

    # update probabilities with sonar distance
    (particles, objectConsensus) = updateparticleswithsonar(particles, sonar_value, mymap, alpha, allow_update)
    drawParticles(particles)
        
    # if there is a consensus of their being a object infront of robot, attack it 
    if objectConsensus:
        distance_to_move = sonar_value + sonar_object_allowance
        if abs(alpha) > 0:
            motor_util.rotate_deg(alpha, interface, motors)
            particles = updateParticlesRotate(particles, alpha * (math.pi/180))
            drawParticles(particles)
            motor_util.rotate_sensor_deg(interface, -alpha)
               
        (hit, actual_distance) = motor_util.forward(distance_to_move, interface, motors)
        particles = updateParticlesForward(particles, actual_distance)
        if hit:
            motor_util.forward(objectBackMove, interface, motors)
            particles = updateParticlesForward(particles, objectBackMove)
        drawParticles(particles)    
        # If not hit, then attack failed and it is up to caller to handle the failure
    
    return (particles, hit, objectConsensus)
        

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
    (particles, _) = navigateToWaypoint(180, 30, particles, interface, motors, 20)  # 1
    (particles, _) = navigateToWaypoint(180, 54, particles, interface, motors, 20)  # 2
    (particles, _) = navigateToWaypoint(138, 54, particles, interface, motors, 20)  # 3
    (particles, _) = navigateToWaypoint(138, 168, particles, interface, motors,20) # 4
    (particles, _) = navigateToWaypoint(114, 168, particles, interface, motors,20) # 5
    (particles, _) = navigateToWaypoint(114, 84, particles, interface, motors ,20)  # 6
    (particles, _) = navigateToWaypoint(84, 84, particles, interface, motors  ,20)   # 7
    (particles, _) = navigateToWaypoint(84, 30, particles, interface, motors  ,20)   # 8

    #interface.stopLogging('logfile.txt')
    interface.terminate()


if __name__ == "__main__":
    main()
